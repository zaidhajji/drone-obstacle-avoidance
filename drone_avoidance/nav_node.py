#!/usr/bin/env python3
"""A* global plan + waypoint following + lidar repulsion + run metrics."""
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from drone_avoidance.planner import GridMap, astar, smooth


class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        for name, val in [('map_file', ''), ('v_max', 1.0), ('influence', 1.2), ('k_rep', 1.5),
                          ('inflation', 0.8), ('resolution', 0.25), ('goal_tol', 0.4),
                          ('collision_dist', 0.35)]:
            self.declare_parameter(name, val)
        p = lambda n: self.get_parameter(n).value
        with open(p('map_file')) as f:
            cfg = yaml.safe_load(f)
        self.gm = GridMap(cfg, p('resolution'), p('inflation'))
        self.goal = tuple(cfg['goal'])

        self.pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.pub_plan = self.create_publisher(
            Path, '/drone/plan', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/drone/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/drone/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.loop)

        self.pos = self.last = self.scan = self.path = self.t0 = None
        self.i = 0
        self.done = False
        self.length = 0.0
        self.min_range = float('inf')
        self.close_calls = 0
        self.in_close = False

    def on_odom(self, m):
        self.pos = (m.pose.pose.position.x, m.pose.pose.position.y)
        if (self.done and self.path is not None and
                math.hypot(self.goal[0] - self.pos[0], self.goal[1] - self.pos[1]) > 2.0):
            self.get_logger().info('Reset detected: replanning')
            self.path, self.done, self.i = None, False, 0
            self.length, self.min_range, self.close_calls = 0.0, float('inf'), 0
            self.last = None
        if self.last is not None and math.hypot(self.pos[0] - self.last[0],
                                                self.pos[1] - self.last[1]) > 3.0:
            self.get_logger().info('Reset detected: replanning')
            self.path, self.done, self.i = None, False, 0
            self.length, self.min_range, self.close_calls = 0.0, float('inf'), 0
            self.last = None
        if self.last is not None:
            self.length += math.hypot(self.pos[0] - self.last[0], self.pos[1] - self.last[1])
        self.last = self.pos
        if self.path is None:
            self.t0 = self.get_clock().now()
            raw = astar(self.gm, self.pos, self.goal)
            if raw is None:
                self.get_logger().error('No global path found')
                self.done = True
                return
            self.path = smooth(self.gm, raw)
            self.get_logger().info(f'Plan: {len(raw)} cells -> {len(self.path)} waypoints')
            msg = Path()
            msg.header.frame_id = 'odom'
            msg.header.stamp = self.get_clock().now().to_msg()
            for x, y in self.path:
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose.position.x, ps.pose.position.y = x, y
                msg.poses.append(ps)
            self.pub_plan.publish(msg)

    def on_scan(self, m):
        self.scan = m
        rs = [r for r in m.ranges if math.isfinite(r) and r >= m.range_min]
        if rs:
            mn = min(rs)
            self.min_range = min(self.min_range, mn)
            lim = self.get_parameter('collision_dist').value
            if mn < lim and not self.in_close:
                self.close_calls += 1
                self.in_close = True
            elif mn > 1.5 * lim:
                self.in_close = False

    def loop(self):
        if self.pos is None or self.scan is None or self.path is None:
            return
        if self.done:
            self.pub.publish(Twist())
            return
        p = lambda n: self.get_parameter(n).value
        gx, gy = self.goal
        d_goal = math.hypot(gx - self.pos[0], gy - self.pos[1])
        if d_goal < p('goal_tol'):
            self.done = True
            t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
            self.get_logger().info(
                f'GOAL REACHED | time {t:.1f} s | path {self.length:.1f} m | '
                f'min clearance {self.min_range:.2f} m | close calls {self.close_calls}')
            self.pub.publish(Twist())
            return

        # follow waypoints
        tx, ty = self.path[self.i]
        if math.hypot(tx - self.pos[0], ty - self.pos[1]) < 0.5 and self.i < len(self.path) - 1:
            self.i += 1
            tx, ty = self.path[self.i]
        dx, dy = tx - self.pos[0], ty - self.pos[1]
        n = math.hypot(dx, dy) or 1.0
        vx, vy = dx / n, dy / n

        # lidar repulsion (yaw stays 0 with planar_move, so body frame = world frame)
        infl, rx, ry, ang = p('influence'), 0.0, 0.0, self.scan.angle_min
        for r in self.scan.ranges:
            if math.isfinite(r) and r < infl:
                mag = (infl - r) / infl
                rx -= mag * math.cos(ang)
                ry -= mag * math.sin(ang)
            ang += self.scan.angle_increment
        vx += p('k_rep') * rx * self.scan.angle_increment
        vy += p('k_rep') * ry * self.scan.angle_increment

        speed = min(p('v_max'), 0.3 + 0.8 * d_goal)      # slow down near B
        norm = math.hypot(vx, vy)
        cmd = Twist()
        if norm > 1e-6:
            cmd.linear.x, cmd.linear.y = speed * vx / norm, speed * vy / norm
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(NavNode())


if __name__ == '__main__':
    main()
