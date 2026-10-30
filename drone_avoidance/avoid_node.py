#!/usr/bin/env python3
"""Reactive obstacle avoidance: goal attraction + lidar repulsion + tangential slide."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class AvoidNode(Node):
    def __init__(self):
        super().__init__('avoid_node')
        # Goal expressed relative to the start position (A -> B = 16 m along x)
        self.declare_parameter('goal_dx', 16.0)
        self.declare_parameter('goal_dy', 0.0)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('influence', 3.0)   # obstacle influence distance (m)
        self.declare_parameter('k_rep', 1.5)
        self.declare_parameter('k_tan', 1.2)

        self.pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.create_subscription(Odometry, '/drone/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/drone/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.loop)

        self.pos = None
        self.goal = None
        self.scan = None
        self.done = False

    def on_odom(self, m):
        self.pos = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.goal is None:
            self.goal = (self.pos[0] + self.get_parameter('goal_dx').value,
                         self.pos[1] + self.get_parameter('goal_dy').value)
            self.get_logger().info(f'Start {self.pos}, goal {self.goal}')

    def on_scan(self, m):
        self.scan = m

    def loop(self):
        if self.pos is None or self.scan is None or self.goal is None:
            return
        cmd = Twist()
        dx, dy = self.goal[0] - self.pos[0], self.goal[1] - self.pos[1]
        dist = math.hypot(dx, dy)

        if dist < 0.4:
            if not self.done:
                self.get_logger().info('Point B reached!')
                self.done = True
            self.pub.publish(cmd)
            return

        v_max = self.get_parameter('v_max').value
        infl = self.get_parameter('influence').value
        k_rep = self.get_parameter('k_rep').value
        k_tan = self.get_parameter('k_tan').value

        ax, ay = dx / dist, dy / dist            # attraction (unit vector)
        rx = ry = 0.0
        hits = 0
        closest, closest_ang = float('inf'), 0.0
        ang = self.scan.angle_min
        for r in self.scan.ranges:
            if math.isfinite(r) and r < infl:
                w = (1.0 / r - 1.0 / infl) / r
                rx -= w * math.cos(ang)
                ry -= w * math.sin(ang)
                hits += 1
                if r < closest:
                    closest, closest_ang = r, ang
            ang += self.scan.angle_increment

        vx, vy = ax, ay
        if hits:
            vx += k_rep * rx / hits
            vy += k_rep * ry / hits
            # slide around the nearest obstacle, on the side closest to the goal
            tx, ty = -math.sin(closest_ang), math.cos(closest_ang)
            if tx * ax + ty * ay < 0:
                tx, ty = -tx, -ty
            k = (infl - closest) / infl
            vx += k_tan * k * tx
            vy += k_tan * k * ty

        n = math.hypot(vx, vy)
        if n > 1e-6:
            cmd.linear.x = v_max * vx / n
            cmd.linear.y = v_max * vy / n
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(AvoidNode())


if __name__ == '__main__':
    main()
