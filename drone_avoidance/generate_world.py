#!/usr/bin/env python3
"""Generate worlds/complex.world and config/map.yaml from ONE description."""
import argparse
import math
import os
import random
import sys

import yaml

ARENA = [-12.0, 12.0, -8.0, 8.0]
START = [-10.0, 0.0]
GOAL = [10.0, 0.0]
H = 3.0
# boxes: cx, cy, size_x, size_y   (three walls with alternating gaps: top, bottom, top)
WALLS = [[-5.0, -2.5, 0.4, 11.0], [0.0, 2.5, 0.4, 11.0], [5.0, -2.5, 0.4, 11.0]]
# cylinders: x, y, radius
CYLS = [[-8, 3, 0.6], [-8, -3, 0.6], [-2.5, 0, 0.7], [-3.5, -5.5, 0.6],
        [2.5, 0, 0.7], [2.5, 5.5, 0.6], [8, 2.5, 0.6], [8, -2.5, 0.6]]

DRONE = """
<model name="drone"><pose>@X@ @Y@ 1.5 0 0 0</pose>
<link name="base_link"><gravity>false</gravity>
<inertial><mass>1.0</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.02</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<visual name="body"><geometry><box><size>0.25 0.25 0.08</size></box></geometry><material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
<visual name="arm1"><pose>0 0 0 0 0 0.785</pose><geometry><box><size>0.7 0.04 0.03</size></box></geometry></visual>
<visual name="arm2"><pose>0 0 0 0 0 -0.785</pose><geometry><box><size>0.7 0.04 0.03</size></box></geometry></visual>
<visual name="r1"><pose>0.25 0.25 0.04 0 0 0</pose><geometry><cylinder><radius>0.12</radius><length>0.01</length></cylinder></geometry><material><ambient>0 0.5 1 1</ambient><diffuse>0 0.5 1 1</diffuse></material></visual>
<visual name="r2"><pose>-0.25 0.25 0.04 0 0 0</pose><geometry><cylinder><radius>0.12</radius><length>0.01</length></cylinder></geometry><material><ambient>0 0.5 1 1</ambient><diffuse>0 0.5 1 1</diffuse></material></visual>
<visual name="r3"><pose>0.25 -0.25 0.04 0 0 0</pose><geometry><cylinder><radius>0.12</radius><length>0.01</length></cylinder></geometry><material><ambient>0 0.5 1 1</ambient><diffuse>0 0.5 1 1</diffuse></material></visual>
<visual name="r4"><pose>-0.25 -0.25 0.04 0 0 0</pose><geometry><cylinder><radius>0.12</radius><length>0.01</length></cylinder></geometry><material><ambient>0 0.5 1 1</ambient><diffuse>0 0.5 1 1</diffuse></material></visual>
<sensor name="lidar" type="ray"><pose>0 0 0.06 0 0 0</pose><always_on>true</always_on><update_rate>10</update_rate><visualize>true</visualize>
<ray><scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
<range><min>0.2</min><max>6.0</max><resolution>0.01</resolution></range></ray>
<plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so"><ros><namespace>/drone</namespace><remapping>~/out:=scan</remapping></ros>
<output_type>sensor_msgs/LaserScan</output_type><frame_name>base_link</frame_name></plugin></sensor>
</link>
<plugin name="drone_move" filename="libgazebo_ros_planar_move.so"><ros><namespace>/drone</namespace></ros>
<update_rate>50</update_rate><publish_rate>20</publish_rate><publish_odom>true</publish_odom><publish_odom_tf>true</publish_odom_tf>
<odometry_frame>odom</odometry_frame><robot_base_frame>base_link</robot_base_frame></plugin>
</model>
"""


def box_sdf(name, x, y, sx, sy, color):
    return (f"<model name='{name}'><static>true</static><pose>{x} {y} {H/2} 0 0 0</pose><link name='l'>"
            f"<collision name='c'><geometry><box><size>{sx} {sy} {H}</size></box></geometry></collision>"
            f"<visual name='v'><geometry><box><size>{sx} {sy} {H}</size></box></geometry>"
            f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>\n")


def cyl_sdf(name, x, y, r):
    return (f"<model name='{name}'><static>true</static><pose>{x} {y} {H/2} 0 0 0</pose><link name='l'>"
            f"<collision name='c'><geometry><cylinder><radius>{r}</radius><length>{H}</length></cylinder></geometry></collision>"
            f"<visual name='v'><geometry><cylinder><radius>{r}</radius><length>{H}</length></cylinder></geometry>"
            f"<material><ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material></visual></link></model>\n")


def random_cylinders(n, seed, fixed):
    rng, out, tries = random.Random(seed), [], 0
    while len(out) < n and tries < 2000:
        tries += 1
        x, y, r = rng.uniform(-10.5, 10.5), rng.uniform(-6.5, 6.5), rng.uniform(0.3, 0.6)
        if math.hypot(x - START[0], y - START[1]) < 2.5 or math.hypot(x - GOAL[0], y - GOAL[1]) < 2.5:
            continue
        if any(abs(x - w[0]) < 1.8 for w in WALLS):          # keep the gaps clear
            continue
        if any(math.hypot(x - c[0], y - c[1]) < r + c[2] + 1.6 for c in fixed + out):
            continue
        out.append([round(x, 2), round(y, 2), round(r, 2)])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='.', help='package source folder')
    ap.add_argument('--random', type=int, default=0, help='extra random cylinders')
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()

    cyls = CYLS + random_cylinders(a.random, a.seed, CYLS)
    cfg = {'arena': ARENA, 'start': START, 'goal': GOAL, 'boxes': WALLS, 'cylinders': cyls}

    try:
        from drone_avoidance.planner import GridMap, astar
    except ImportError:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from planner import GridMap, astar
    if astar(GridMap(cfg), START, GOAL) is None:
        sys.exit('No path from A to B with these obstacles: change --seed or --random')

    s = ""
    xmin, xmax, ymin, ymax = ARENA
    for k, (x, y, sx, sy) in enumerate([[0, ymax + .2, xmax - xmin + .8, .4], [0, ymin - .2, xmax - xmin + .8, .4],
                                        [xmax + .2, 0, .4, ymax - ymin + .8], [xmin - .2, 0, .4, ymax - ymin + .8]]):
        s += box_sdf(f'border{k}', x, y, sx, sy, '0.3 0.3 0.3 1')
    for k, w in enumerate(WALLS):
        s += box_sdf(f'wall{k}', *w, '0.6 0.6 0.7 1')
    for k, c in enumerate(cyls):
        s += cyl_sdf(f'obs{k}', *c)
    s += (f"<model name='goal'><static>true</static><pose>{GOAL[0]} {GOAL[1]} 0.02 0 0 0</pose><link name='l'>"
          "<visual name='v'><geometry><cylinder><radius>0.6</radius><length>0.04</length></cylinder></geometry>"
          "<material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material></visual></link></model>\n")
    s += DRONE.replace('@X@', str(START[0])).replace('@Y@', str(START[1]))

    world = ('<?xml version="1.0"?>\n<sdf version="1.6"><world name="complex">\n'
             '<include><uri>model://ground_plane</uri></include>\n<include><uri>model://sun</uri></include>\n'
             + s + '</world></sdf>\n')
    os.makedirs(os.path.join(a.out, 'worlds'), exist_ok=True)
    os.makedirs(os.path.join(a.out, 'config'), exist_ok=True)
    with open(os.path.join(a.out, 'worlds', 'complex.world'), 'w') as f:
        f.write(world)
    with open(os.path.join(a.out, 'config', 'map.yaml'), 'w') as f:
        yaml.safe_dump(cfg, f)
    print(f'OK: {len(cyls)} cylinders, {len(WALLS)} walls -> worlds/complex.world + config/map.yaml')


if __name__ == '__main__':
    main()
