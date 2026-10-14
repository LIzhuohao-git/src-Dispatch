#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""【已弃用，请改用 odom_calib.py --rotate】

本脚本要求人工把小车原地转 N 圈，然后只取一次首尾朝向差，且在 turns>1 时把
"单圈内的测量值"直接乘 turns —— 量纲上没有意义；人工旋转时轮子也不滚动，
/odom 与实测的对应关系并不可靠。

请改用：
    ros2 run originbot_base odom_calib.py --rotate

新工具会让小车【自己】原地转整圈，用激光雷达测出真实转角（独立于里程计），
并给出 correct_factor_vth 的建议值。本文件保留仅为兼容旧习惯，不再维护。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import subprocess
import sys


def quat_to_yaw(q):
    # q: geometry_msgs Quaternion-like with x,y,z,w
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

class YawMeasure(Node):
    def __init__(self):
        super().__init__('vth_calib_measure')
        self.sub = self.create_subscription(Odometry, 'odom', self.cb, 10)
        self.current_yaw = None

    def cb(self, msg: Odometry):
        self.current_yaw = quat_to_yaw(msg.pose.pose.orientation)


def get_ros2_param(node_name, param_name):
    try:
        out = subprocess.check_output(['ros2', 'param', 'get', node_name, param_name], stderr=subprocess.DEVNULL)
        s = out.decode('utf-8').strip()
        # output like: 'Parameter name: value' or just 'value'
        if ':' in s:
            parts = s.split(':', 1)
            val = parts[1].strip()
        else:
            val = s
        return float(val)
    except Exception:
        return None


def main(argv=None):
    rclpy.init()
    nm = YawMeasure()

    print('Waiting for first /odom message...')
    while rclpy.ok() and nm.current_yaw is None:
        rclpy.spin_once(nm, timeout_sec=0.1)

    start = nm.current_yaw
    print(f'Start yaw: {math.degrees(start):.3f}°')

    # get turns desired
    turns = 1
    if len(sys.argv) > 1:
        try:
            turns = int(sys.argv[1])
        except Exception:
            turns = 1
    expected = turns * 2.0 * math.pi
    print(f'Please rotate the robot in place by {turns} full turn(s) ({math.degrees(expected):.1f}°).')
    input('When done, press Enter to capture end yaw...')

    # wait briefly to get updated odom
    end = None
    for _ in range(50):
        rclpy.spin_once(nm, timeout_sec=0.05)
        if nm.current_yaw is not None:
            end = nm.current_yaw
    if end is None:
        print('No odom received after rotation, aborting.')
        rclpy.shutdown()
        return

    print(f'End yaw: {math.degrees(end):.3f}°')

    # compute smallest difference, then wrap to positive
    diff = end - start
    # normalize to (-pi, pi]
    diff = (diff + math.pi) % (2*math.pi) - math.pi
    if diff < 0:
        measured = diff + 2.0*math.pi
    else:
        measured = diff

    # if user rotated multiple turns, measured may be in (0,2pi); ask user to input actual turns if needed
    print(f'Measured rotation (single-wrap) = {math.degrees(measured):.3f}° ({measured:.6f} rad)')
    if turns > 1:
        # If user intended multiple turns, multiply
        measured_total = measured * turns
    else:
        measured_total = measured

    print(f'Expected rotation = {math.degrees(expected):.3f}° ({expected:.6f} rad)')

    old_factor = get_ros2_param('/originbot_base', 'correct_factor_vth')
    if old_factor is None:
        print('Could not read current parameter /originbot_base correct_factor_vth. Please provide old factor (or press Enter to use 1.0):')
        s = input().strip()
        try:
            old_factor = float(s) if s != '' else 1.0
        except Exception:
            old_factor = 1.0

    new_factor = old_factor * (expected / measured_total) if measured_total > 1e-9 else None

    if new_factor is None:
        print('Measured rotation too small, aborting.')
    else:
        print(f'Old correct_factor_vth = {old_factor:.6f}')
        print(f'Computed new correct_factor_vth = {new_factor:.6f}')
        print('\nTo apply the new factor run:')
        print(f'  ros2 param set /originbot_base correct_factor_vth {new_factor:.6f}')
        print('\nOr re-launch with the parameter updated in your launch file.')

    rclpy.shutdown()

if __name__ == "__main__":
    main()
