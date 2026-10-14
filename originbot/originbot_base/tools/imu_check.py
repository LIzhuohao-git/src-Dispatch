#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IMU 安装与数据正确性检测工具（OriginBot 底盘固件 IMU）

【用途】
  判断底盘 IMU 是否装对、数据是否可信，并给出建议的安装位姿修正值。
  覆盖七项：
    1. 话题/坐标系是否规范（frame_id 应为 imu_link）
    2. 加计换算是否正确（静止时加速度模长应 ≈ 9.81）
    3. 【安装朝向】—— 用静止比力与静态 TF 交叉验证
    4. 陀螺零偏/噪声是否超标（影响建图与 EKF 融合质量）
    5. 姿态输出可用性（协方差是否声明为不可用）
    6. 【旋转一致性】—— 陀螺积分转角 vs 里程计转角 vs 雷达真值（--rotate 启用）
    7. 【陀螺符号】—— 陀螺角速度符号与配置的安装 RPY 是否自洽（--rotate 启用）

【原理】
  小车【水平静止】时，加速度计测到的是『比力』（地面支撑力），方向沿
  base_link 的 +Z（向上）。换算到 IMU 自身坐标系应为 Rᵀ·(0, 0, +g)，
  其中 R 是 base_link→imu_link 的安装旋转（imu.json 里的 roll/pitch/yaw）。
  把实测比力与按 TF 推算的方向比对，即得安装朝向是否与声明一致。

  ⚠ 静止比力只能定出 roll/pitch，且存在【绕 roll 翻转 180° 的固有二义性】：
    水平装读 +g、翻转装读 -g，两种装法各自的读数都自洽。因此本工具用
    min(夹角, 180°-夹角) 作为偏差量，并靠第 6/7 项旋转检测彻底定性。

【第 6 项是脚本自己驱动小车转的】
  脚本用 /cmd_vel 让小车【自行原地转整圈】，过程中连续采 /imu，并与
  /odom 的转角、激光雷达实测的转角三方比对：
      陀螺/里程计  —— 两者都来自底盘，反映综合一致性
      陀螺/雷达    —— 隔离出 IMU 的刻度误差（雷达是独立传感器）
      里程计/雷达  —— 就是 correct_factor_vth 的刻度误差
  必须自己驱动而不是手推：手推时轮子不滚动，/odom 几乎为 0，比值没有意义。

【前置条件】
  - 底盘已启动：ros2 launch originbot_bringup originbot.launch.py
  - 第 1~5 项要求小车【水平静止】；第 6 项要求四周 0.5m 内无障碍
  - 已 source 环境：source /opt/ros/humble/setup.bash && source install/setup.bash

【用法】
  ros2 run originbot_base imu_check.py                # 静态检测（默认）
  ros2 run originbot_base imu_check.py --seconds 10
  ros2 run originbot_base imu_check.py --rotate       # 额外自行转 2 圈做一致性检测

【判读标准】
  加速度模长      9.81 ± 0.5 m/s²           判为正常
  安装朝向偏差    min(夹角,180°-夹角) < 5°  判为与声明一致
                  （roll 的 180° 二义性需靠第 6/7 项消解）
  陀螺零偏        < 0.02 rad/s（≈1.1°/s）   判为正常
  陀螺噪声(σ)     < 0.05 rad/s              判为正常
  旋转一致性      陀螺/雷达 应在 1.0±0.05（雷达是独立真值）
                  陀螺/里程计 1.0±0.15；里程计/雷达 即 vth 系数误差
  陀螺符号        应与 sign(cos(pitch)·cos(roll)) 一致（见第 7 项说明）

【退出码】0 = 无问题；1 = 未收到 /imu；2 = 发现需处理的问题
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

try:
    import calib_common as cc
    HAVE_COMMON = True
except ImportError:                     # 单文件被拷走时仍可用（只是没有第 6/7 项）
    HAVE_COMMON = False

G = 9.81
GENERATED_DIR = Path.home() / ".originbot_delivery" / "config" / "generated"

# 与 robot.launch.py IMU_DEFAULTS / robot_params.py imu 组保持一致
IMU_DEFAULTS = {
    "x": -0.05, "y": 0.10, "z": 0.05,
    "roll": 3.1416, "pitch": 0.0, "yaw": 1.5708,
}

# 判据阈值
ACC_NORM_TOL = 0.5        # 加速度模长容差 m/s²
GYRO_BIAS_MAX = 0.02      # 陀螺零偏上限 rad/s
GYRO_NOISE_MAX = 0.05     # 陀螺噪声上限 rad/s
GRAVITY_OK_DEG = 5.0      # 与声明朝向的允许偏差（比较 min(夹角, 180°-夹角)）

# 底盘 /imu 用默认 reliable QoS
IMU_QOS = QoSProfile(depth=10)


def load_imu_config():
    """读取 Web 端参数配置页生成的 imu.json（缺文件回落出厂默认值）。"""
    config = dict(IMU_DEFAULTS)
    try:
        with open(GENERATED_DIR / "imu.json", "r", encoding="utf-8") as handle:
            stored = json.load(handle)
    except (OSError, ValueError):
        return config
    for key in config:
        if key in stored:
            try:
                config[key] = float(stored[key])
            except (TypeError, ValueError):
                pass
    return config


def quat_to_rotmat(x, y, z, w):
    """四元数 → 3x3 旋转矩阵。"""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_rotmat(roll, pitch, yaw):
    """RPY（弧度，ZYX 顺序）→ 旋转矩阵，与 tf2 的 setRPY 一致。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def angle_between_deg(a, b):
    """两个向量的夹角（度）。"""
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float("nan")
    cos = float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))
    return math.degrees(math.acos(cos))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


_BaseNode = cc.MotionDriver if HAVE_COMMON else Node


class ImuSampler(_BaseNode):
    """采样 /imu。

    有 calib_common 时基类是 MotionDriver，因此顺带具备 /odom 累计与闭环驱动
    （第 6/7 项要脚本自己把小车转起来）；没有时退化为纯 Node，只做静态检测。
    """

    def __init__(self):
        super().__init__("imu_check")
        self.create_subscription(Imu, "/imu", self._on_imu, IMU_QOS)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.imu_msgs = []
        self.stamps = []
        self.grabber = cc.ScanGrabber(self) if HAVE_COMMON else None
        if not HAVE_COMMON:
            # MotionDriver 已经订阅了 /odom；这条分支只在缺 calib_common 时生效
            self.yaw_total = 0.0
            self._last_fallback_yaw = None
            self.create_subscription(Odometry, "/odom", self._on_odom_only, 10)

    def _on_imu(self, msg):
        self.imu_msgs.append(msg)
        self.stamps.append(time.monotonic())

    def _on_odom_only(self, msg):
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        if self._last_fallback_yaw is not None:
            self.yaw_total += wrap_pi(yaw - self._last_fallback_yaw)
        self._last_fallback_yaw = yaw

    def sample(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def lookup(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (np.array([t.x, t.y, t.z]),
                quat_to_rotmat(q.x, q.y, q.z, q.w))


def publish_zero_once(node):
    """发一帧零速（保持车辆不动，抵抗其它遥控发布者）。"""
    try:
        node.cmd_pub.publish(Twist())
    except Exception:
        pass


def section(title):
    print("\n" + "-" * 66)
    print(title)
    print("-" * 66)


def check_static(sampler, config):
    """静态检测：量程、重力方向交叉验证、零偏、噪声。"""
    problems = []
    msgs = sampler.imu_msgs
    span = sampler.stamps[-1] - sampler.stamps[0] if len(sampler.stamps) > 1 else 0.0
    rate = (len(msgs) - 1) / span if span > 1e-6 else float("nan")
    print(f"  采样 {len(msgs)} 帧，平均 {rate:.1f} Hz")

    # ---- 1. frame_id ----
    section("[1] 话题与坐标系")
    frame_id = msgs[-1].header.frame_id
    print(f"  /imu.header.frame_id = \"{frame_id}\"")
    if frame_id == "imu_link":
        print("  ✓ 与 robot.launch.py 静态 TF 的 child_frame_id 一致。")
    else:
        print("  ✗ 应为 \"imu_link\"（TF base_link→imu_link 的 child_frame_id）。")
        print("    改 originbot_base/src/originbot_base.cpp 的 publishImu() 后重新编译。")
        problems.append("frame_id 不是 imu_link")

    # ---- 2. 加速度量程 ----
    section("[2] 加速度计量程 / 换算系数")
    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y,
                     m.linear_acceleration.z] for m in msgs])
    acc_mean = acc.mean(axis=0)
    acc_norm = float(np.linalg.norm(acc_mean))
    print(f"  静止比力均值 = ({acc_mean[0]:+.3f}, {acc_mean[1]:+.3f}, "
          f"{acc_mean[2]:+.3f}) m/s²")
    print(f"  模长 = {acc_norm:.3f} m/s²   (理论 {G:.2f}, 容差 ±{ACC_NORM_TOL})")
    if abs(acc_norm - G) < ACC_NORM_TOL:
        print("  ✓ 量程/换算正确，且含重力（raw 加速度）。")
    else:
        print(f"  ✗ 模长偏差 {acc_norm - G:+.3f} m/s²。可能原因：")
        print("    - 小车未水平放置，或测试期间被碰动")
        print("    - 固件 ACC_RATIO 换算系数不对（检查 originbot_base.h）")
        problems.append(f"加速度模长 {acc_norm:.2f} 偏离 {G:.2f}")
        print("    ⚠ 以下重力方向判定在模长异常时不可靠，仅供参考。")

    # ---- 3. 重力方向交叉验证（核心）----
    section("[3] 安装朝向交叉验证（静止比力 vs 静态 TF）")
    print("  原理：加速度计测的是『比力』（地面支撑力），小车水平静止时该力沿")
    print("        base_link 的 +Z（向上）。把它按 TF 换算到 IMU 系，应与实测值同向。")
    print("  ⚠ 局限：静止比力只能定出 roll/pitch，且存在【绕 roll 翻转 180° 的")
    print("     固有二义性】——水平装与翻转装的读数恰为 ±g，二者都自洽。")
    print("     只有 [6] 旋转检测（陀螺实测）才能彻底确认朝向。")
    tf_result = sampler.lookup("base_link", "imu_link")
    if tf_result is None:
        print("  ✗ 未找到 TF base_link→imu_link，无法交叉验证。")
        print("    请确认底盘已启动（robot.launch.py 会发布该 TF）。")
        problems.append("缺少 base_link→imu_link TF")
        return problems, acc_mean, acc

    trans, rot = tf_result
    print(f"\n  TF base_link→imu_link 平移 = ({trans[0]:+.3f}, {trans[1]:+.3f}, "
          f"{trans[2]:+.3f}) m")
    print(f"  TF 对应的 RPY = roll={math.degrees(config['roll']):+.1f}°, "
          f"pitch={math.degrees(config['pitch']):+.1f}°, "
          f"yaw={math.degrees(config['yaw']):+.1f}°  (来自 imu.json)")

    expected_in_imu = rot.T @ np.array([0.0, 0.0, G])
    delta_deg = angle_between_deg(acc_mean, expected_in_imu)
    print(f"\n  实测比力(IMU系)   = ({acc_mean[0]:+.3f}, {acc_mean[1]:+.3f}, "
          f"{acc_mean[2]:+.3f})")
    print(f"  按TF推算应有方向  = ({expected_in_imu[0]:+.3f}, "
          f"{expected_in_imu[1]:+.3f}, {expected_in_imu[2]:+.3f})")
    print(f"  两向量夹角        = {delta_deg:.2f}°")

    # 比力与重力在同一 RPYδ 下同向翻转，故二义：d 与 180-d 都"合理"。
    # 用 min(d, 180-d) 作为"与声明朝向的偏差量"，并显式报告二义性。
    deviation = min(delta_deg, 180.0 - delta_deg)
    if deviation < GRAVITY_OK_DEG:
        yaw_ambiguous = delta_deg < 90.0
        print(f"  ✓ 与声明的安装朝向一致（偏差 {deviation:.1f}°，"
              f"在 ±{GRAVITY_OK_DEG}° 容差内）。")
        if yaw_ambiguous:
            print("    注意：由于比力只有单一方向，roll 翻转 180° 的装法读数同样自洽，"
                  "本项无法区分。")
            print("    需要彻底确认请跑：imu_check.py --rotate")
    else:
        print(f"  ✗ 与声明的安装朝向偏差 {deviation:.1f}°，超出 ±{GRAVITY_OK_DEG}°。")
        print("    说明 IMU 实际安装角与 imu.json 里填的不一致。")
        print(f"    当前配置 roll={math.degrees(config['roll']):+.1f}°, "
              f"pitch={math.degrees(config['pitch']):+.1f}°, "
              f"yaw={math.degrees(config['yaw']):+.1f}°")
        print("    排查顺序：")
        print("      1) 先确认小车确实水平静止（倾斜会直接体现为偏差）")
        print("      2) 再核对 roll/pitch 是否与芯片实际贴装方向一致")
        print("      3) yaw 无法由静止数据确定，用 --rotate 实测")
        problems.append(f"IMU 安装朝向偏差 {deviation:.1f}°")

    # 只含比力的方向反推（yaw 不可观测）
    if acc_norm > 1e-3:
        unit = acc_mean / np.linalg.norm(acc_mean)
        roll_meas = math.degrees(math.atan2(unit[1], unit[2]))
        pitch_meas = math.degrees(math.atan2(-unit[0],
                                             math.sqrt(unit[1] ** 2 + unit[2] ** 2)))
        print("\n  仅由静止比力可反推的安装角（yaw 不可观测，且 roll 有二义）：")
        print(f"    roll  ≈ {roll_meas:+.1f}°    "
              f"(当前配置 {math.degrees(config['roll']):+.1f}°)")
        print(f"    pitch ≈ {pitch_meas:+.1f}°    "
              f"(当前配置 {math.degrees(config['pitch']):+.1f}°)")

    # ---- 4. 陀螺零偏与噪声 ----
    section("[4] 陀螺零偏与噪声（静止）")
    gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y,
                      m.angular_velocity.z] for m in msgs])
    gyro_mean = gyro.mean(axis=0)
    gyro_std = gyro.std(axis=0)
    bias_norm = float(np.linalg.norm(gyro_mean))
    noise_norm = float(np.linalg.norm(gyro_std))
    print(f"  零偏均值 = ({gyro_mean[0]:+.5f}, {gyro_mean[1]:+.5f}, "
          f"{gyro_mean[2]:+.5f}) rad/s")
    print(f"  零偏模长 = {bias_norm:.5f} rad/s ({math.degrees(bias_norm):.3f} °/s)"
          f"   上限 {GYRO_BIAS_MAX}")
    print(f"  噪声 σ   = {noise_norm:.5f} rad/s"
          f"   上限 {GYRO_NOISE_MAX}")
    if bias_norm < GYRO_BIAS_MAX:
        print("  ✓ 陀螺零偏正常。")
    else:
        print("  ! 陀螺零偏偏大。若小车确实静止，说明陀螺需要重新标定：")
        print("      ros2 service call /imu_calibrate std_srvs/srv/Trigger {}")
        problems.append(f"陀螺零偏 {bias_norm:.4f} rad/s 偏大")
    if noise_norm < GYRO_NOISE_MAX:
        print("  ✓ 陀螺噪声正常。")
    else:
        print("  ! 陀螺噪声偏大，可能受到振动干扰（检查 IMU 固定是否牢靠）。")
        problems.append(f"陀螺噪声 {noise_norm:.4f} rad/s 偏大")

    # ---- 5. 姿态协方差自查 ----
    section("[5] 姿态输出可用性")
    cov = msgs[-1].orientation_covariance[0]
    print(f"  orientation_covariance[0] = {cov:.3g}")
    if cov >= 1e5:
        print("  ! 驱动声明姿态不可用（协方差 ≥1e5）。")
        print("    robot_localization 的 EKF 若配置 imu0_config 含姿态项，"
              "会拒绝该测量。")
        print("    改用角速度融合（imu0_config 的 yaw/vyaw 用 twist 而非 pose），"
              "或在 EKF 里关闭姿态项。")
    else:
        print("  ✓ 驱动声明姿态可用（消费端仍应按需选择 pose/twist）。")

    return problems, acc_mean, acc


def check_rotate(sampler, args, config):
    """旋转一致性：脚本自行驱动小车原地转整圈，三方比对。

    为什么必须自己驱动：手推小车时轮子只是被拖着走、并不滚动，/odom 几乎为 0，
    "陀螺/里程计"比值毫无意义。脚本驱动时轮子真正滚动，三个量才可比。

    另外注意陀螺积分必须覆盖【旋转全过程】：本函数在调用 rotate_to_angle 之前
    记下样本下标，之后只对新增的样本积分 —— 旧版本是在旋转【结束之后】才开始
    采样，积到的基本是静止数据，所以那个检查实际上一直是失效的。
    """
    section("[6] 旋转一致性（陀螺 vs 里程计 vs 雷达）")
    if not HAVE_COMMON:
        print("  未找到 calib_common.py，无法自动驱动小车旋转，已跳过第 6/7 项。")
        print("  （手推旋转时 /odom 不转，比值无意义，故不做替代方案。）")
        return []

    turns = args.rotate_turns
    target_deg = turns * 360.0
    mount_xy, mount_yaw = _resolve_mount(args)
    print(f"  小车将【自行原地转 {turns:g} 圈】（{target_deg:.0f}°），"
          f"角速度 {args.turn_speed:.2f} rad/s，"
          f"预计 {abs(math.radians(target_deg)) / args.turn_speed:.0f} s")
    print("  请确保四周 0.5m 内无障碍、地面平整。")
    if not args.no_prompt:
        input("  准备好后按回车开始……")

    ref = None
    if sampler.grabber is not None:
        ref, info = cc.capture_reference(sampler, sampler.grabber, seconds=1.5,
                                         zero_pub=lambda: publish_zero_once(sampler))
        if ref is None:
            print(f"  ! 雷达参考帧采集失败：{info['reason']}")
            print("    → 只能比对陀螺与里程计。")

    if not sampler.wait_for_odom():
        print("  ✗ 未收到 /odom，无法闭环驱动旋转，跳过第 6/7 项。")
        return []

    sampler.reset_accumulators()
    start_idx = len(sampler.imu_msgs)
    t0 = time.monotonic()
    result = sampler.rotate_to_angle(target_deg, args.turn_speed)
    elapsed = time.monotonic() - t0
    if not result.ok:
        print(f"  ✗ 驱动未正常完成（{result.reason}，{elapsed:.1f}s），已停车。")
        if result.reason == "stall":
            print("    停滞通常是打滑或轮子悬空：检查地面与电量后重试。")
        return []

    # ---- 陀螺积分（只覆盖旋转期间新增的样本）----
    if len(sampler.imu_msgs) - start_idx < 10:
        print("  ✗ 旋转期间未采到足够的 /imu 数据，跳过。")
        return []
    stamps = np.array(sampler.stamps[start_idx:])
    gyro_z = np.array([m.angular_velocity.z
                       for m in sampler.imu_msgs[start_idx:]])
    # 零阶保持积分，对本场景的 50Hz 采样足够
    gyro_angle = float(np.sum(gyro_z[:-1] * np.diff(stamps)))
    odom_angle = sampler.yaw_total
    print(f"\n  旋转用时        : {elapsed:.1f} s，采到 {len(gyro_z)} 帧 /imu")
    print(f"  陀螺积分转角    : {math.degrees(gyro_angle):+8.2f}°")
    print(f"  里程计转角      : {math.degrees(odom_angle):+8.2f}°")

    problems = []

    # ---- 雷达真值（独立于底盘的两个传感器）----
    lidar_angle = None
    if ref is not None:
        cur, info = cc.capture_points(sampler, sampler.grabber, seconds=1.5,
                                      zero_pub=lambda: publish_zero_once(sampler))
        if cur is None:
            print(f"  ! 雷达当前帧采集失败：{info['reason']}")
        else:
            print("\n  解算雷达真值中……")
            est = cc.estimate_rotation(ref, cur, predicted_dth=odom_angle,
                                       mount_xy=mount_xy, mount_yaw=mount_yaw)
            print(est.describe("  "))
            if est.reliable:
                lidar_angle = est.dth
                print(f"\n  雷达实测转角    : {math.degrees(lidar_angle):+8.2f}°"
                      f"  ← 独立真值")
            else:
                print("  ! 雷达结果不可靠，本次退化到只比对陀螺与里程计。")
    print("  " + "-" * 62)

    if abs(odom_angle) < math.radians(180.0):
        print("  ! 里程计转角 <180°：底盘可能没转够或未发布 /odom，比值不可靠。")
    if abs(gyro_angle) < math.radians(180.0):
        print("  ✗ 陀螺积分转角 <180°：IMU 数据可能有问题（或未真正转动）。")
        problems.append("陀螺积分转角异常小")

    def ratio_line(name, numerator, denominator, lo, hi, what):
        if denominator is None or abs(denominator) < math.radians(180.0):
            return
        if abs(numerator) < math.radians(180.0):
            return
        ratio = abs(numerator) / abs(denominator)
        mark = "✓" if lo <= ratio <= hi else "✗"
        print(f"  {mark} {name} = {ratio:.4f}   (期望 1.0，范围 {lo}~{hi})   {what}")
        if not (lo <= ratio <= hi):
            problems.append(f"{name} {ratio:.3f}")

    if lidar_angle is not None:
        ratio_line("陀螺 / 雷达  ", gyro_angle, lidar_angle, 0.95, 1.05,
                   "← 隔离出 IMU 刻度误差")
        ratio_line("里程计 / 雷达", odom_angle, lidar_angle, 0.98, 1.02,
                   "← 即 correct_factor_vth 的刻度误差")
    ratio_line("陀螺 / 里程计", gyro_angle, odom_angle, 0.85, 1.15,
               "← 两个底盘传感器之间的一致性")

    problems += check_gyro_sign(gyro_angle, odom_angle, config)
    return problems


def check_gyro_sign(gyro_angle, odom_angle, config):
    """第 7 项：陀螺角速度符号是否与配置的安装 RPY 自洽。

    驱动发布的 angular_velocity.z 是芯片自身轴的角速度。车体绕 base_link +Z
    以 ω 逆时针旋转时，芯片系下 ω_imu.z = R[2][2]·ω，而 ZYX 的 RPY 有
    R[2][2] = cos(pitch)·cos(roll)。出厂 roll=3.1416 时这是负的，即"车体逆时针
    → 陀螺 z 为负"。

    ⚠ 该推断假设固件没有预先旋转过陀螺数据。这一条未经硬件验证，所以**只作
    提示、不计入 problems**，避免误报。上机时先手推转一下看符号再决定信不信。
    """
    section("[7] 陀螺符号与安装 RPY 的自洽性")
    if abs(gyro_angle) < math.radians(180.0) or abs(odom_angle) < math.radians(180.0):
        print("  转角样本不足，跳过符号检查。")
        return []

    expected = 1.0 if math.cos(config["pitch"]) * math.cos(config["roll"]) >= 0 \
        else -1.0
    print(f"  配置 roll={math.degrees(config['roll']):+.1f}° "
          f"pitch={math.degrees(config['pitch']):+.1f}° "
          f"→ cos(pitch)·cos(roll) 符号为 {expected:+.0f}")
    print(f"  实测：陀螺 {math.degrees(gyro_angle):+.1f}°，"
          f"里程计 {math.degrees(odom_angle):+.1f}°")
    if gyro_angle * odom_angle * expected > 0:
        print("  ✓ 陀螺符号与配置的安装 RPY 自洽。")
        print("    这也顺带消解了第 [3] 项提到的『绕 roll 翻转 180°』二义性：")
        print("    两种装法的陀螺符号相反，实测符号只会与其中一种相符。")
        return []
    if gyro_angle * odom_angle < 0:
        print("  ! 陀螺与里程计的转向符号相反。若里程计方向已确认正确，")
        print("    则说明 IMU 的安装 RPY（或陀螺轴）与配置不符。")
    print("  ? 陀螺符号与配置的安装 RPY 预期相反。可能原因：")
    print("    a) roll/pitch 填错（例如漏了 180° 翻转）")
    print("    b) 固件已经预先旋转过陀螺数据 —— 若是这样，本条提示属误报")
    print("    先手推小车转一下、看 angular_velocity.z 的符号，再决定是否修改配置。")
    return []


def _resolve_mount(args):
    """雷达安装位姿：命令行覆盖 > lidar.json > 出厂默认。"""
    x, y, yaw = cc.load_lidar_mount() if HAVE_COMMON else (0.15, 0.0, 0.0)
    if args.mount_x is not None:
        x = args.mount_x
    if args.mount_y is not None:
        y = args.mount_y
    if args.mount_yaw is not None:
        yaw = args.mount_yaw
    return (x, y), yaw


def main():
    parser = argparse.ArgumentParser(
        description="OriginBot IMU 安装与数据正确性检测")
    parser.add_argument("--seconds", type=float, default=6.0,
                        help="静态采样时长秒数（默认 6）")
    parser.add_argument("--rotate", action="store_true",
                        help="额外做旋转一致性检测（脚本会自行驱动小车原地转圈）")
    parser.add_argument("--rotate-turns", type=float, default=2.0,
                        help="旋转检测的圈数（默认 2）")
    parser.add_argument("--turn-speed", type=float, default=0.6,
                        help="旋转检测的角速度 rad/s（默认 0.6）")
    parser.add_argument("--no-prompt", action="store_true",
                        help="旋转检测不等待回车，自动开始")
    parser.add_argument("--mount-x", type=float, default=None,
                        help="覆盖雷达安装 X 偏移（米），默认读 lidar.json")
    parser.add_argument("--mount-y", type=float, default=None,
                        help="覆盖雷达安装 Y 偏移（米），默认读 lidar.json")
    parser.add_argument("--mount-yaw", type=float, default=None,
                        help="覆盖雷达安装偏航（弧度），默认读 lidar.json")
    args = parser.parse_args()

    config = load_imu_config()

    print("=" * 66)
    print("OriginBot IMU 检测")
    print("=" * 66)
    print(f"  配置(imu.json): x={config['x']:+.3f} y={config['y']:+.3f} "
          f"z={config['z']:+.3f}")
    print(f"                 roll={math.degrees(config['roll']):+.1f}° "
          f"pitch={math.degrees(config['pitch']):+.1f}° "
          f"yaw={math.degrees(config['yaw']):+.1f}°")
    print(f"\n  请确保小车【水平静止】放置，测试期间不要触碰。"
          f"开始采样 {args.seconds:.0f} 秒……")

    rclpy.init()
    sampler = ImuSampler()
    try:
        sampler.sample(args.seconds)
        if len(sampler.imu_msgs) < 10:
            print("\n✗ 未收到足够的 /imu 数据。请确认底盘已启动：")
            print("    ros2 launch originbot_bringup originbot.launch.py")
            return 1

        problems, _, _ = check_static(sampler, config)
        if args.rotate:
            problems += check_rotate(sampler, args, config)

        print("\n" + "=" * 66)
        if problems:
            print(f"检测完成：发现 {len(problems)} 个问题")
            for item in problems:
                print(f"  · {item}")
            print("\n对应的修正办法见上方各节提示。"
                  "IMU 安装位姿可在 Web 端参数配置页「IMU 安装位姿」修改，"
                  "保存后重启底盘生效。")
            return 2
        print("检测完成：✓ 未发现明显问题，IMU 安装与数据均正常。")
        return 0
    except KeyboardInterrupt:
        print("\n已中断（底盘自身的 600ms 看门狗也会让它停下）。")
        return 130
    finally:
        try:
            sampler.stop_robot()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
