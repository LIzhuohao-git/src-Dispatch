#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OriginBot 运行时自检脚本（RDK X5 / ROS2 Humble）

针对以下四类"配置错但运行不报错、只表现为建图/定位精度差"的隐性风险，做运行时实测：

  #4  相机 frame_id / 话题 / TF 一致性
  #7  IMU 静态 TF 方向（用静止比力方向与 TF 做交叉验证）
  #13 激光雷达实际量程（对比驱动声明的 range_max=20m）
  #14 激光雷达角度裁剪方向（正前方 0° 是否被裁剪成盲区）

用法：
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run originbot_navigation runtime_selfcheck            # 若已 colcon build
    # 或直接：
    python3 scripts/runtime_selfcheck.py [--collect-sec 6]

期望值来源：
    雷达/IMU 的安装位姿期望值从 Web 端参数配置页生成的
    ~/.originbot_delivery/config/generated/{lidar,imu}.json 读取
    （配置不存在时回落到出厂默认值）。因此本脚本只做"实际发布的 TF 与
    配置是否一致"的一致性检查；改了 Web 端参数后无需同步改本文件。

本脚本只做"是否一致/是否合理"的快速体检。若需要给出【建议修正值】的深度
标定，请用 originbot_base/tools/ 下的专项工具：
    ros2 run originbot_base lidar_crop_check.py   # 雷达该裁剪哪些角度
    ros2 run originbot_base imu_check.py          # IMU 装对没装对
    ros2 run originbot_base odom_calib.py         # 里程计标定 vx/vth

建议运行条件：
    - 机器人保持静止、放在水平地面（#7 的比力交叉验证依赖此前提）
    - 相机 / 雷达 / 底盘已启动（至少底盘 + 雷达；相机可选）
    - 周围有一定开阔空间（#13 量程校验在开阔环境才更有意义）

退出码：0 = 无 FAIL；1 = 存在 FAIL（WARN/SKIP 不改变退出码）。
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from tf2_ros import Buffer, TransformListener

G = 9.81

# 相机/雷达这类传感器驱动通常用 best_effort；底盘 originbot_base 的 /imu 用默认 reliable。
SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
RELIABLE_QOS = QoSProfile(depth=10)

GENERATED_DIR = Path.home() / ".originbot_delivery" / "config" / "generated"

# 与各 launch 文件内置默认值保持一致（配置文件不存在时的回落值）
LIDAR_DEFAULTS = {
    "mount_x": 0.15, "mount_y": 0.0, "mount_z": 0.08,
    "mount_roll": 0.0, "mount_pitch": 0.0, "mount_yaw": 0.0,
}
IMU_DEFAULTS = {
    "x": -0.05, "y": 0.10, "z": 0.05,
    "roll": 3.1416, "pitch": 0.0, "yaw": 1.5708,
}


def _load_launch_json(filename, defaults):
    """读取 Web 端参数配置页生成的 launch JSON（缺键/缺文件回落 defaults）。

    这些位姿由 Web 端参数配置页统一管理，因此自检的"期望值"必须与生成文件
    同源；若在此写死常量，用户改完参数后自检会误报"TF 不一致"。
    """
    values = dict(defaults)
    try:
        with open(GENERATED_DIR / filename, "r", encoding="utf-8") as handle:
            stored = json.load(handle)
    except (OSError, ValueError):
        return values
    for key in values:
        if key in stored:
            try:
                values[key] = float(stored[key])
            except (TypeError, ValueError):
                pass
    return values


def _expected_tf():
    """launch 文件声明的期望 TF（雷达/IMU 位姿从生成配置读取）。"""
    lidar = _load_launch_json("lidar.json", LIDAR_DEFAULTS)
    imu = _load_launch_json("imu.json", IMU_DEFAULTS)
    return {
        ("base_footprint", "base_link"): ((0.0, 0.0, 0.13325), (0.0, 0.0, 0.0)),
        ("base_link", "imu_link"): (
            (imu["x"], imu["y"], imu["z"]),
            (imu["roll"], imu["pitch"], imu["yaw"]),
        ),
        ("base_link", "laser_link"): (
            (lidar["mount_x"], lidar["mount_y"], lidar["mount_z"]),
            (lidar["mount_roll"], lidar["mount_pitch"], lidar["mount_yaw"]),
        ),
    }


EXPECTED_TF = _expected_tf()
# aruco.launch.py 里 base_footprint -> camera_optical 的默认值
CAMERA_EXPECTED = ((0.12, 0.0, 0.25), (0.0, 0.0, 0.0))

TOL_TRANS = 0.003      # 平移容差（米）
TOL_ROT = 0.02         # 角度容差（弧度，约 1.15°）


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_rotmat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def fmt_rpy(roll, pitch, yaw):
    return (
        f"roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°, "
        f"yaw={math.degrees(yaw):.1f}°"
    )


class SelfCheckNode(Node):
    def __init__(self, collect_sec):
        super().__init__("runtime_selfcheck")
        self.collect_sec = collect_sec
        self.exit_code = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_info = None
        self.image_frame = None
        self.imu_msgs = []
        self.scans = []

        self.create_subscription(Imu, "/imu", self._imu_cb, RELIABLE_QOS)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, SENSOR_QOS)
        self.create_subscription(CameraInfo, "/camera_info", self._cinfo_cb, SENSOR_QOS)
        self.create_subscription(Image, "/image", self._image_cb, SENSOR_QOS)

    # ---------------- 回调 ----------------
    def _imu_cb(self, msg):
        if len(self.imu_msgs) < 5000:
            self.imu_msgs.append(msg)

    def _scan_cb(self, msg):
        if len(self.scans) < 5000:
            self.scans.append(msg)

    def _cinfo_cb(self, msg):
        self.camera_info = msg

    def _image_cb(self, msg):
        self.image_frame = msg.header.frame_id

    # ---------------- 工具 ----------------
    def _topic_exists(self, name):
        names = {n.lstrip("/") for n, _ in self.get_topic_names_and_types()}
        return name.lstrip("/") in names

    def _lookup(self, target, source, timeout=2.0):
        try:
            return self.tf_buffer.lookup_transform(
                target, source,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout),
            )
        except Exception:
            return None

    # ---------------- 报告工具 ----------------
    def _ok(self, text):
        print("  [PASS] " + text)

    def _warn(self, text):
        print("  [WARN] " + text)

    def _fail(self, text):
        print("  [FAIL] " + text)
        self.exit_code = 1

    def _skip(self, text):
        print("  [SKIP] " + text)

    # ---------------- 各检查项 ----------------
    def check_camera(self):
        print("\n[#4] 相机 frame_id / 话题 / TF 一致性")
        print("-" * 60)

        img_ok = self._topic_exists("/image")
        info_ok = self._topic_exists("/camera_info")
        print(f"  话题 /image       : {'存在' if img_ok else '不存在'}")
        print(f"  话题 /camera_info : {'存在' if info_ok else '不存在'}")

        if not img_ok and not info_ok:
            self._skip("相机未启动（未检测到 /image、/camera_info）。"
                       "启动相机后重跑以完整校验 #4。")
            return

        actual_frame = None
        if self.camera_info is not None:
            actual_frame = self.camera_info.header.frame_id
            print(f"  /camera_info.header.frame_id : \"{actual_frame}\"")
        else:
            self._warn("存在 /camera_info 话题但未收到数据（QoS 不匹配或驱动未发布）。")

        if self.image_frame is not None:
            print(f"  /image.header.frame_id       : \"{self.image_frame}\"")

        # 配置里 aruco_localization 默认用 camera_optical
        declared = "camera_optical"
        print(f"  aruco_localization 期望 camera_frame = \"{declared}\"")

        if actual_frame is not None and actual_frame != declared:
            self._fail(f"/camera_info 实际 frame_id \"{actual_frame}\" 与 "
                       f"aruco 配置 \"{declared}\" 不一致，视觉定位会静默失效。")
        elif actual_frame is not None:
            self._ok("frame_id 与 aruco 配置一致。")

        if self.image_frame is not None and actual_frame is not None and \
                self.image_frame != actual_frame:
            self._fail(f"/image.frame_id \"{self.image_frame}\" 与 "
                       f"/camera_info.frame_id \"{actual_frame}\" 不一致。")

        # TF 检查：base_footprint -> 实际 camera frame
        tf_frame = actual_frame or declared
        tf = self._lookup("base_footprint", tf_frame)
        if tf is not None:
            t = tf.transform.translation
            q = tf.transform.rotation
            r, p, y = quat_to_rpy(q)
            print(f"  TF base_footprint -> {tf_frame} : "
                  f"t=({t.x:.3f},{t.y:.3f},{t.z:.3f}), {fmt_rpy(r, p, y)}")
            exp_t, exp_r = CAMERA_EXPECTED
            ok_t = all(abs(a - b) < TOL_TRANS for a, b in zip((t.x, t.y, t.z), exp_t))
            ok_r = all(abs(wrap_pi(a - b)) < TOL_ROT for a, b in zip((r, p, y), exp_r))
            if ok_t and ok_r:
                self._ok("相机 TF 与 aruco.launch.py 默认值一致。")
            else:
                self._warn("相机 TF 与 aruco.launch.py 默认值不一致，"
                           "请确认实际安装位置（camera_x/y/z/yaw）。")
        else:
            self._fail(f"未找到 TF base_footprint -> {tf_frame}。"
                       "相机 TF 仅在 originbot_delivery/aruco.launch.py 启动时发布；"
                       "若未启动视觉定位，忽略此项。")

    def check_imu_tf(self):
        print("\n[#7] IMU 静态 TF 方向（含比力交叉验证）")
        print("-" * 60)

        tf = self._lookup("base_link", "imu_link")
        if tf is None:
            self._fail("未找到 TF base_link -> imu_link。底盘(robot.launch.py)是否已启动？")
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        r, p, y = quat_to_rpy(q)
        print(f"  TF base_link -> imu_link : t=({t.x:.3f},{t.y:.3f},{t.z:.3f}), "
              f"{fmt_rpy(r, p, y)}")

        exp_t, exp_r = EXPECTED_TF[("base_link", "imu_link")]
        ok_t = all(abs(a - b) < TOL_TRANS for a, b in zip((t.x, t.y, t.z), exp_t))
        ok_r = all(abs(wrap_pi(a - b)) < TOL_ROT for a, b in zip((r, p, y), exp_r))
        if ok_t and ok_r:
            self._ok("TF 与 robot.launch.py 声明一致 (roll=180°, yaw=90° 的翻转安装)。")
        else:
            self._warn("TF 与 robot.launch.py 声明不一致，请核对 IMU 实际安装。")

        # 其它静态 TF 快速核对
        for (src, dst), (exp_t, exp_r) in EXPECTED_TF.items():
            if (src, dst) == ("base_link", "imu_link"):
                continue
            tt = self._lookup(src, dst)
            if tt is None:
                self._warn(f"未找到 TF {src} -> {dst}。")
                continue
            tr = tt.transform.translation
            rq = tt.transform.rotation
            rr, pp, yy = quat_to_rpy(rq)
            ok = all(abs(a - b) < TOL_TRANS for a, b in zip((tr.x, tr.y, tr.z), exp_t)) and \
                all(abs(wrap_pi(a - b)) < TOL_ROT for a, b in zip((rr, pp, yy), exp_r))
            tag = "一致" if ok else "不一致"
            print(f"  TF {src} -> {dst} : t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}), "
                  f"{fmt_rpy(rr, pp, yy)} [{tag}]")
            if not ok:
                self._warn(f"{src} -> {dst} 与 launch 声明不一致。")

        # IMU 数据本身
        if not self.imu_msgs:
            self._warn("未收到 /imu 数据（底盘未启动或 QoS 不匹配），跳过比力交叉验证。")
            return

        m0 = self.imu_msgs[-1]
        print(f"  /imu.header.frame_id = \"{m0.header.frame_id}\"")
        if m0.header.frame_id != "imu_link":
            self._fail(f"/imu.frame_id 应为 \"imu_link\"，实际 \"{m0.header.frame_id}\"。")

        acc = np.mean([[m.linear_acceleration.x,
                        m.linear_acceleration.y,
                        m.linear_acceleration.z] for m in self.imu_msgs], axis=0)
        gyro = np.mean([[m.angular_velocity.x,
                         m.angular_velocity.y,
                         m.angular_velocity.z] for m in self.imu_msgs], axis=0)
        acc_norm = float(np.linalg.norm(acc))
        gyro_norm = float(np.linalg.norm(gyro))

        print(f"  静止采样(均值): 角速度模长={gyro_norm:.4f} rad/s, "
              f"加速度模长={acc_norm:.3f} m/s²")
        print(f"  加速度向量(imu 系) = ({acc[0]:.2f}, {acc[1]:.2f}, {acc[2]:.2f})")

        if abs(acc_norm - G) < 0.5:
            self._ok(f"加速度模长≈{G:.2f}，说明 /imu 含重力（raw），"
                     f"EKF 配置 imu0_remove_gravitational_acceleration:=true 正确。")
        else:
            self._warn(f"加速度模长 {acc_norm:.2f} 与 {G:.2f} 偏差较大，"
                       "请确认机器人水平静止，或 IMU 量程/标定是否正确。")

        # 比力交叉验证：水平静止时，imu 系测得的【比力】（地面支撑力）应等于
        # R^T @ (0,0,+g)。注意加速度计测的是比力而非重力，故这里用 +g 而非 -g；
        # 用 -g 会让水平安装与翻转安装的判定整体颠倒。
        if gyro_norm > 0.2:
            self._warn(f"角速度模长 {gyro_norm:.3f} 较大，机器人可能在运动，"
                       "比力交叉验证结果不可靠，请静止后重跑。")
        R = quat_to_rotmat(q)  # imu -> base_link 的旋转
        expected_imu = R.T @ np.array([0.0, 0.0, G])
        if acc_norm > 1e-3 and np.linalg.norm(expected_imu) > 1e-3:
            cos = float(np.dot(acc, expected_imu) /
                        (np.linalg.norm(acc) * np.linalg.norm(expected_imu)))
            print(f"  比力方向一致性 cos(实测, 按TF推算) = {cos:+.3f}")
            # 单方向的静止比力无法区分 roll 翻转 180°（水平装读 +g、翻转装读 -g，
            # 两种装法各自的读数都自洽），故取 |cos| 作为"与声明相符"的判据。
            if abs(cos) > 0.95:
                self._ok("IMU 静态 TF 方向与实测比力方向相符。")
                print("    注：静止比力存在绕 roll 翻转 180° 的二义性，"
                      "本项无法区分；要彻底确认请用 --rotate 或"
                      "ros2 run originbot_base imu_check.py --rotate")
            else:
                self._warn("IMU 静态 TF 方向与实测比力方向不一致，"
                           "请核对 IMU 安装朝向与 robot.launch.py 的 roll/yaw。"
                           "（用 imu_check.py 可给出偏差量与建议值）")

    def check_lidar_range(self):
        print("\n[#13] 激光雷达实际量程")
        print("-" * 60)

        if not self.scans:
            self._warn("未收到 /scan 数据（雷达未启动或 QoS 不匹配）。")
            return

        declared = self.scans[0].range_max
        rmin = self.scans[0].range_min
        print(f"  驱动声明 /scan.range_max = {declared:.2f} m (range_min={rmin:.2f})")
        print(f"  参考：Cartographer lds_2d.lua max_range=20，Nav2 laser_max_range=20")

        max_finite = 0.0
        valid_count = 0
        far_bins = {12: 0, 15: 0, 18: 0}
        for s in self.scans:
            for r in s.ranges:
                if math.isfinite(r) and rmin < r < declared:
                    valid_count += 1
                    max_finite = max(max_finite, r)
                    for th in far_bins:
                        if r > th:
                            far_bins[th] += 1

        print(f"  实测最大有效距离 = {max_finite:.2f} m  （共 {valid_count} 个有效点）")
        print(f"  距离分布: >12m 有 {far_bins[12]} 点, >15m 有 {far_bins[15]} 点, "
              f">18m 有 {far_bins[18]} 点")

        if max_finite < 1e-3:
            self._fail("未采集到任何有效测距点，请检查雷达驱动。")
            return

        # 结论：室内 max_finite 受墙限制，不具结论性；给出解释性提示
        if far_bins[12] > 0:
            self._ok("存在 >12m 的有效回波，20m 量程配置基本可信。")
        else:
            self._warn("未观测到 >12m 的有效回波。室内受墙限制属正常；"
                       "若在开阔环境实测仍无 >12m 回波，说明 STL06N 实际量程达不到 20m，"
                       "建议把 Cartographer/Nav2 的 max_range 下调到 ~12m，减少远场噪声。")

    def check_lidar_mask(self):
        print("\n[#14] 激光雷达角度裁剪方向（正前方是否致盲）")
        print("-" * 60)

        if not self.scans:
            self._warn("未收到 /scan 数据。")
            return

        s0 = self.scans[0]
        angle_min = s0.angle_min
        angle_inc = s0.angle_increment
        n = len(s0.ranges)
        print(f"  /scan: angle_min={math.degrees(angle_min):.1f}°, "
              f"angle_increment={math.degrees(angle_inc):.3f}°, 点数={n}")

        bins = 360
        masked = [0] * bins
        valid = [0] * bins
        for s in self.scans:
            if len(s.ranges) != n:
                continue
            for i, r in enumerate(s.ranges):
                ang = math.degrees(angle_min + i * angle_inc)
                b = int(round(ang)) % bins
                # 裁剪/无回波：驱动把该点置 0（或 inf/nan）
                if (not math.isfinite(r)) or r <= 0.0:
                    masked[b] += 1
                else:
                    valid[b] += 1

        frac = [
            (masked[b] / (masked[b] + valid[b]) if (masked[b] + valid[b]) else 1.0)
            for b in range(bins)
        ]

        # 四个方向（±5° 平均）的无效占比
        def sector_frac(deg, half=5):
            acc_m = acc_v = 0
            for d in range(deg - half, deg + half + 1):
                acc_m += masked[d % bins]
                acc_v += valid[d % bins]
            return acc_m / (acc_m + acc_v) if (acc_m + acc_v) else 1.0

        dirs = [(0, "正前方(0°)"), (90, "左(90°)"),
                (180, "正后方(180°)"), (270, "右(270°)")]
        for d, label in dirs:
            print(f"  {label}: 无效占比 {sector_frac(d)*100:.0f}%")

        # 连续被裁剪的扇区
        sectors = self._masked_sectors(frac)
        if sectors:
            desc = ", ".join(self._fmt_sector(s, e) for s, e in sectors)
            print(f"  连续被裁剪扇区: {desc}")
        else:
            print("  连续被裁剪扇区: 无（未检测到连续无效区）")

        # 关键判断：正前方 0° 是否被裁剪
        fwd = sector_frac(0)
        if fwd > 0.9:
            self._fail("正前方 0° 被裁剪/致盲！导航将看不到前方障碍。"
                       "请检查 stl06n.launch.py 的 enable_angle_crop_func / "
                       "angle_crop_min/max 或激光雷达安装朝向。")
        elif fwd > 0.3:
            self._warn("正前方部分被裁剪，可能影响前方避障。")
        else:
            self._ok("正前方 0° 未被裁剪，前方视野正常。")

        # 对照配置 [95°, 265°]（期望裁剪后向）
        rear = sector_frac(180)
        if rear > 0.9 and fwd <= 0.1:
            self._ok("裁剪扇区落在后向，符合 stl06n.launch.py 的 [95°,265°] 预期（仅裁后方）。")
        elif rear <= 0.1 and fwd <= 0.1:
            self._warn("未见明显裁剪扇区，可能 enable_angle_crop_func 未生效。")

    def _masked_sectors(self, frac, thr=0.9):
        runs = []
        i = 0
        while i < 360:
            if frac[i] >= thr:
                j = i
                while j < 360 and frac[j] >= thr:
                    j += 1
                runs.append([i, j - 1])
                i = j
            else:
                i += 1
        if len(runs) >= 2 and runs[0][0] == 0 and runs[-1][1] == 359:
            merged = [runs[-1][0], runs[0][1]]
            runs = runs[1:-1] + [merged]
        return runs

    def _fmt_sector(self, s, e):
        if s <= e:
            return f"{s}°~{e}°"
        return f"{s}°~360°/0°~{e}°(跨0°)"

    def run(self):
        print("=" * 60)
        print(" OriginBot 运行时自检 (RDK X5 / ROS2 Humble)")
        print(f" 采样时长: {self.collect_sec}s   时间: "
              f"{self.get_clock().now().to_msg().sec}")
        print("=" * 60)

        self.check_camera()
        self.check_imu_tf()
        self.check_lidar_range()
        self.check_lidar_mask()

        print("\n" + "=" * 60)
        if self.exit_code:
            print(" 结果: 存在 FAIL，请按上述 [FAIL] 项排查。")
        else:
            print(" 结果: 无 FAIL（WARN/SKIP 项请结合现场情况判断）。")
        print("=" * 60)


def main(args=None):
    parser = argparse.ArgumentParser(description="OriginBot 运行时自检")
    parser.add_argument("--collect-sec", type=float, default=6.0,
                        help="采样时长（秒），默认 6")
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = SelfCheckNode(parsed.collect_sec)

    import time as _time
    end = _time.monotonic() + node.collect_sec
    while _time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.run()
    code = node.exit_code
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
