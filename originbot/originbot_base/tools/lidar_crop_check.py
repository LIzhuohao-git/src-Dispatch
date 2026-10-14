#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光雷达角度裁剪检测工具（OriginBot / STL06N）

【用途】
  实测雷达视野里哪些角度被车身/支架/线束遮挡，自动算出应裁剪的区间，
  直接给出可填入 Web 端参数配置页「激光雷达」分组的值。
  同时校验雷达正前方是否被遮挡（用于发现安装方向配反）。

【背景】
  雷达转一圈 360°，其中朝向车尾/车身的扇区会被自己的结构挡到，测出
  0.1~0.3m 的假障碍。驱动把这些角度置 NaN 后建图/避障才干净。
  裁剪比较发生在雷达【自身角度系】（见 ldlidar/src/ros2_node/main.cpp
  的 ToLaserscanMessagePublish），因此本脚本会按配置里的 scan_dir
  做镜像换算，输出的数值可直接粘贴。

【前置条件】
  - 底盘已启动（提供 /scan）：ros2 launch originbot_bringup originbot.launch.py
  - 小车停在一处相对空旷的地方（四周 >1m 无杂物），静止不动
  - 已 source 环境：source /opt/ros/humble/setup.bash && source install/setup.bash

【用法】
  ros2 run originbot_base lidar_crop_check.py            # 常规检测
  ros2 run originbot_base lidar_crop_check.py --seconds 8
  ros2 run originbot_base lidar_crop_check.py --threshold 0.4
  ros2 run originbot_base lidar_crop_check.py --dir-check   # 额外做方向实测
  ros2 run originbot_base lidar_crop_check.py --dir-check --no-prompt

【判读标准】
  遮挡段          距离 < 阈值 且【在多数帧里】持续出现的扇区。
                  偶发的近点（走过的人、桌腿）会被逐帧稳定性过滤掉。
  建议裁剪区间    最大连续遮挡段两端各留 --margin 度安全余量。
  正前方 0°±15°   若被判为遮挡 → 说明车头方向有东西挡着，或
                  scan_dir 配反了（雷达 0° 实际指向车尾）。

【--dir-check 会自行驱动小车转 90°】
  没有 calib_common.py 时退化为"请你手动把小车原地左转 90° 后回车"。
  判定用**实际转过的角度**做归一（不是写死的 90°），所以驱动少转了也能正确判读。

【退出码】0 = 正常；1 = 未收到 /scan；2 = 正前方疑似遮挡（需人工确认）
"""

import argparse
import json
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

try:
    import calib_common as cc
    HAVE_COMMON = True
except ImportError:                     # 单文件被拷走时仍可用（只是要人工转）
    HAVE_COMMON = False

GENERATED_DIR = Path.home() / ".originbot_delivery" / "config" / "generated"

# 与 ldlidar/launch/stl06n.launch.py 内置默认值保持一致。
# 键名 = ldlidar 驱动声明的 ROS 参数名；旧版 Web 端用的是 scan_dir /
# enable_angle_crop，_load 时做别名映射。
LIDAR_DEFAULTS = {
    "laser_scan_dir": True,
    "enable_angle_crop_func": False,
    "angle_crop_min": 95.0,
    "angle_crop_max": 265.0,
    "min_range": 0.1,
    "max_range": 20.0,
}

# 旧键名 → 新键名（读不到新键时回落到旧键）
LEGACY_KEYS = {
    "laser_scan_dir": "scan_dir",
    "enable_angle_crop_func": "enable_angle_crop",
}


# 分箱粒度：1° 一个扇区（STL06N 角分辨率约 0.72°，1° 足够且输出紧凑）
BINS = 360


def load_lidar_config():
    """读取 Web 端参数配置页生成的 lidar.json（缺文件回落出厂默认值）。"""
    config = dict(LIDAR_DEFAULTS)
    try:
        with open(GENERATED_DIR / "lidar.json", "r", encoding="utf-8") as handle:
            stored = json.load(handle)
    except (OSError, ValueError):
        return config
    for key in config:
        if key in stored:
            config[key] = stored[key]
        elif key in LEGACY_KEYS and LEGACY_KEYS[key] in stored:
            config[key] = stored[LEGACY_KEYS[key]]
    return config


def scan_angle_to_bin(angle_rad):
    """ROS scan 的角度（弧度）→ 扇区下标（度）。

    driver 声明 angle_min=0、angle_max=2π、angle_increment>0，
    按 REP-103 逆时针为正，故 0° = 车头正前方。
    """
    deg = math.degrees(angle_rad) % 360.0
    return min(BINS - 1, int(deg))


class ScanCollector(Node):
    """订阅 /scan，把每帧测量按 1° 分箱。"""

    def __init__(self):
        super().__init__("lidar_crop_check")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos)
        self.frames = []          # 每帧的 [(bin, range), ...]
        self.meta = None          # 首帧的 (angle_min, angle_increment, range_min, range_max)
        self.nan_bins = [0] * BINS   # 各扇区在多少帧里是 NaN（+inf 也算）

    def _on_scan(self, msg):
        if self.meta is None:
            self.meta = (msg.angle_min, msg.angle_increment,
                         msg.range_min, msg.range_max)
        frame = {}
        for index, raw_range in enumerate(msg.ranges):
            bin_index = scan_angle_to_bin(msg.angle_min + index * msg.angle_increment)
            # NaN / inf 都按"无测量"处理。驱动把裁剪区与越界读数都置 NaN
            # （ldlidar main.cpp:265-274），所以 isfinite 已经够用。
            # 注意这与 calib_common.prepare_points 的口径不同：那边是给运动估算
            # 用的，额外要求 r >= r_min（防别的驱动写 0.0）；本工具做的是"遮挡
            # 直方图"，0.0 在这里本来就该算成遮挡读数，两者不要混用。
            if not math.isfinite(raw_range):
                self.nan_bins[bin_index] += 1
                frame[bin_index] = None
            else:
                # 同一扇区可能落进多个点，取最近的回波
                if bin_index not in frame or frame[bin_index] is None:
                    frame[bin_index] = raw_range
                else:
                    frame[bin_index] = min(frame[bin_index], raw_range)
        self.frames.append(frame)


def collect(seconds):
    collector = ScanCollector()
    deadline = collector.get_clock().now().nanoseconds + int(seconds * 1e9)
    while rclpy.ok() and collector.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(collector, timeout_sec=0.2)
    return collector


def build_blocked_map(collector, threshold, hit_ratio):
    """逐扇区判定是否"持续遮挡"。

    判据：该扇区在 ≥ hit_ratio 比例的帧里，测距都 < threshold。
    用比例而不是"任一帧"，是为了过滤偶发近点（走动的人、临时伸进来的桌腿）。
    """
    blocked = [False] * BINS
    min_range_of_bin = [float("inf")] * BINS
    for frame in collector.frames:
        for bin_index, value in frame.items():
            if value is not None and value < min_range_of_bin[bin_index]:
                min_range_of_bin[bin_index] = value
    for bin_index in range(BINS):
        hits = 0
        seen = 0
        for frame in collector.frames:
            value = frame.get(bin_index)
            if value is None:
                continue
            seen += 1
            if value < threshold:
                hits += 1
        if seen > 0 and hits >= hit_ratio * seen:
            blocked[bin_index] = True
    return blocked, min_range_of_bin


def find_runs(blocked):
    """找出所有连续遮挡段（含跨界段），返回 [(起始角, 结束角, 跨度), ...]。"""
    runs = []
    if all(blocked):
        return [(0.0, 359.0, 360.0)]
    # 从任意未遮挡扇区之后开始扫，保证跨界段能连起来
    start_at = next(i for i in range(BINS) if not blocked[i]) + 1
    index = 0
    while index < BINS:
        pos = (start_at + index) % BINS
        if blocked[pos]:
            length = 0
            while length < BINS and blocked[(start_at + index + length) % BINS]:
                length += 1
            first = (start_at + index) % BINS
            last = (start_at + index + length - 1) % BINS
            runs.append((float(first), float(last),
                         float(length)))
            index += length
        else:
            index += 1
    runs.sort(key=lambda item: item[2], reverse=True)
    return runs


def suggest_crop(run, margin):
    """把遮挡段两端各留 margin 度余量，得到裁剪区间（雷达自身角度系）。"""
    start_deg, end_deg, span = run
    crop_min = (start_deg - margin) % 360.0
    crop_max = (end_deg + margin) % 360.0
    if span + 2 * margin >= 360.0:
        # 退化情形：整圈都算遮挡。此时裁剪无意义，直接返回全裁剪并让上层告警
        return 0.0, 359.999
    return crop_min, crop_max


def to_config_frame(crop_min, crop_max, scan_dir):
    """把"雷达自身角度系"的裁剪区间换算到 Web 端配置语义。

    驱动在自身角度系里比较 (dir_angle >= min && dir_angle <= max)，
    而 dir_angle 随测量时刻从 0° 增到 360°，与 scan_dir 的正装/倒装无关 ——
    所以实测得到的区间就是驱动要的值。scan_dir=False 时，ROS 输出角度是
    自身角度的镜像（angle_ros = 360° - dir_angle），本脚本的 blocked[]
    建立在 ROS 角度轴上，因此需要镜像回自身角度系。

    返回 (配置起始角, 配置结束角, 覆盖视野跨度)。

    ⚠ 与 direction_check 的一致性约定：blocked[] 建立在【ROS 输出角度轴】上，
      而本函数是唯一把它换算到配置语义的地方。若日后有人"顺手"在调用
      suggest_crop 之前先把 blocked[] 旋转一下（例如为了做方向实测），
      下面的镜像换算就会错一倍。方向实测只用于判读，不参与这里的数值。
    """
    span = (crop_max - crop_min) % 360.0
    if scan_dir:
        return crop_min, crop_max, span
    # 镜像：(a, b) → (360-b, 360-a)，闭区间语义下仍是合法区间
    return ((360.0 - crop_max) % 360.0,
            (360.0 - crop_min) % 360.0,
            span)


def print_histogram(blocked, threshold, scan_dir):
    """打印按 15° 聚合的遮挡概览，便于快速定位。"""
    print("\n遮挡概览（每格代表 15° 视野，共 24 格）:")
    print("         0°        90°       180°       270°")
    for base in range(0, 360, 15):
        bar = "".join("#" if blocked[(base + offset) % BINS] else "."
                      for offset in range(0, 15))
        print(f"  {base:>3}°  {bar}")
    print("\n  图例：'#'=持续遮挡   '.'=正常回波")
    print(f"  判据：该角度在多数帧中距离 < {threshold:.2f} m")
    print(f"  角度轴：ROS 输出角度，0°=车头正前方，逆时针为正；"
          f"配置 scan_dir={scan_dir}")


def check_forward(blocked, half_width=15):
    """检查车头方向附近是否被判为遮挡。"""
    blocked_count = sum(1 for offset in range(-half_width, half_width + 1)
                        if blocked[offset % BINS])
    return blocked_count, 2 * half_width + 1


def direction_check(blocked_before, seconds, threshold, hit_ratio, args):
    """方向实测：原地左转 90°，看遮挡扇区是否跟着一起转。

    用于消除 scan_dir 语义歧义：正装时雷达自身 0° 与车头 0° 同向，
    原地左转 90° 后车身遮挡（原本在车尾 180°）应出现在 90°(左) 一侧。

    有 calib_common 时脚本自己驱动小车转（并等它停稳 —— 转动中的运动模糊会
    污染遮挡图）；没有时退化为人工推动。
    """
    print("\n" + "=" * 66)
    print("方向实测（可选）")
    print("=" * 66)
    before_runs = find_runs(blocked_before)
    if not before_runs:
        print("  转向前没有识别到遮挡段，无法做方向判定。")
        return None

    commanded = 90.0
    driver = None
    if HAVE_COMMON and not args.no_prompt:
        print(f"  小车将【自行原地向左转 {commanded:.0f}°】，然后自动复测。")
        print("  （转向过程中请勿平移，四周 0.5m 内保持无障碍）")
        driver = cc.MotionDriver("lidar_crop_dir")
        try:
            driver.spin_for(1.0)
            if not driver.wait_for_odom():
                print("  ✗ 未收到 /odom，无法自动转向。")
                print("    请手动把小车原地向左转 90°，转好后按回车。")
                try:
                    input()
                except EOFError:
                    return None
            else:
                try:
                    input("  准备好后按回车开始……")
                except EOFError:
                    pass
                driver.reset_accumulators()
                result = driver.rotate_to_angle(commanded, 0.6)
                if not result.ok:
                    print(f"  ✗ 转向未正常完成（{result.reason}），跳过方向判定。")
                    return None
        finally:
            if driver is not None:
                try:
                    driver.stop_robot()
                except Exception:
                    pass
    else:
        print(f"  请把小车【原地向左转 {commanded:.0f}°】（逆时针），转好后按回车。")
        print("  （转向过程中请勿平移）")
        try:
            input()
        except EOFError:
            return None

    # 实际转过的角度：命令方向已知（逆时针为正），大小取里程计读数。
    # 这样即便里程计符号反了，也不会把"同向"误判成"反向"。
    if driver is not None and driver.samples:
        actual_deg = math.copysign(abs(math.degrees(driver.yaw_total)), commanded)
    else:
        actual_deg = commanded          # 人工方式只能假定转够了 90°

    collector = collect(seconds)
    if not collector.frames:
        print("  转向后未收到 /scan，跳过方向判定。")
        return None
    after_runs = find_runs(build_blocked_map(collector, threshold, hit_ratio)[0])
    if not after_runs:
        print("  转向后未能识别遮挡段，跳过方向判定。")
        return None

    print(f"\n  转向前最大遮挡段起始角: {before_runs[0][0]:6.1f}°")
    print(f"  转向后最大遮挡段起始角: {after_runs[0][0]:6.1f}°")
    print(f"  实际转过角度          : {actual_deg:+.1f}°")
    delta = (after_runs[0][0] - before_runs[0][0]) % 360.0
    print(f"  遮挡段移动量          : {delta:+.1f}°")

    if abs(actual_deg) < 45.0:
        print("  ? 实际只转了不到 45°，样本不足，建议重测。")
        return None

    # 用实际转角归一，而不是写死 ±90°：驱动少转一点也仍能正确判读。
    same = abs((delta - actual_deg + 180.0) % 360.0 - 180.0)
    opposite = abs((delta + actual_deg + 180.0) % 360.0 - 180.0)
    if same < 30.0:
        print("  ✓ 遮挡段跟着转了同样的角度，scan 角度轴与车体一致"
              "（正装语义正确）。")
        return True
    if opposite < 30.0:
        print("  ✗ 遮挡段反向移动了同样的角度，角度轴与车体相反，"
              "当前 scan_dir 与实际安装不符。")
        return False
    print("  ? 移动量既不等于 +实际转角 也不等于 −实际转角，"
          "可能转动被卡住或有干扰，建议重测。")
    return None


def main():
    parser = argparse.ArgumentParser(
        description="激光雷达角度裁剪检测（OriginBot / STL06N）")
    parser.add_argument("--seconds", type=float, default=5.0,
                        help="采样时长秒数（默认 5）")
    parser.add_argument("--threshold", type=float, default=0.35,
                        help="判定遮挡的距离阈值，米（默认 0.35）")
    parser.add_argument("--margin", type=float, default=3.0,
                        help="建议区间两端的安全余量，度（默认 3）")
    parser.add_argument("--hit-ratio", type=float, default=0.6,
                        help="判为持续遮挡所需的最小帧比例（默认 0.6）")
    parser.add_argument("--dir-check", action="store_true",
                        help="额外做原地转 90° 的安装方向实测（脚本自行驱动小车）")
    parser.add_argument("--no-prompt", action="store_true",
                        help="方向实测不等待回车，自动开始")
    args = parser.parse_args()

    config = load_lidar_config()
    scan_dir = bool(config["laser_scan_dir"])

    print("=" * 66)
    print("激光雷达角度裁剪检测")
    print("=" * 66)
    print(f"  采样时长      : {args.seconds:.1f} s")
    print(f"  遮挡阈值      : {args.threshold:.2f} m")
    print(f"  配置安装方向  : {scan_dir} "
          f"({'正装' if scan_dir else '倒装'})")
    print(f"  当前裁剪配置  : enable={config['enable_angle_crop_func']}, "
          f"[{config['angle_crop_min']:.0f}°, {config['angle_crop_max']:.0f}°]")
    print("\n请确保小车静止、四周 1m 内尽量空旷……")

    rclpy.init()
    try:
        collector = collect(args.seconds)
        if not collector.frames or collector.meta is None:
            print("\n✗ 未收到 /scan 数据。请确认雷达已启动：")
            print("    ros2 launch originbot_bringup originbot.launch.py")
            return 1

        angle_min, angle_inc, range_min, range_max = collector.meta
        print(f"\n  /scan: {len(collector.frames)} 帧, "
              f"angle_min={angle_min:.3f} rad, "
              f"angle_increment={angle_inc:.5f} rad ({math.degrees(angle_inc):.3f}°), "
              f"range=[{range_min:.2f}, {range_max:.2f}] m")

        blocked, min_range_of_bin = build_blocked_map(
            collector, args.threshold, args.hit_ratio)
        print_histogram(blocked, args.threshold, scan_dir)

        runs = find_runs(blocked)
        print("\n" + "-" * 66)
        print("检测到的遮挡段（按跨度排序）")
        print("-" * 66)
        if not runs:
            print("  未检测到持续遮挡。若车尾确实有结构件，可调大 --threshold")
            print("  （例如车身在 0.25m 内 → --threshold 0.3）后重试。")
        else:
            for start_deg, end_deg, span in runs[:6]:
                # 该段内的最小回波距离（起点/终点扇区中取更近的那个）
                nearest = min(min_range_of_bin[int(start_deg)],
                              min_range_of_bin[int(end_deg)])
                print(f"  [{start_deg:6.1f}° , {end_deg:6.1f}°]  "
                      f"跨度 {span:5.1f}°  最小距离 {nearest:.2f} m")

        exit_code = 0
        if runs:
            crop_min, crop_max = suggest_crop(runs[0], args.margin)
            cfg_min, cfg_max, span = to_config_frame(crop_min, crop_max, scan_dir)
            print("\n" + "-" * 66)
            print("建议值（可直接填入 Web 端参数配置页 →「激光雷达」分组）")
            print("-" * 66)
            print("  启用角度裁剪          : true")
            print(f"  裁剪起始角 angle_crop_min : {cfg_min:.1f}")
            print(f"  裁剪结束角 angle_crop_max : {cfg_max:.1f}")
            if not scan_dir:
                print(f"  （已按 scan_dir=False 做镜像换算；"
                      f"雷达自身角度系为 [{crop_min:.1f}°, {crop_max:.1f}°]）")
            print(f"\n  覆盖视野: {span:.1f}° "
                  f"（占整圈 {span / 3.6:.1f}%）")
            if span > 180.0:
                print("  ⚠ 覆盖视野超过 180°，雷达实际可用的避障范围已不足一半，"
                      "请确认小车四周确实被车身挡住这么大片。")

        blocked_forward, forward_total = check_forward(blocked)
        print("\n" + "-" * 66)
        print("正前方（0°±15°）检查")
        print("-" * 66)
        if blocked_forward == 0:
            print("  ✓ 车头方向视野正常，无遮挡。")
        elif blocked_forward < forward_total * 0.5:
            print(f"  ! 车头方向有 {blocked_forward}/{forward_total} 个角度被判遮挡，"
                  "可能只是附近有杂物，建议挪开小车后重测。")
        else:
            print(f"  ✗ 车头方向 {blocked_forward}/{forward_total} 个角度被判遮挡！")
            print("    两种可能：")
            print("      a) 车头前方确实有东西（墙/家具）→ 挪到空旷处重测")
            print("      b) scan_dir 配反了（雷达 0° 实际指向车尾）→")
            print(f"         把 Web 端「雷达安装方向（正装）」改成 "
                  f"{'倒装（取消勾选）' if scan_dir else '正装（勾选）'}后重启底盘再测")
            exit_code = 2

        if args.dir_check:
            direction_check(blocked, args.seconds, args.threshold,
                            args.hit_ratio, args)

        return exit_code
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
