#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""里程计标定工具（OriginBot 差速底盘）—— 自动行驶 + 雷达实测真值

【用途】
  实测底盘里程计准不准，算出 correct_factor_vx（直线）与
  correct_factor_vth（旋转）的建议值，直接可填进 Web 端参数配置页。

【本工具只打印，不写任何配置文件】
  配置真源是 Web 端参数配置页生成的 ~/.originbot_delivery/config/generated/*.json，
  由 launch 启动时读取。本工具只读它、只打印建议值与 ros2 param set 命令。

【怎么做到的"自主检测"】
  脚本自己开动小车：直行到里程计读数达到 --distance（默认 1.0m）、或原地转到
  --turns 圈；然后**用激光雷达测出真实位移/转角**，与里程计读数相比，得到修正系数：

      新系数 = 旧系数 × (雷达实测 / 里程计读数)

  为什么需要雷达：脚本能驱动小车，但没有独立参考就不知道自己实际走了多远。
  雷达运动前后两次扫描做匹配即可给出真值（原理见 calib_common.py 头部）。
  雷达不可用、或解算置信度不足时，自动回落到人工量尺/读数输入。

【前置条件】
  - 底盘已启动：ros2 launch originbot_bringup originbot.launch.py
  - 小车放在【水平地面】，直线测试前方 ≥1.5m 无障碍、地面平整
  - 已 source 环境：source /opt/ros/humble/setup.bash && source install/setup.bash
  - 未启用 EKF 时 /odom 由底盘发布；启用 EKF 时这里仍订阅 /odom
    （底盘始终发布 /odom，EKF 另发 /odometry/filtered），故两种模式下都适用

【用法】
  ros2 run originbot_base odom_calib.py              # 先自检，再做直线 + 旋转
  ros2 run originbot_base odom_calib.py --check      # 只做静止自检（不动车）
  ros2 run originbot_base odom_calib.py --linear     # 只做直线
  ros2 run originbot_base odom_calib.py --rotate     # 只做旋转
  ros2 run originbot_base odom_calib.py --distance 2 --manual
  ros2 run originbot_base odom_calib.py --ground-truth manual   # 强制人工量尺
  ros2 run originbot_base odom_calib.py --no-prompt  # 全自动（雷达可靠时不需回车）

【实测精度（合成场景蒙特卡洛，见 calib_common.py --selftest 的 C9/C12）】
  直线：系统性偏差约 ±2mm，随机离散(1σ)约 1.8mm —— 1m 行程上合计约 0.4%
  旋转：系统性偏差约 ±0.08°，随机离散(1σ)约 0.03°
  ⚠ 本工具打印的 1σ 是**保守估计**（由代价面曲率推出后再乘 3 倍安全系数，
    因为相邻 beam 的残差是相关的，独立信息量远少于点数）。它用来判断"这次
    结果可不可信"，不是精确的置信区间；上面的偏差也不含停车俯仰、雷达外参、
    base_link 是否真在旋转中心等系统性因素。务必用钢尺交叉核对一次。

【判读标准】
  建议值与旧值接近（差 <2%）→ 里程计本来就准，无需改
  建议值偏离明显         → 按提示写入 Web 端「底盘速度标定」分组后重启底盘
  雷达与钢尺差 >2%       → 估算器在此环境下不可信，请用 --ground-truth manual
  解算判为"不可靠"       → 环境太对称/太空旷或有移动物体，工具会说明原因并
                          自动回落到人工输入

【退出码】0 = 正常；1 = 未收到 /odom 或自检失败
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    import calib_common as cc
    HAVE_COMMON = True
except ImportError:                     # 单文件被拷走时仍可用（只有人工模式）
    HAVE_COMMON = False

GENERATED_DIR = Path.home() / ".originbot_delivery" / "config" / "generated"

# 与 robot.launch.py CHASSIS_DEFAULTS / robot_params.py chassis 组保持一致
CHASSIS_DEFAULTS = {
    "correct_factor_vx": 1.00715,
    "correct_factor_vth": 1.016434,
}

NODE_NAME = "/originbot_base"


def load_chassis_config():
    """读取 Web 端参数配置页生成的 chassis.json（缺文件回落出厂默认值）。"""
    config = dict(CHASSIS_DEFAULTS)
    try:
        with open(GENERATED_DIR / "chassis.json", "r", encoding="utf-8") as handle:
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


def quat_to_yaw(q):
    """四元数 → 偏航角（弧度）。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(angle):
    """把角度折算到 (-π, π]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def ask_float(prompt, default=None):
    """读取一个浮点数；直接回车则返回 default。"""
    while True:
        suffix = f" [回车默认 {default}]" if default is not None else ""
        try:
            text = input(f"{prompt}{suffix}: ").strip()
        except EOFError:
            return default
        if not text and default is not None:
            return default
        try:
            return float(text)
        except ValueError:
            print("  请输入一个数字，例如 1.002")


def read_factor(args_value, key, config):
    """确定"当前系数"。返回 (值, 来源说明)。

    优先级：命令行显式指定 > 运行中节点的活参数 > chassis.json > 出厂默认。
    读活参数很重要：chassis.json 只是 launch 启动时的输入，若有人用
    `ros2 param set` 临时改过，或底盘压根没用这份 json，两者的值会不一致，
    而"旧值 → 新值"的换算必须基于**实际生效**的那个。
    """
    if args_value is not None:
        return args_value, "命令行指定"
    if HAVE_COMMON:
        value, source = cc.get_live_factor(NODE_NAME, key, config[key])
        if source is not None:
            return value, f"来自运行中的 {NODE_NAME} 参数"
    return config[key], "来自 chassis.json（或出厂默认）"


def resolve_mount(args):
    """安装位姿：命令行覆盖 > lidar.json > 出厂默认。"""
    x, y, yaw = cc.load_lidar_mount() if HAVE_COMMON else (0.15, 0.0, 0.0)
    if args.mount_x is not None:
        x = args.mount_x
    if args.mount_y is not None:
        y = args.mount_y
    if args.mount_yaw is not None:
        yaw = args.mount_yaw
    return (x, y), yaw


# ---------------------------------------------------------------------------
# 无 calib_common 时的兜底（只支持时间开环驱动 + 人工测量）
# ---------------------------------------------------------------------------
class FallbackDriver(Node):
    """calib_common.py 缺失时的最小可用驱动。

    只有 odom 累计 + 按时长开环驱动 —— 无法闭环到里程计读数，也没有雷达真值，
    所以这种模式下只能用 --ground-truth manual 配合人工量尺。
    """

    def __init__(self):
        super().__init__("odom_calib")
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.distance = 0.0
        self.yaw_total = 0.0
        self.last_x = self.last_y = self.last_yaw = None
        self.speed = self.yaw_rate = 0.0
        self.samples = 0

    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        if self.last_x is not None:
            self.distance += math.hypot(x - self.last_x, y - self.last_y)
            self.yaw_total += wrap_pi(yaw - self.last_yaw)
        self.last_x, self.last_y, self.last_yaw = x, y, yaw
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z
        self.samples += 1

    def reset_accumulators(self):
        self.distance = 0.0
        self.yaw_total = 0.0

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_odom(self, timeout=8.0):
        """等到收到第一帧 /odom（与 MotionDriver.wait_for_odom 同义）。"""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.samples == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.samples > 0

    def stop_robot(self):
        stop = Twist()
        for _ in range(20):
            try:
                self.cmd_pub.publish(stop)
            except Exception:
                return
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, linear_x, angular_z, seconds):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        deadline = time.monotonic() + seconds
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                self.cmd_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)
        except BaseException:
            self.stop_robot()
            raise
        self.stop_robot()


# ---------------------------------------------------------------------------
# 真值选择
# ---------------------------------------------------------------------------
def resolve_actual(args, est, lidar_value, *, prompt, fallback_default):
    """决定本项的"实测真值"。

    雷达可靠 → 直接用；雷达不可靠 → 打印原因，并把雷达值作为人工输入的默认值
    （按回车即采纳，比量钢尺省事）；--no-prompt 或 --ground-truth lidar 下不可靠
    → 跳过本项，不静默换成人工值。
    """
    if est is not None:
        print("\n  —— 雷达实测 ——")
        print(est.describe("  "))
        if est.reliable and lidar_value is not None:
            print("  → 采用雷达实测值作为真值。")
            return lidar_value, "雷达自动测量"

    if args.ground_truth == "manual":
        if args.no_prompt:
            print("  ! 指定了 --ground-truth manual，但 --no-prompt 下无法请你输入"
                  "实测值 → 跳过本项标定。")
            return None, None
        return ask_float(prompt, default=fallback_default), "人工输入"

    if args.ground_truth == "lidar":
        print("  ! 指定了 --ground-truth lidar，但雷达结果不可靠 → 跳过本项标定。")
        print("    想人工确认：改用 --ground-truth auto，工具会把雷达值作为默认值。")
        return None, None

    # auto 模式：回落到人工，且把雷达值当作默认值
    if est is None:
        print("\n  ! 未能取得雷达真值（见上方原因），回落到人工测量。")
    if args.no_prompt:
        print("  ! 雷达结果不可靠，且指定了 --no-prompt → 跳过本项标定。"
              "（去掉 --no-prompt 可人工确认后继续）")
        return None, None
    return (ask_float(prompt,
                      default=lidar_value if lidar_value is not None
                      else fallback_default),
            "人工输入")


def report_factors(old_factor, actual, odom_value, *, unit, scale_note, sigma=None):
    """由 (实测, 里程计) 算出新系数并打印。返回 (新系数, 是否可信)。

    sigma 是与 actual 同单位的 1σ 不确定度（直线给米、旋转给度），由调用方挑
    对应的那一项 —— 旋转解只填 sigma_dth，它的 sigma_d 恒为 inf，不能混用。

    不可信时不列入汇总的 ros2 param set 建议，避免用户照抄一个错值。
    """
    if odom_value <= 0:
        print("  ✗ 里程计读数非正，无法标定。")
        return None, False

    ratio = actual / odom_value
    error_pct = (odom_value - actual) / actual * 100.0
    new_factor = old_factor * ratio
    print(f"\n  实测 / 里程计 = {actual:.4f} / {odom_value:.4f} = {ratio:.6f}"
          f"  {unit}")
    print(f"  里程计偏差   : {error_pct:+.2f}% "
          f"({'偏大' if error_pct > 0 else '偏小'}，{scale_note})")

    reliable = True
    # 只对"雷达给出的真值"做不确定度门槛：人工钢尺没有这个量
    if sigma is not None and actual > 0 and math.isfinite(sigma):
        rel = sigma / abs(actual)
        if rel > 0.01:
            reliable = False
            print(f"  ⚠ 雷达自身不确定度 {sigma:.4f} {unit} = 实测值的 "
                  f"{rel * 100:.2f}%，超过 1% 门槛，本次建议值不可信。")
            print("     建议：换更开阔/有特征的场地，或用 --distance 2 拉长行程"
                  "（误差按比例摊薄），或改用 --ground-truth manual 人工量尺。")
    return new_factor, reliable


def do_static_check(driver, grabber, args, mount, mount_yaw):
    """静止自检：不动车，验证雷达链路与估算器。"""
    print("\n" + "=" * 66)
    print("静止自检（不动车，用已知答案 0 验证整条雷达链路）")
    print("=" * 66)
    ok, message = cc.static_selftest(driver, grabber, seconds=1.5, settle=1.5,
                                     mount_xy=mount, mount_yaw=mount_yaw,
                                     r_max=args.r_max,
                                     zero_pub=lambda: publish_zero_once(driver))
    print(("  ✓ " if ok else "  ✗ ") + message)
    return ok


def publish_zero_once(driver):
    """发一帧零速（自检期间保持车辆不动，抵抗其它遥控发布者）。"""
    try:
        driver.cmd_pub.publish(Twist())
    except Exception:
        pass


def test_linear(driver, grabber, old_factor, args, mount, mount_yaw):
    """直线测量：标定 correct_factor_vx。"""
    print("\n" + "=" * 66)
    print("① 直线测试 —— 标定 correct_factor_vx")
    print("=" * 66)
    print(f"  指令速度 {args.linear_speed:.3f} m/s，目标里程 {args.distance:.2f} m"
          f"（理论用时约 {abs(args.distance) / args.linear_speed:.0f} s）")
    print("  请确保小车前方 1.5m 内无障碍、地面平整。")
    if HAVE_COMMON:
        print("\n  流程：采集雷达参考帧 → 自动直行到目标里程 → 采集当前帧 → 解算真值")
    else:
        print("\n  ⚠ 未找到 calib_common.py，只能人工量尺（无法自动行驶/测真值）。")
    if not args.no_prompt:
        input("  准备好后按回车开始……")

    ref = None
    use_lidar = HAVE_COMMON and args.ground_truth != "manual"
    if use_lidar:
        ref, info = cc.capture_reference(driver, grabber, seconds=1.5,
                                         r_max=args.r_max,
                                         zero_pub=lambda: publish_zero_once(driver))
        if ref is None:
            print(f"  ! 雷达参考帧采集失败：{info['reason']}")
            print("    → 本次改为人工量尺。")
            use_lidar = False

    driver.reset_accumulators()
    if HAVE_COMMON:
        result = driver.drive_distance(args.distance, args.linear_speed)
        if not result.ok:
            print(f"  ✗ 驱动未正常完成（{result.reason}），已停车。")
            print(f"    走到 {result.reached:.3f} m / 目标 {args.distance:.3f} m。")
            if result.reason == "stall":
                print("    停滞通常是打滑或轮子悬空：检查地面与电量后重试。")
            return None, False
    else:
        seconds = (float(args.linear_time) if args.linear_time is not None
                   else abs(args.distance) / args.linear_speed)
        driver.drive(args.linear_speed, 0.0, seconds)

    odom_distance = driver.distance
    print(f"\n  里程计报告位移: {odom_distance:.4f} m")
    if odom_distance < 0.2:
        print("  ✗ 里程计位移过小（<0.2m），小车可能没动或打滑。请检查底盘后重试。")
        return None, False

    est = None
    if use_lidar:
        cur, info = cc.capture_points(driver, grabber, seconds=1.5,
                                      r_max=args.r_max,
                                      zero_pub=lambda: publish_zero_once(driver))
        if cur is None:
            print(f"  ! 雷达当前帧采集失败：{info['reason']}，回落到人工测量。")
        else:
            print("\n  解算雷达真值中……")
            est = cc.estimate_translation(ref, cur, predicted_dx=odom_distance,
                                          mount_xy=mount, mount_yaw=mount_yaw)

    actual, source = resolve_actual(
        args, est, est.distance if est is not None else None,
        prompt="  请输入【钢尺实测】位移（米）", fallback_default=args.distance)
    if actual is None or actual <= 0.05:
        print("  ✗ 实测值无效，跳过直线标定。")
        return None, False
    print(f"  真值来源：{source}")

    # sigma 只在真正采用了雷达值时才作为门槛：人工钢尺量出来的值没有这个
    # 不确定度，拿雷达的 sigma 去否掉人工结果是不对的。
    return report_factors(old_factor, actual, odom_distance, unit="m",
                          scale_note="走过头/没走够",
                          sigma=(est.sigma_d
                                 if (est is not None
                                     and source == "雷达自动测量") else None))


def test_rotate(driver, grabber, old_factor, args, mount, mount_yaw):
    """原地旋转测量：标定 correct_factor_vth。"""
    print("\n" + "=" * 66)
    print("② 旋转测试 —— 标定 correct_factor_vth")
    print("=" * 66)
    turns = args.turns
    print(f"  指令角速度 {args.turn_speed:.3f} rad/s，原地转 {turns:g} 圈"
          f"（理论用时约 {abs(turns * 2 * math.pi) / args.turn_speed:.0f} s）")
    print("  请确保小车四周 0.5m 内无障碍，地面平整。")
    if HAVE_COMMON:
        print("\n  流程：采集雷达参考帧 → 自动原地转整圈 → 采集当前帧 → 解算真值")
        print("  提示：转整数圈时雷达真值只在 360° 的意义下可观测，工具用里程计")
        print("        预测限定搜索窗来消解多圈歧义；环境过于对称（正方形房间）时")
        print("        会判为不可靠并让你人工确认。")
    if not args.no_prompt:
        input("  准备好后按回车开始……")

    ref = None
    use_lidar = HAVE_COMMON and args.ground_truth != "manual"
    if use_lidar:
        ref, info = cc.capture_reference(driver, grabber, seconds=1.5,
                                         r_max=args.r_max,
                                         zero_pub=lambda: publish_zero_once(driver))
        if ref is None:
            print(f"  ! 雷达参考帧采集失败：{info['reason']}")
            print("    → 本次改为人工读数。")
            use_lidar = False

    driver.reset_accumulators()
    if HAVE_COMMON:
        result = driver.rotate_to_angle(turns * 360.0, args.turn_speed)
        if not result.ok:
            print(f"  ✗ 驱动未正常完成（{result.reason}），已停车。")
            if result.reason == "stall":
                print("    停滞通常是打滑或轮子悬空：检查地面与电量后重试。")
            return None, False
    else:
        driver.drive(0.0, args.turn_speed,
                     abs(turns * 2 * math.pi) / args.turn_speed)

    odom_deg = math.degrees(driver.yaw_total)
    print(f"\n  里程计报告转角: {odom_deg:.2f}°")
    if abs(odom_deg) < 45.0:
        print("  ✗ 里程计转角过小（<45°），小车可能没转。请检查底盘后重试。")
        return None, False

    est = None
    if use_lidar:
        cur, info = cc.capture_points(driver, grabber, seconds=1.5,
                                      r_max=args.r_max,
                                      zero_pub=lambda: publish_zero_once(driver))
        if cur is None:
            print(f"  ! 雷达当前帧采集失败：{info['reason']}，回落到人工读数。")
        else:
            print("\n  解算雷达真值中……")
            est = cc.estimate_rotation(ref, cur, predicted_dth=driver.yaw_total,
                                       mount_xy=mount, mount_yaw=mount_yaw)
            if est.reliable and math.hypot(est.residual_dx,
                                           est.residual_dy) > 0.03:
                print(f"  ! 旋转中心诊断：解出 {math.hypot(est.residual_dx, est.residual_dy) * 1000:.0f} mm "
                      f"残余平移。正常应接近 0；持续偏大说明 base_link 原点"
                      f"不在实际旋转中心上，转角真值会偏 —— 请核对雷达安装位姿。")

    lidar_deg = math.degrees(est.dth) if est is not None else None
    actual_deg, source = resolve_actual(
        args, est, lidar_deg,
        prompt=f"  请输入【实测】转角（度，转 {turns:g} 圈 = {turns * 360:g}）",
        fallback_default=turns * 360.0)
    if actual_deg is None or abs(actual_deg) < 10.0:
        print("  ✗ 实测值无效，跳过旋转标定。")
        return None, False
    print(f"  真值来源：{source}")

    # 里程计与实测的符号应一致；不一致说明转向相反，属于底盘问题而非标定
    if odom_deg * actual_deg < 0:
        print("  ✗ 里程计转向与实测方向相反！这不是标定问题，"
              "请检查电机接线/轮子方向。")
        return None, False

    return report_factors(old_factor, abs(actual_deg), abs(odom_deg), unit="°",
                          scale_note="转过头/没转够",
                          sigma=(math.degrees(est.sigma_dth)
                                 if (est is not None
                                     and source == "雷达自动测量") else None))


def print_summary(results, old_config):
    """汇总建议值 + 给出应用方法。只打印，不写任何文件。"""
    print("\n" + "=" * 66)
    print("标定结果汇总")
    print("=" * 66)
    print(f"  {'参数':<22}{'当前值':>12}{'建议值':>14}{'变化':>10}")
    print("  " + "-" * 56)

    suggestions = {}
    for key, label, new_value, reliable in results:
        old_value = old_config[key]
        if new_value is None:
            print(f"  {label:<20}{old_value:>13.6f}{'(未测)':>13}{'':>10}")
            continue
        delta_pct = (new_value - old_value) / old_value * 100.0
        flag = "" if reliable else "  (低置信)"
        print(f"  {label:<20}{old_value:>13.6f}{new_value:>13.6f}"
              f"{delta_pct:>+9.2f}%{flag}")
        if reliable:
            suggestions[key] = new_value

    if not suggestions:
        print("\n  没有得到可信的建议值。")
        if results:
            print("  （雷达判为不可靠时会自动回落到人工测量，可加 --ground-truth manual）")
        return

    print("\n  —— 应用方法（二选一）——")
    print("\n  [推荐] 写入 Web 端参数配置页 →「底盘速度标定」分组，"
          "保存后重启底盘：")
    lines = ", ".join(f"{k}={v:.6f}" for k, v in suggestions.items())
    print(f"    {lines}")
    print(f"\n  或直接改生成文件 {GENERATED_DIR / 'chassis.json'} 后重启底盘。")
    print("  （Web 端参数配置页从未保存过底盘参数时该文件可能不存在，"
          "保存一次即会生成）")

    print("\n  [临时] 对运行中的节点直接生效（重启后失效）：")
    for key, value in suggestions.items():
        print(f"    ros2 param set {NODE_NAME} {key} {value:.6f}")

    print("\n  改完建议复跑本工具验证：偏差应从原先的百分比降到 ±2% 以内。")
    if not all(item[3] for item in results if item[2] is not None):
        print("  注意：标了 (低置信) 的项未列入上面的建议，请先改善测量条件。")


def main():
    parser = argparse.ArgumentParser(
        description="OriginBot 里程计标定（自动行驶 + 雷达实测真值）")
    parser.add_argument("--linear", action="store_true", help="只做直线标定")
    parser.add_argument("--rotate", action="store_true", help="只做旋转标定")
    parser.add_argument("--distance", type=float, default=1.0,
                        help="直线测试的目标行程，米（默认 1.0）")
    parser.add_argument("--linear-speed", type=float, default=0.12,
                        help="直线测试速度 m/s（默认 0.12，慢速更准）")
    parser.add_argument("--turns", type=float, default=2.0,
                        help="旋转测试圈数（默认 2；整数圈即可，真值由雷达给出）")
    parser.add_argument("--turn-speed", type=float, default=0.6,
                        help="旋转测试角速度 rad/s（默认 0.6）")
    parser.add_argument("--ground-truth", choices=("auto", "lidar", "manual"),
                        default="auto",
                        help="真值来源：auto=优先雷达（默认）；lidar=只用雷达；"
                             "manual=只人工量尺")
    parser.add_argument("--manual", action="store_true",
                        help="等价于 --ground-truth manual")
    parser.add_argument("--check", action="store_true",
                        help="只做静止自检（不动车）后退出")
    parser.add_argument("--no-prompt", action="store_true",
                        help="不等待回车，自动开始（适合脚本调用）")
    parser.add_argument("--r-max", type=float, default=None,
                        help="雷达有效距离上限（米），默认 12.0（STL06N 经验值）。"
                             "远场噪点多时调小，开阔场地可调大")
    parser.add_argument("--mount-x", type=float, default=None,
                        help="覆盖雷达安装 X 偏移（米），默认读 lidar.json")
    parser.add_argument("--mount-y", type=float, default=None,
                        help="覆盖雷达安装 Y 偏移（米），默认读 lidar.json")
    parser.add_argument("--mount-yaw", type=float, default=None,
                        help="覆盖雷达安装偏航（弧度），默认读 lidar.json")
    parser.add_argument("--linear-time", type=float, default=None,
                        help="已弃用：改用 --distance。给出时按 速度×时间 换算目标里程")
    parser.add_argument("--factor-vx", type=float, default=None,
                        help="手动指定当前 correct_factor_vx（跳过读活参数/json）")
    parser.add_argument("--factor-vth", type=float, default=None,
                        help="手动指定当前 correct_factor_vth")
    args = parser.parse_args()

    if args.manual:
        args.ground_truth = "manual"

    if args.r_max is None:
        args.r_max = cc.R_MAX if HAVE_COMMON else 12.0

    if args.linear_time is not None and "--distance" not in sys.argv:
        args.distance = args.linear_speed * args.linear_time
        print(f"提示：--linear-time 已弃用，已按 速度×时间 换算为 "
              f"--distance {args.distance:.3f}")

    do_linear = args.linear or not args.rotate
    do_rotate = args.rotate or not args.linear

    config = load_chassis_config()
    old_vx, src_vx = read_factor(args.factor_vx, "correct_factor_vx", config)
    old_vth, src_vth = read_factor(args.factor_vth, "correct_factor_vth", config)

    mount, mount_yaw = resolve_mount(args) if HAVE_COMMON else ((0.15, 0.0), 0.0)

    print("=" * 66)
    print("OriginBot 里程计标定（自动行驶 + 雷达实测真值）")
    print("=" * 66)
    print(f"  基准 correct_factor_vx  = {old_vx:.6f}（{src_vx}）")
    print(f"  基准 correct_factor_vth = {old_vth:.6f}（{src_vth}）")
    if HAVE_COMMON:
        print(f"  雷达安装偏移 mount      = x={mount[0]:+.3f} y={mount[1]:+.3f} "
              f"yaw={math.degrees(mount_yaw):+.1f}°")
    else:
        print("  ⚠ 未找到 calib_common.py：无法自动行驶/雷达测真值，"
              "只能用人工量尺。")
    print("\n  ⚠ 本工具会让小车【自行移动】，请清空周围 1.5m 并扶稳电源。")
    print("  本工具只打印建议值，不写任何配置文件。")

    if not HAVE_COMMON and args.ground_truth != "manual":
        print("\n  ! 缺少 calib_common.py → 强制转为人工测量模式。")
        args.ground_truth = "manual"

    rclpy.init()
    driver = cc.MotionDriver("odom_calib") if HAVE_COMMON else FallbackDriver()
    grabber = cc.ScanGrabber(driver) if HAVE_COMMON else None
    try:
        if not driver.wait_for_odom():
            print("\n✗ 未收到 /odom。请确认底盘已启动：")
            print("    ros2 launch originbot_bringup originbot.launch.py")
            return 1

        # 人工模式下不需要雷达，跳过需要雷达的静止自检（--check 时总是做）
        if args.check or (HAVE_COMMON and args.ground_truth != "manual"):
            if not do_static_check(driver, grabber, args, mount, mount_yaw):
                print("\n  静止自检未通过时继续标定会得到不可信的数值，已中止。")
                print("  排查：周围是否有人走动/雷达被挡/小车是否真的静止。")
                print("  确实想跳过自检：加 --ground-truth manual 用人工量尺。")
                return 1
        if args.check:
            print("\n自检结束（--check）。")
            return 0

        results = []
        if do_linear:
            new_vx, ok_vx = test_linear(driver, grabber, old_vx, args,
                                        mount, mount_yaw)
            results.append(("correct_factor_vx", "线速度标定 vx", new_vx, ok_vx))
        if do_rotate:
            new_vth, ok_vth = test_rotate(driver, grabber, old_vth, args,
                                          mount, mount_yaw)
            results.append(("correct_factor_vth", "角速度标定 vth", new_vth,
                            ok_vth))

        driver.stop_robot()
        print_summary(results, {"correct_factor_vx": old_vx,
                                "correct_factor_vth": old_vth})
        return 0
    except KeyboardInterrupt:
        print("\n已中断（底盘自身的 600ms 看门狗也会让它停下）。")
        return 130
    finally:
        try:
            driver.stop_robot()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
