#!/usr/bin/env python3
"""车队身份与回桩位姿（robot_identity）配置。

每台车一份 per-robot 配置，与"全车队共用"的配置束（map/stations/routes）区分开：
这里放的是"这辆车是谁、停在地图系哪个位置、开机默认要什么控制权"。

文件：~/.originbot_delivery/config/robot_identity.json
  {
    "robot_id": "rb03",                                  # 中心主键，同时决定 MQTT topic
    "boot_pose": {"x": 0.2, "y": -1.5, "yaw": 1.57},     # 该车充电桩/待命位（map 系），可为 null
    "default_authority": "debug"                          # 上电默认控制权
  }

不使用 ArUco 时，boot_pose 是 AMCL 冷启动初始位姿的唯一自动来源：delivery_server
启动导航栈时把它注入 /initialpose，避免每台车都要人工在 Web 上点一次"设定位姿"。
"""

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

ROBOT_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
AUTHORITY_VALUES = ("controlled", "debug")

DEFAULT_IDENTITY = {
    "robot_id": "",
    "boot_pose": None,
    "default_authority": "debug",
}


def default_identity_path():
    return Path.home() / ".originbot_delivery" / "config" / "robot_identity.json"


def load_identity(path):
    """读取身份配置；文件缺失或损坏时返回默认值副本（不抛异常）。"""
    data = dict(DEFAULT_IDENTITY)
    try:
        with open(path, "r", encoding="utf-8") as handle:
            loaded = json.load(handle)
    except (OSError, ValueError):
        return data
    if isinstance(loaded, dict):
        data.update(loaded)
    if not isinstance(data.get("robot_id"), str):
        data["robot_id"] = ""
    if data.get("default_authority") not in AUTHORITY_VALUES:
        data["default_authority"] = DEFAULT_IDENTITY["default_authority"]
    pose = data.get("boot_pose")
    if pose is not None:
        try:
            data["boot_pose"] = {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw": float(pose.get("yaw", 0.0)),
            }
        except (KeyError, TypeError, ValueError):
            data["boot_pose"] = None
    return data


def save_identity(path, data):
    directory = Path(path).parent
    directory.mkdir(parents=True, exist_ok=True)
    tmp_path = str(path) + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False)
    os.replace(tmp_path, path)


def validate_robot_id(robot_id):
    if not ROBOT_ID_RE.match(robot_id or ""):
        raise ValueError(
            f"robot_id 非法：{robot_id!r}（只允许字母/数字/._-，且以字母或数字开头，"
            "需可直接用于 MQTT topic 与文件名）")
    return robot_id


def current_map_pose(timeout_sec=8.0, map_frame="map", base_frame="base_footprint"):
    """从 TF 取当前 map→base 位姿，返回 (x, y, yaw)。用于把"车现在停的位置"记为 boot_pose。

    ROS 相关模块延迟导入：只做 --show / 写 robot_id 时无需 ROS 环境也能跑。
    """
    import math

    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    try:
        node = Node("robot_identity_capture")
        buffer = Buffer()
        TransformListener(buffer, node)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = buffer.lookup_transform(map_frame, base_frame, Time())
            except Exception:
                continue
            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return t.x, t.y, yaw
        raise RuntimeError(
            f"{timeout_sec:.0f} 秒内未取到 {map_frame}→{base_frame} TF，"
            "请确认底盘与定位（AMCL/导航栈）已启动")
    finally:
        rclpy.shutdown()


def describe(data):
    pose = data.get("boot_pose")
    pose_text = (
        "未设置（冷启动需人工在 Web 设定位姿）"
        if pose is None
        else "x={:.3f} y={:.3f} yaw={:.3f}".format(pose["x"], pose["y"], pose["yaw"])
    )
    return (
        f"文件      : {data.get('_path', default_identity_path())}\n"
        f"robot_id  : {data.get('robot_id') or '（未分配）'}\n"
        f"boot_pose : {pose_text}\n"
        f"默认控制权: {data.get('default_authority')}"
    )


def main(args=None):
    parser = argparse.ArgumentParser(
        prog="set_robot_id",
        description="读写车端 robot_id / boot_pose / default_authority 配置",
    )
    parser.add_argument("--config", default=str(default_identity_path()),
                        help="身份配置文件路径（默认 ~/.originbot_delivery/config/robot_identity.json）")
    parser.add_argument("--show", action="store_true", help="打印当前配置（默认动作）")
    parser.add_argument("--robot-id", help="设置本车 robot_id（需全局唯一）")
    parser.add_argument("--default-authority", choices=AUTHORITY_VALUES,
                        help="上电默认控制权：controlled=受控 / debug=本地接管")
    parser.add_argument("--boot-pose", nargs=3, type=float, metavar=("X", "Y", "YAW"),
                        help="手工指定回桩位姿（map 系，米/弧度）")
    parser.add_argument("--capture-boot-pose", action="store_true",
                        help="把当前 map 位姿记为 boot_pose（需定位已启动）")
    parser.add_argument("--clear-boot-pose", action="store_true", help="清除 boot_pose")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    opts = parser.parse_args(args)

    path = opts.config
    data = load_identity(path)
    changed = False

    if opts.robot_id is not None:
        data["robot_id"] = validate_robot_id(opts.robot_id.strip())
        changed = True
    if opts.default_authority is not None:
        data["default_authority"] = opts.default_authority
        changed = True
    if opts.clear_boot_pose:
        data["boot_pose"] = None
        changed = True
    if opts.boot_pose is not None:
        x, y, yaw = opts.boot_pose
        data["boot_pose"] = {"x": x, "y": y, "yaw": yaw}
        changed = True
    if opts.capture_boot_pose:
        x, y, yaw = current_map_pose(
            map_frame=opts.map_frame, base_frame=opts.base_frame)
        data["boot_pose"] = {"x": x, "y": y, "yaw": yaw}
        changed = True
    if changed:
        save_identity(path, data)
        data = load_identity(path)
        print(f"已写入 {path}")

    print(describe({**data, "_path": path}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
