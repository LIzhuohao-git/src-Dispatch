#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""camera_info 中转节点（带标定内参保底）。

hobot_usb_cam 只在启动瞬间发布一次 /camera_info（RELIABLE + VOLATILE、非 latched）。
由于 DDS 发现是异步的，即使相机延迟启动，也可能在订阅方被"发现"之前就发完了这条
单次消息，导致后启动的节点（如 aruco_localization）永远拿不到内参。

本节点在启动时立即发布一份兜底相机内参（TRANSIENT_LOCAL / latched），保证任意时刻
订阅 latched 话题的节点都能拿到 camera_info；随后订阅原始 /camera_info，收到真实
内参时用真实值覆盖兜底值。这样彻底消除"接住单次发布"的启动竞态。

兜底内参的来源优先级：
  1. 标定文件 calibration_file（由 originbot_bringup 的 calibrate_camera.py 生成，
     实测值，推荐。启动日志为 INFO）；
  2. fx/fy/cx/cy 参数（占位默认 500/500/320/240，仅供画面预览；ArUco 的 PnP
     测距会有系统性偏差，启动日志为 WARN 提醒标定）。

配合 camera.launch.py 使用：本节点在相机前启动，相机延迟 3s 启动，尽量接住真实内参。
"""

import os

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


def load_calibration(path):
    """读取 calibrate_camera.py 生成的标定文件，失败返回 None。"""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError):
        return None
    if not isinstance(data, dict):
        return None
    matrix = data.get("camera_matrix")
    distortion = data.get("distortion_coefficients")
    kdata = matrix.get("data") if isinstance(matrix, dict) else None
    ddata = distortion.get("data") if isinstance(distortion, dict) else None
    if not kdata or len(kdata) < 9:
        return None
    ddata = [float(v) for v in (ddata or [])]
    while len(ddata) < 5:
        ddata.append(0.0)
    size = data.get("image_size") or []
    return {
        "fx": float(kdata[0]),
        "fy": float(kdata[4]),
        "cx": float(kdata[2]),
        "cy": float(kdata[5]),
        "d": ddata[:5],
        "width": int(size[0]) if len(size) > 0 and size[0] else None,
        "height": int(size[1]) if len(size) > 1 and size[1] else None,
        "rms": data.get("rms_reproj_error"),
    }


class CameraInfoRelay(Node):
    def __init__(self):
        super().__init__("camera_info_relay")

        self.declare_parameter("source_topic", "/camera_info")
        self.declare_parameter("latched_topic", "/camera_info_latched")
        self.declare_parameter("frame_id", "camera_optical")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fx", 500.0)
        self.declare_parameter("fy", 500.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter(
            "calibration_file",
            os.path.join(
                os.path.expanduser("~/.originbot_delivery/config"),
                "camera_calibration.yaml",
            ),
        )

        source = self.get_parameter("source_topic").value
        latched = self.get_parameter("latched_topic").value
        calibration_path = self.get_parameter("calibration_file").value
        self._calibration = load_calibration(calibration_path)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(CameraInfo, latched, latched_qos)
        # 原始话题用默认（RELIABLE + VOLATILE），与 hobot_usb_cam 的发布 QoS 匹配
        self._sub = self.create_subscription(CameraInfo, source, self._on_info, 10)
        self._received = False

        # 立即发布兜底内参；在收到真实内参前，每 2s 补发一次，
        # 避免首次 publish 早于 DDS writer 就绪导致 latch 丢失。
        self._pub.publish(self._make_default())
        self._timer = self.create_timer(2.0, self._republish_default)

        if self._calibration is not None:
            rms = self._calibration.get("rms")
            rms_text = f"{rms:.2f}px" if isinstance(rms, (int, float)) else "未知"
            self.get_logger().info(
                "camera_info_relay: {} -> {} (latched, 兜底内参来自标定文件: {}, "
                "fx={:.1f}, fy={:.1f}, cx={:.0f}, cy={:.0f}, RMS={})".format(
                    source, latched, calibration_path,
                    self._calibration["fx"], self._calibration["fy"],
                    self._calibration["cx"], self._calibration["cy"], rms_text,
                )
            )
        else:
            self.get_logger().warning(
                "camera_info_relay: {} -> {} (latched)。未找到标定文件 {}，"
                "兜底使用占位内参 (fx=fy=500)——仅够画面预览，会导致 ArUco "
                "定位/测距系统性偏差。请运行 `ros2 run originbot_bringup "
                "calibrate_camera` 生成标定文件".format(source, latched, calibration_path)
            )

    def _make_default(self):
        calib = self._calibration
        if calib is not None:
            fx = calib["fx"]
            fy = calib["fy"]
            cx = calib["cx"]
            cy = calib["cy"]
            d = list(calib["d"])
            width = calib["width"] or int(self.get_parameter("width").value)
            height = calib["height"] or int(self.get_parameter("height").value)
        else:
            fx = float(self.get_parameter("fx").value)
            fy = float(self.get_parameter("fy").value)
            cx = float(self.get_parameter("cx").value)
            cy = float(self.get_parameter("cy").value)
            d = [0.0, 0.0, 0.0, 0.0, 0.0]
            width = int(self.get_parameter("width").value)
            height = int(self.get_parameter("height").value)
        msg = CameraInfo()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.d = d
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _republish_default(self):
        if not self._received:
            self._pub.publish(self._make_default())

    def _on_info(self, msg):
        self._pub.publish(msg)
        if not self._received:
            self._received = True
            self.get_logger().info(
                f"已收到并转发真实 camera_info (frame_id={msg.header.frame_id}, "
                f"{msg.width}x{msg.height})")


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
