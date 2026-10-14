#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 ArUco 标志位重新锚定到当前 AMCL 位姿（相机实测，覆盖 markers.json）。

什么时候需要：
  相机内参变更（重新标定）后，已登记标志位的 map 系坐标仍是按旧内参解算的，
  与新 PnP 观测不自洽——典型症状：识别到标志位后 /initialpose 修正方向错误，
  aruco 日志出现"偏差 dyaw≈180°"或位置偏差远超 0.3m，AMCL 越修越偏。

原理：
  导航态下 AMCL 只靠激光定位是可信的。把机器人停在标志位正前方，用当前内参
  解算标志位相对位姿，再与 AMCL 位姿、相机外参组合，得到标志位在 map 系的
  真值位姿，覆盖写回 markers.json：

      T_map_marker = T_map_base(TF) · T_base_camera(TF) · T_camera_marker(PnP)

  机器人位姿取自 TF（map -> base_footprint）：AMCL 持续发布 map->odom 的 TF，
  与机器人是否静止无关。（不能订阅 /amcl_pose——nav2 AMCL 只在运动量超过
  阈值后才发布该话题，静止观测时它永远是哑的。）

用法：
  1. 先在 Web 端删除要重测的标志位（避免旧坏坐标继续污染 AMCL）；
  2. Web 端"启动导航"，等 AMCL 收敛（地图上机器人图标与实际一致）；
  3. 手动把机器人停到标志位正前方 0.5~1.5m、标志位于画面中央且基本正对，
     机器人静止（推车/遥控均可）；
  4. 运行：
       ros2 run originbot_delivery reregister_markers --marker-id 3
     （不指定 --marker-id 时，画面里检测到的所有 id 都会重新锚定）
  5. aruco_localization 约 1s 内热加载新坐标，无需重启，继续导航验证；
     验证标准：经过标志位时不再频繁出现"ArUco 修正 AMCL 位姿"日志，
     或修正偏差明显小于 0.3m 且方向合理。

常用参数：
  --marker-id N      只锚定指定 id
  --frames 15        采样观测数（默认 15）
  --max-stddev 0.08  观测位置标准差上限（米），超了判失败、不写盘
  --marker-size 0.07 标志位边长（米），默认取 markers.json 的 marker_size
"""

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

DEFAULT_MARKER_FILE = os.path.expanduser(
    "~/.originbot_delivery/config/markers.json"
)
MIN_DISTANCE = 0.2
MAX_DISTANCE = 3.0


def quaternion_to_rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_from_rvec_tvec(rvec, tvec):
    rmat, _ = cv2.Rodrigues(rvec)
    matrix = np.eye(4)
    matrix[:3, :3] = rmat
    matrix[:3, 3] = np.asarray(tvec).reshape(3)
    return matrix


def invert_transform(matrix):
    inv = np.eye(4)
    rot = matrix[:3, :3]
    t = matrix[:3, 3]
    inv[:3, :3] = rot.T
    inv[:3, 3] = -rot.T @ t
    return inv


def matrix_to_pose(matrix):
    x, y, z = matrix[0, 3], matrix[1, 3], matrix[2, 3]
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return x, y, z, yaw


def transform_to_matrix(transform):
    matrix = np.eye(4)
    t = transform.transform.translation
    matrix[0, 3] = t.x
    matrix[1, 3] = t.y
    matrix[2, 3] = t.z
    matrix[:3, :3] = quaternion_to_rotation(transform.transform.rotation)
    return matrix


class Reanchor(Node):
    def __init__(self, args):
        super().__init__("reregister_markers")
        self.args = args
        self.camera_matrix = None
        self.distortion = None
        self.latest_frame = None

        import cv_bridge
        self.cv_bridge = cv_bridge.CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CameraInfo, args.camera_info, self._on_info, info_qos)
        self.create_subscription(Image, args.image_topic, self._on_image, 5)

    def _on_info(self, msg):
        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(msg.d, dtype=np.float64).reshape(-1)

    def _on_image(self, msg):
        try:
            self.latest_frame = self.cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8")
        except Exception:
            pass

    def lookup(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time())
            return transform_to_matrix(transform)
        except Exception:
            return None


def estimate_pose(corners, marker_size, camera_matrix, distortion):
    """与 aruco_localization 相同的 PnP 约定，保证新旧坐标可比。"""
    try:
        result = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, camera_matrix, distortion
        )
        rvecs, tvecs = result[0], result[1]
    except (AttributeError, TypeError, ValueError, cv2.error):
        half = marker_size / 2.0
        obj_points = np.array(
            [[-half, half, 0.0], [half, half, 0.0],
             [half, -half, 0.0], [-half, -half, 0.0]],
            dtype=np.float32,
        )
        rvecs, tvecs = [], []
        for corner in corners:
            _, rvec, tvec = cv2.solvePnP(
                obj_points, corner, camera_matrix, distortion)
            rvecs.append(rvec)
            tvecs.append(tvec)
    return rvecs, tvecs


def detect_markers(gray):
    try:
        parameters = cv2.aruco.DetectorParameters_create()
    except AttributeError:
        parameters = cv2.aruco.DetectorParameters()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    try:
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        corners, ids, _ = cv2.aruco.detectMarkers(
            dictionary, gray, parameters=parameters)
    return corners, ids


def save_markers(path, updates):
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    markers = {int(m["id"]): m for m in data.get("markers", [])
               if isinstance(m.get("id"), (int, float))}
    for marker_id, entry in updates.items():
        markers[marker_id] = {
            "id": marker_id,
            "size": data.get("marker_size", 0.07),
            "x": entry["x"],
            "y": entry["y"],
            "yaw": entry["yaw"],
        }
    data["markers"] = [markers[k] for k in sorted(markers)]
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False)
    os.replace(tmp_path, path)


def main():
    parser = argparse.ArgumentParser(
        description="用当前内参+AMCL 位姿重新锚定 ArUco 标志位")
    parser.add_argument("--marker-id", type=int, default=None,
                        help="只锚定指定 id（默认：检测到的全部 id）")
    parser.add_argument("--frames", type=int, default=15,
                        help="采样观测数（默认 15）")
    parser.add_argument("--max-stddev", type=float, default=0.08,
                        help="观测位置标准差上限米（默认 0.08）")
    parser.add_argument("--marker-size", type=float, default=None,
                        help="标志位边长米（默认取 markers.json 的 marker_size）")
    parser.add_argument("--image-topic", default="/image")
    parser.add_argument("--camera-info", default="/camera_info_latched")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--camera-frame", default="camera_optical")
    parser.add_argument("--marker-file", default=DEFAULT_MARKER_FILE)
    args = parser.parse_args()

    if not os.path.exists(args.marker_file):
        print(f"错误：找不到 {args.marker_file}", file=sys.stderr)
        sys.exit(1)
    with open(args.marker_file, "r", encoding="utf-8") as handle:
        store = json.load(handle)
    marker_size = args.marker_size or float(store.get("marker_size", 0.07))

    rclpy.init(args=None)
    node = Reanchor(args)
    print(f"标志位边长 {marker_size*100:.0f}cm，目标 {args.frames} 次观测")

    print(f"等待 TF {args.map_frame} -> {args.base_frame}"
          "（需先在 Web 端启动导航并等 AMCL 收敛）…")
    deadline = time.time() + 20.0
    map_to_base = None
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        map_to_base = node.lookup(args.map_frame, args.base_frame)
        if map_to_base is not None:
            break
    if map_to_base is None:
        print(f"错误：拿不到 TF {args.map_frame}->{args.base_frame}。"
              "请确认导航在运行且 AMCL 已收敛。", file=sys.stderr)
        node.destroy_node(); rclpy.try_shutdown(); sys.exit(1)
    pose = matrix_to_pose(map_to_base)
    print(f"定位已就绪: x={pose[0]:.3f} y={pose[1]:.3f} "
          f"yaw={math.degrees(pose[3]):.1f}°")

    while node.camera_matrix is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"内参已就绪: fx={node.camera_matrix[0,0]:.1f}")

    base_to_camera = node.lookup(args.base_frame, args.camera_frame)
    while base_to_camera is None and rclpy.ok():
        time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.05)
        base_to_camera = node.lookup(args.base_frame, args.camera_frame)
    if base_to_camera is None:
        print(f"错误：拿不到 TF {args.base_frame}->{args.camera_frame}，"
              "请确认 ArUco 子系统在运行（它负责发布相机外参 TF）。",
              file=sys.stderr)
        node.destroy_node(); rclpy.try_shutdown(); sys.exit(1)

    observations = {}  # id -> list[(x, y, yaw, dist)]
    try:
        while len(observations) == 0 or min(
                len(v) for v in observations.values()) < args.frames:
            rclpy.spin_once(node, timeout_sec=0.05)
            frame = node.latest_frame
            node.latest_frame = None
            if frame is None:
                time.sleep(0.05)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = detect_markers(gray)
            if ids is None or len(ids) == 0:
                time.sleep(0.3)
                continue
            map_to_base = node.lookup(args.map_frame, args.base_frame)
            if map_to_base is None:
                print("警告：TF 断开，暂停采样"); time.sleep(0.5); continue
            map_to_camera = map_to_base @ base_to_camera
            rvecs, tvecs = estimate_pose(
                corners, marker_size, node.camera_matrix, node.distortion)
            for idx, marker_id_array in enumerate(ids):
                marker_id = int(marker_id_array[0])
                if args.marker_id is not None and marker_id != args.marker_id:
                    continue
                distance = float(np.linalg.norm(tvecs[idx]))
                if distance < MIN_DISTANCE or distance > MAX_DISTANCE:
                    print(f"  id={marker_id} 距离 {distance:.2f}m 超出"
                          f" [{MIN_DISTANCE},{MAX_DISTANCE}]m，跳过")
                    continue
                map_to_marker = map_to_camera @ matrix_from_rvec_tvec(
                    rvecs[idx], tvecs[idx])
                x, y, _, yaw = matrix_to_pose(map_to_marker)
                observations.setdefault(marker_id, []).append(
                    (x, y, yaw, distance))
            done = {k: len(v) for k, v in observations.items()}
            print(f"已采样 {done} / {args.frames}")
            time.sleep(0.4)
    except KeyboardInterrupt:
        print("\n手动中断")

    if not observations:
        print("错误：没采到任何观测。请确认标志位入镜、距离 0.2~3.0m。",
              file=sys.stderr)
        node.destroy_node(); rclpy.try_shutdown(); sys.exit(1)

    updates = {}
    failed = False
    for marker_id, obs in observations.items():
        xs = np.asarray([o[0] for o in obs])
        ys = np.asarray([o[1] for o in obs])
        yaw = math.atan2(sum(math.sin(o[2]) for o in obs),
                         sum(math.cos(o[2]) for o in obs))
        std = max(xs.std(), ys.std())
        dists = np.asarray([o[3] for o in obs])
        print(f"\n===== id={marker_id}（{len(obs)} 次观测，"
              f"平均距离 {dists.mean():.2f}m） =====")
        old = next((m for m in store.get("markers", [])
                    if int(m.get("id", -1)) == marker_id), None)
        if old:
            print(f"  旧值: x={old['x']:.3f} y={old['y']:.3f} "
                  f"yaw={math.degrees(old['yaw']):.1f}°")
        print(f"  新值: x={xs.mean():.3f} y={ys.mean():.3f} "
              f"yaw={math.degrees(yaw):.1f}°  位置标准差 {std*100:.1f}cm")
        if std > args.max_stddev:
            print(f"  !! 标准差超限（>{args.max_stddev}m），该 id 不写盘，"
                  "请确认机器人停稳、标志位正对后重试")
            failed = True
            continue
        updates[marker_id] = {"x": float(xs.mean()), "y": float(ys.mean()),
                              "yaw": yaw}

    if updates:
        save_markers(args.marker_file, updates)
        print(f"\n已写回 {args.marker_file}（{len(updates)} 个 id），"
              "aruco_localization 约 1s 内自动加载，无需重启。")
        print("验证：继续导航经过标志位，不应再出现大幅'ArUco 修正 AMCL 位姿'日志。")
    if failed:
        sys.exit(2)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
