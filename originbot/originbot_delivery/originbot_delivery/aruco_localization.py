#!/usr/bin/env python3

import json
import math
import os
import time

import cv2
import cv_bridge
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def matrix_from_xyz_yaw(x, y, z, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([
        [c, -s, 0.0, x],
        [s, c, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
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


class MarkerStore:
    def __init__(self, path):
        self.path = path
        self.last_mtime = None
        self.dictionary_name = "DICT_6X6_1000"
        self.marker_size = 0.07
        self.markers = {}
        self.reload()

    def reload_if_changed(self):
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            if self.last_mtime is not None:
                self.last_mtime = None
            return False
        if self.last_mtime is None:
            self.reload()
            return True
        if mtime != self.last_mtime:
            self.reload()
            return True
        return False

    def reload(self):
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, ValueError):
            return

        self.dictionary_name = str(data.get("dictionary", "DICT_6X6_1000"))
        self.marker_size = float(data.get("marker_size", 0.07))
        self.markers = {}
        for marker in data.get("markers", []):
            try:
                marker_id = int(marker["id"])
                self.markers[marker_id] = {
                    "x": float(marker["x"]),
                    "y": float(marker["y"]),
                    "yaw": float(marker.get("yaw", 0.0)),
                }
            except (KeyError, TypeError, ValueError):
                continue
        try:
            self.last_mtime = os.path.getmtime(self.path)
        except OSError:
            self.last_mtime = None

    def save(self):
        data = {
            "dictionary": self.dictionary_name,
            "marker_size": self.marker_size,
            "markers": [
                {
                    "id": marker_id,
                    "size": self.marker_size,
                    "x": marker["x"],
                    "y": marker["y"],
                    "yaw": marker.get("yaw", 0.0),
                }
                for marker_id, marker in sorted(self.markers.items())
            ],
        }
        tmp_path = self.path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2, ensure_ascii=False)
        os.replace(tmp_path, self.path)
        try:
            self.last_mtime = os.path.getmtime(self.path)
        except OSError:
            pass

    def get(self, marker_id):
        return self.markers.get(int(marker_id))


class ArucoLocalization(Node):
    def __init__(self):
        super().__init__("aruco_localization")
        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("camera_info_topic", "/camera_info_latched")
        self.declare_parameter("camera_frame", "camera_optical")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("marker_file", "")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("max_process_fps", 2.0)
        self.declare_parameter("detect_scale", 0.35)
        self.declare_parameter("min_marker_distance", 0.2)
        self.declare_parameter("max_marker_distance", 3.0)
        self.declare_parameter("position_covariance", 0.03)
        self.declare_parameter("yaw_covariance", 0.05)
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("correction_min_interval", 5.0)
        self.declare_parameter("correction_position_threshold", 0.3)
        self.declare_parameter("correction_yaw_threshold", 0.26)
        self.declare_parameter("auto_register", True)
        self.declare_parameter("register_min_observations", 5)
        self.declare_parameter("register_min_span_sec", 1.0)
        self.declare_parameter("register_amcl_timeout", 10.0)
        self.declare_parameter("register_max_stddev", 0.15)

        marker_file = self.get_parameter("marker_file").value
        if not marker_file:
            from ament_index_python.packages import get_package_share_directory

            config_dir = os.path.expanduser("~/.originbot_delivery/config")
            os.makedirs(config_dir, exist_ok=True)
            marker_file = os.path.join(config_dir, "markers.json")
            package_marker_file = os.path.join(
                get_package_share_directory("originbot_delivery"),
                "config",
                "markers.json",
            )
            if not os.path.exists(marker_file) and os.path.exists(package_marker_file):
                import shutil

                shutil.copyfile(package_marker_file, marker_file)
        self.marker_store = MarkerStore(marker_file)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.publish_debug_image = self.get_parameter("publish_debug_image").value
        self.max_process_fps = float(self.get_parameter("max_process_fps").value)
        self.detect_scale = float(self.get_parameter("detect_scale").value)
        self._last_process_time = 0.0
        self._last_reload_check = 0.0
        self.min_marker_distance = self.get_parameter("min_marker_distance").value
        self.max_marker_distance = self.get_parameter("max_marker_distance").value
        self.position_covariance = self.get_parameter("position_covariance").value
        self.yaw_covariance = self.get_parameter("yaw_covariance").value
        self.amcl_pose_topic = self.get_parameter("amcl_pose_topic").value
        self.initialpose_topic = self.get_parameter("initialpose_topic").value
        self.correction_min_interval = self.get_parameter(
            "correction_min_interval"
        ).value
        self.correction_position_threshold = self.get_parameter(
            "correction_position_threshold"
        ).value
        self.correction_yaw_threshold = self.get_parameter(
            "correction_yaw_threshold"
        ).value
        self.auto_register = self.get_parameter("auto_register").value
        self.register_min_observations = int(
            self.get_parameter("register_min_observations").value
        )
        self.register_min_span_sec = float(
            self.get_parameter("register_min_span_sec").value
        )
        self.register_amcl_timeout = float(
            self.get_parameter("register_amcl_timeout").value
        )
        self.register_max_stddev = float(
            self.get_parameter("register_max_stddev").value
        )

        self.camera_matrix = None
        self.distortion = None

        self.cv_bridge = cv_bridge.CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 5
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/aruco/pose", 10
        )
        self.status_pub = self.create_publisher(String, "/aruco/status", 10)
        self.compressed_pub = self.create_publisher(
            CompressedImage, "/aruco/compressed", 5
        )
        self.amcl_pose = None
        self.last_correction_time = 0.0
        self.register_observations = {}
        self.registered_ids = set()
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.amcl_pose_callback,
            10,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.initialpose_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, "/aruco/debug_image", 5)
        else:
            self.debug_pub = None

        if self.auto_register:
            self.get_logger().info(
                "ArUco 自动登记已开启：建图期间（未收到 AMCL 位姿）检测到的新标志位"
                "将自动写入 markers.json（map 系坐标）"
            )

        self.get_logger().info(
            f"ArUco localization ready with {len(self.marker_store.markers)} known markers"
        )

    def camera_info_callback(self, msg):
        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(msg.d, dtype=np.float64).reshape(-1)

    def amcl_pose_callback(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.amcl_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            time.time(),
        )

    def _maybe_correct_amcl(self, pose):
        """AMCL 位姿与 ArUco 位姿偏差较大时，低频重设 AMCL 初始位姿。"""
        if self.amcl_pose is None:
            return
        now = time.time()
        if now - self.last_correction_time < self.correction_min_interval:
            return
        amcl_x, amcl_y, amcl_yaw, _ = self.amcl_pose
        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        yaw = yaw_from_quaternion(pose.pose.pose.orientation)
        dx = x - amcl_x
        dy = y - amcl_y
        dyaw = math.atan2(math.sin(yaw - amcl_yaw), math.cos(yaw - amcl_yaw))
        if (
            math.hypot(dx, dy) < self.correction_position_threshold
            and abs(dyaw) < self.correction_yaw_threshold
        ):
            return
        correction = PoseWithCovarianceStamped()
        correction.header.frame_id = self.map_frame
        correction.header.stamp = Time().to_msg()
        correction.pose.pose.position.x = x
        correction.pose.pose.position.y = y
        correction.pose.pose.orientation = quaternion_from_yaw(yaw)
        correction.pose.covariance = [
            0.0625, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0625, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0625,
        ]
        self.initial_pose_pub.publish(correction)
        self.last_correction_time = now
        self.get_logger().info(
            "ArUco 修正 AMCL 位姿: "
            f"偏差 dx={dx:.2f} dy={dy:.2f} dyaw={math.degrees(dyaw):.1f}°"
        )

    def image_callback(self, msg):
        now = time.monotonic()
        if now - self._last_process_time < 1.0 / self.max_process_fps:
            return
        self._last_process_time = now

        if self.camera_matrix is None or self.distortion is None:
            self.get_logger().debug("Waiting for CameraInfo", throttle_duration_sec=5.0)
            # 尚未收到相机内参时也发布压缩图，保证 Web 端能看到画面（仅暂不做 ArUco 位姿解算）
            try:
                frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except cv_bridge.CvBridgeError as exc:
                self.get_logger().warning(f"Image conversion failed: {exc}")
                return
            self.publish_camera(frame, [], msg.header)
            return

        if now - self._last_reload_check >= 1.0:
            self._last_reload_check = now
            self.marker_store.reload_if_changed()

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except cv_bridge.CvBridgeError as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return

        detected_ids = []
        # 检测前降采样：真实场景 detectMarkers 很耗 CPU，先缩小再灰度可大幅提速；
        # 检测到的角点坐标随后按比例还原到原图坐标系，定位精度不受影响。
        scale = self.detect_scale
        if scale != 1.0:
            detect_gray = cv2.cvtColor(
                cv2.resize(frame, None, fx=scale, fy=scale), cv2.COLOR_BGR2GRAY
            )
        else:
            detect_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionary = self.get_dictionary()
        if dictionary is None:
            self.publish_camera(frame, detected_ids, msg.header)
            return

        try:
            parameters = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            parameters = cv2.aruco.DetectorParameters()

        try:
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            corners, ids, _ = detector.detectMarkers(detect_gray)
        except AttributeError:
            corners, ids, _ = cv2.aruco.detectMarkers(
                detect_gray, dictionary, parameters=parameters
            )
        if scale != 1.0 and corners:
            corners = [corner / scale for corner in corners]
        if ids is None or len(ids) == 0:
            self.publish_camera(frame, detected_ids, msg.header)
            return
        detected_ids = [int(marker_id[0]) for marker_id in ids]

        marker_size = float(self.marker_store.marker_size)
        try:
            estimate_result = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size, self.camera_matrix, self.distortion
            )
            if len(estimate_result) == 2:
                rvecs, tvecs = estimate_result
            else:
                rvecs, tvecs, _ = estimate_result
        except AttributeError:
            half = marker_size / 2.0
            obj_points = np.array(
                [
                    [-half, half, 0.0],
                    [half, half, 0.0],
                    [half, -half, 0.0],
                    [-half, -half, 0.0],
                ],
                dtype=np.float32,
            )
            rvecs, tvecs = [], []
            for corner in corners:
                _, rvec, tvec = cv2.solvePnP(
                    obj_points, corner, self.camera_matrix, self.distortion
                )
                rvecs.append(rvec)
                tvecs.append(tvec)
        except (TypeError, ValueError, cv2.error):
            self.publish_camera(frame, detected_ids, msg.header)
            return

        base_to_camera = self.lookup_base_to_camera()
        if base_to_camera is None:
            self.publish_camera(frame, detected_ids, msg.header)
            return
        camera_to_base = invert_transform(base_to_camera)

        map_to_camera = None
        if self._registration_enabled():
            map_to_base = self.lookup_map_to_base()
            if map_to_base is not None:
                map_to_camera = map_to_base @ base_to_camera

        weighted_x = 0.0
        weighted_y = 0.0
        weighted_sin = 0.0
        weighted_cos = 0.0
        total_weight = 0.0

        for marker_index, marker_id_array in enumerate(ids):
            marker_id = int(marker_id_array[0])
            marker = self.marker_store.get(marker_id)
            rvec = rvecs[marker_index]
            tvec = tvecs[marker_index]
            distance = float(np.linalg.norm(tvec))
            if distance < self.min_marker_distance or distance > self.max_marker_distance:
                continue
            camera_to_marker = matrix_from_rvec_tvec(rvec, tvec)

            if marker is None:
                if map_to_camera is not None:
                    map_to_marker = map_to_camera @ camera_to_marker
                    marker_x, marker_y, _, marker_yaw = matrix_to_pose(map_to_marker)
                    self._record_marker_observation(
                        marker_id, marker_x, marker_y, marker_yaw, time.time()
                    )
                continue

            map_to_marker = matrix_from_xyz_yaw(
                marker["x"], marker["y"], 0.0, marker["yaw"]
            )
            marker_to_camera = invert_transform(camera_to_marker)
            map_to_camera = map_to_marker @ marker_to_camera
            map_to_base = map_to_camera @ camera_to_base
            x, y, _, yaw = matrix_to_pose(map_to_base)

            weight = 1.0 / (distance + 0.15)
            weighted_x += weight * x
            weighted_y += weight * y
            weighted_sin += weight * math.sin(yaw)
            weighted_cos += weight * math.cos(yaw)
            total_weight += weight

        self._maybe_register_markers()

        if total_weight <= 0.0:
            self.publish_camera(frame, detected_ids, msg.header)
            return

        x = weighted_x / total_weight
        y = weighted_y / total_weight
        yaw = math.atan2(weighted_sin, weighted_cos)

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.map_frame
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation = quaternion_from_yaw(yaw)

        pos_var = self.position_covariance * self.position_covariance
        yaw_var = self.yaw_covariance * self.yaw_covariance
        pose.pose.covariance = [
            pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pos_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, yaw_var,
        ]
        self.pose_pub.publish(pose)
        self._maybe_correct_amcl(pose)

        if self.publish_debug_image and self.debug_pub.get_subscription_count() > 0:
            try:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            except (AttributeError, cv2.error):
                pass
            for marker_index, marker_id_array in enumerate(ids):
                try:
                    cv2.aruco.drawAxis(
                        frame,
                        self.camera_matrix,
                        self.distortion,
                        rvecs[marker_index],
                        tvecs[marker_index],
                        marker_size * 0.5,
                    )
                except (AttributeError, cv2.error):
                    break
            debug_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        self.publish_camera(frame, detected_ids, msg.header)

    def _registration_enabled(self):
        """建图期间（没有 AMCL 位姿输入）才允许自动登记标志位。"""
        if not self.auto_register:
            return False
        if self.amcl_pose is None:
            return True
        _, _, _, stamp = self.amcl_pose
        return (time.time() - stamp) > self.register_amcl_timeout

    def _record_marker_observation(self, marker_id, x, y, yaw, stamp):
        self.register_observations.setdefault(marker_id, []).append(
            (x, y, yaw, stamp)
        )

    def _maybe_register_markers(self):
        """观测收敛后把新标志位的 map 系坐标写入 markers.json。"""
        if not self.auto_register:
            return
        now = time.time()
        pruned = {}
        for marker_id, observations in self.register_observations.items():
            kept = [
                (x, y, yaw, stamp)
                for (x, y, yaw, stamp) in observations
                if now - stamp <= 30.0
            ]
            if kept:
                pruned[marker_id] = kept
        self.register_observations = pruned

        for marker_id, observations in list(self.register_observations.items()):
            if marker_id in self.registered_ids:
                continue
            if len(observations) < self.register_min_observations:
                continue
            first_stamp = min(stamp for _, _, _, stamp in observations)
            last_stamp = max(stamp for _, _, _, stamp in observations)
            if last_stamp - first_stamp < self.register_min_span_sec:
                continue
            xs = np.asarray([x for x, _, _, _ in observations])
            ys = np.asarray([y for _, y, _, _ in observations])
            if xs.std() > self.register_max_stddev or ys.std() > self.register_max_stddev:
                continue
            x = float(xs.mean())
            y = float(ys.mean())
            yaw = math.atan2(
                sum(math.sin(yaw) for _, _, yaw, _ in observations),
                sum(math.cos(yaw) for _, _, yaw, _ in observations),
            )
            self.marker_store.markers[marker_id] = {"x": x, "y": y, "yaw": yaw}
            self.marker_store.save()
            self.registered_ids.add(marker_id)
            self.register_observations.pop(marker_id, None)
            self.get_logger().info(
                f"已自动登记 ArUco 标志位 id={marker_id} "
                f"(x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°, "
                f"共 {len(observations)} 次观测)"
            )

    def publish_camera(self, frame, detected_ids, header):
        status = String()
        status.data = ",".join(str(marker_id) for marker_id in detected_ids) or "none"
        self.status_pub.publish(status)

        # 有订阅者（如 web 端画面）时才编码发布压缩图，无人查看时省掉每帧 JPEG 编码
        if self.compressed_pub.get_subscription_count() > 0:
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80]
            )
            if not ok:
                return
            compressed = CompressedImage()
            compressed.header = header
            compressed.format = "jpeg"
            compressed.data = encoded.tobytes()
            self.compressed_pub.publish(compressed)

    def get_dictionary(self):
        dictionary_name = self.marker_store.dictionary_name
        try:
            dictionary_value = getattr(cv2.aruco, dictionary_name)
            return cv2.aruco.getPredefinedDictionary(dictionary_value)
        except (AttributeError, cv2.error):
            self.get_logger().error(
                f"Unknown ArUco dictionary {dictionary_name}",
                throttle_duration_sec=5.0,
            )
            return None

    def lookup_base_to_camera(self):
        if self.base_frame == self.camera_frame:
            return np.eye(4)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().debug(
                f"base -> camera TF unavailable: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

        matrix = np.eye(4)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix[0, 3] = translation.x
        matrix[1, 3] = translation.y
        matrix[2, 3] = translation.z
        matrix[:3, :3] = quaternion_to_rotation(rotation)
        return matrix

    def lookup_map_to_base(self):
        if self.map_frame == self.base_frame:
            return np.eye(4)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().debug(
                f"map -> {self.base_frame} TF unavailable: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

        matrix = np.eye(4)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix[0, 3] = translation.x
        matrix[1, 3] = translation.y
        matrix[2, 3] = translation.z
        matrix[:3, :3] = quaternion_to_rotation(rotation)
        return matrix

def quaternion_to_rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
