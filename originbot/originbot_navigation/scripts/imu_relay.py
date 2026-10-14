#!/usr/bin/env python3
"""IMU 中继：把原始 IMU 数据转换到 base_footprint 系，供 Cartographer 使用。

背景：Cartographer 要求 IMU 与 tracking_frame（base_footprint）完全重合
（sensor_to_tracking 平移 < 1e-5），而 OriginBot 的 IMU 实际安装在
imu_link（相对 base_link 有 x/y/z 偏移和 roll/yaw 旋转），直接订阅 /imu
会导致 cartographer_node FATAL 崩溃。

本节点将 /imu（原始 IMU 系）按 base_link->imu_link 的旋转变换到
base_footprint 系后，以 frame_id=base_footprint 重新发布到 /imu_cartographer，
Cartographer 订阅该虚拟话题即可（重合检查通过，数据方向正确）。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener


def quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(R):
    """旋转矩阵 -> 四元数 (x, y, z, w)。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
            w = (R[2, 1] - R[1, 2]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            y = 0.25 * s
            x = (R[0, 1] + R[1, 0]) / s
            z = (R[1, 2] + R[2, 1]) / s
            w = (R[0, 2] - R[2, 0]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            z = 0.25 * s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            w = (R[1, 0] - R[0, 1]) / s
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


def quaternion_multiply(q1, q2):
    """四元数乘法 q1 ⊗ q2。q = (x, y, z, w)"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class ImuRelay(Node):
    def __init__(self):
        super().__init__("imu_relay")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(Imu, "/imu", self.imu_callback, 2)
        self.pub = self.create_publisher(Imu, "/imu_cartographer", 10)
        self._rot = None
        self._rot_quat = None
        self._tf_warned = False
        self._ready_ros_time = None

    def _get_rotation(self):
        """获取 base_footprint <- imu_link 的旋转矩阵（带缓存）。"""
        if self._rot is not None:
            return self._rot, self._rot_quat
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_footprint", "imu_link", rclpy.time.Time()
            )
        except Exception:
            if not self._tf_warned:
                self.get_logger().warning(
                    "等待 base_footprint -> imu_link TF ...", throttle_duration_sec=3.0
                )
                self._tf_warned = True
            return None, None
        R = quaternion_to_matrix(tf.transform.rotation)
        self._rot = R
        self._rot_quat = matrix_to_quaternion(R)
        self._ready_ros_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            "imu_relay 已获取 TF 旋转，开始转换 IMU 数据到 base_footprint 系"
        )
        return self._rot, self._rot_quat

    def _stamp_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def imu_callback(self, msg):
        R, qR = self._get_rotation()
        if R is None:
            return
        # 丢弃就绪前积压的旧消息，避免 Cartographer 时间乱序（ordered_multi_queue）
        if self._ready_ros_time is not None:
            if self._stamp_sec(msg.header.stamp) < self._ready_ros_time - 0.5:
                return

        out = Imu()
        out.header = msg.header
        out.header.frame_id = "base_footprint"

        g = np.array([
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ])
        g2 = R @ g
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = g2

        a = np.array([
            msg.linear_acceleration.x, msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        a2 = R @ a
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = a2

        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        q2 = quaternion_multiply(qR, q)
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q2

        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
