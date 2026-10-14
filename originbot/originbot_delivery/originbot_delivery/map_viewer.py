#!/usr/bin/env python3

import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class MapViewer(Node):
    def __init__(self):
        super().__init__("map_viewer")
        self.declare_parameter("map_file", "")
        map_file = self.get_parameter("map_file").value
        if not map_file or not os.path.exists(map_file):
            raise RuntimeError(f"地图文件不存在: {map_file}")

        self.publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.map_msg = self.load_map(map_file)
        self.create_timer(2.0, self.publish_map)
        self.publish_map()
        self.get_logger().info(
            f"已打开地图 {os.path.basename(map_file)} "
            f"({self.map_msg.info.width}x{self.map_msg.info.height})"
        )

    def load_map(self, map_file):
        with open(map_file, "r", encoding="utf-8") as handle:
            meta = yaml.safe_load(handle)

        image_path = os.path.join(
            os.path.dirname(os.path.abspath(map_file)), meta["image"]
        )
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"地图图片加载失败: {image_path}")

        resolution = float(meta["resolution"])
        origin = meta["origin"]
        negate = int(meta.get("negate", 0))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.25))

        gray = image.astype(np.float64)
        if negate:
            gray = 255.0 - gray
        occupancy = (255.0 - gray) / 255.0
        occ = np.full(image.shape, -1, dtype=np.int8)
        occ[occupancy > occupied_thresh] = 100
        occ[occupancy < free_thresh] = 0

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.info.resolution = resolution
        msg.info.width = int(image.shape[1])
        msg.info.height = int(image.shape[0])
        msg.info.origin = Pose(
            position=Point(x=float(origin[0]), y=float(origin[1]), z=0.0),
            orientation=quaternion_from_yaw(float(origin[2])),
        )
        # cv2.imread 读入的 image[0] 是图像顶行(地图 y 最大)，而 ROS OccupancyGrid
        # 约定 data[0] 对应 origin(左下角, y 最小)。这里上下翻转，使 data 布局与
        # map_server 加载的结果一致，否则"打开地图"与"导航"时 web 端地图会上下颠倒。
        msg.data = [int(value) for value in occ[::-1].flatten()]
        return msg

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MapViewer()
    except RuntimeError as exc:
        print(f"map_viewer 启动失败: {exc}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
