#!/usr/bin/env python3
# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0 (the "License");

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def quaternion_conjugate(q):
    return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)


def quaternion_multiply(a, b):
    return Quaternion(
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )


def quaternion_normalize(q):
    norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
    if norm < 1e-12:
        return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
    return Quaternion(w=q.w / norm, x=q.x / norm, y=q.y / norm, z=q.z / norm)


def rotate_vector(q, v):
    # Rotate v by unit quaternion q.
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    cx = qy * v.z - qz * v.y
    cy = qz * v.x - qx * v.z
    cz = qx * v.y - qy * v.x
    dx = qy * cz - qz * cy
    dy = qz * cx - qx * cz
    dz = qx * cy - qy * cx
    return Vector3(
        x=v.x + 2.0 * (qw * cx + dx),
        y=v.y + 2.0 * (qw * cy + dy),
        z=v.z + 2.0 * (qw * cz + dz),
    )


class MapToOdom(Node):
    def __init__(self):
        super().__init__("map_to_odom")
        self.declare_parameter("map_pose_topic", "/odometry/filtered")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_rate", 30.0)

        self.map_pose_topic = self.get_parameter("map_pose_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.latest_map_pose = None
        self.subscription = self.create_subscription(
            Odometry, self.map_pose_topic, self.map_pose_callback, 10)

        rate = self.get_parameter("publish_rate").value
        self.timer = self.create_timer(1.0 / rate, self.publish_map_to_odom)

    def map_pose_callback(self, msg):
        if msg.header.frame_id != "map":
            self.get_logger().warning(
                f"Ignoring pose with frame_id {msg.header.frame_id}, expected map")
            return
        self.latest_map_pose = msg

    def publish_map_to_odom(self):
        if self.latest_map_pose is None:
            return

        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time())
        except Exception as exc:
            self.get_logger().warning(
                f"odom -> {self.base_frame} lookup failed: {exc}")
            return

        map_to_base = self.latest_map_pose.pose.pose

        # Invert odom_T_base to get base_T_odom.
        q_base_odom = quaternion_conjugate(odom_to_base.transform.rotation)
        t_base_odom = rotate_vector(q_base_odom, odom_to_base.transform.translation)
        t_base_odom = Vector3(
            x=-t_base_odom.x, y=-t_base_odom.y, z=-t_base_odom.z)

        # Compose map_T_odom = map_T_base * base_T_odom.
        q_map_odom = quaternion_multiply(map_to_base.orientation, q_base_odom)
        q_map_odom = quaternion_normalize(q_map_odom)
        t_map_odom = rotate_vector(map_to_base.orientation, t_base_odom)
        t_map_odom = Vector3(
            x=map_to_base.position.x + t_map_odom.x,
            y=map_to_base.position.y + t_map_odom.y,
            z=map_to_base.position.z + t_map_odom.z,
        )

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = self.odom_frame
        transform.transform.translation = t_map_odom
        transform.transform.rotation = q_map_odom
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = MapToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
