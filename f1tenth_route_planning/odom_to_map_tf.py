#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToMapTF(Node):
    def __init__(self):
        super().__init__('odom_to_map_tf')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')

        self.tf_pub = TransformBroadcaster(self)
        odom_topic = self.get_parameter('odom_topic').value
        self.sub = self.create_subscription(Odometry, odom_topic, self.cb, 10)
        self.get_logger().info(
            f"Bridging {odom_topic} → TF "
            f"{self.get_parameter('map_frame').value} → {self.get_parameter('base_frame').value}"
        )

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp     # odom 타임스탬프 사용
        t.header.frame_id = self.get_parameter('map_frame').value
        t.child_frame_id  = self.get_parameter('base_frame').value

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q

        self.tf_pub.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(OdomToMapTF())
    rclpy.shutdown()
