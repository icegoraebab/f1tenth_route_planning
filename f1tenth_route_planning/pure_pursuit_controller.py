#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import tf2_ros

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_topic', '/drive')
        self.declare_parameter('lookahead', 1.0)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('v_min', 0.2)
        self.declare_parameter('k_curv', 2.0)
        self.declare_parameter('goal_tol', 0.4)
        self.declare_parameter('use_scan_safety', True)
        self.declare_parameter('stop_dist', 0.6)
        self.declare_parameter('slow_dist', 1.0)
        self.declare_parameter('front_angle_deg', 60.0)

        self.map_frame  = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.cmd_topic  = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.Ld = self.get_parameter('lookahead').get_parameter_value().double_value
        self.L  = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.v_min = self.get_parameter('v_min').get_parameter_value().double_value
        self.k_curv = self.get_parameter('k_curv').get_parameter_value().double_value
        self.goal_tol = self.get_parameter('goal_tol').get_parameter_value().double_value
        self.use_scan = self.get_parameter('use_scan_safety').get_parameter_value().bool_value
        self.stop_dist = self.get_parameter('stop_dist').get_parameter_value().double_value
        self.slow_dist = self.get_parameter('slow_dist').get_parameter_value().double_value
        self.front_ang = math.radians(self.get_parameter('front_angle_deg').get_parameter_value().double_value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_sub = self.create_subscription(Path, '/global_path', self.on_path, 1)
        if self.use_scan:
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 5)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, self.cmd_topic, 10)

        self.path_pts: List[Tuple[float,float]] = []
        self.min_front = None

        self.timer = self.create_timer(0.02, self.step)  # 50 Hz
        self.get_logger().info('PurePursuit: /global_path 추종 대기중')

    def on_path(self, msg: Path):
        self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(f'path pts={len(self.path_pts)}')

    def on_scan(self, msg: LaserScan):
        if not msg.ranges: self.min_front=None; return
        angs = [msg.angle_min + i*msg.angle_increment for i in range(len(msg.ranges))]
        ds = [r for a,r in zip(angs,msg.ranges)
              if -self.front_ang<=a<=self.front_ang and msg.range_min<=r<=msg.range_max]
        self.min_front = min(ds) if ds else None

    def step(self):
        if not self.path_pts: return
        pose = self.lookup_pose()
        if pose is None: return
        x,y,yaw = pose

        gx,gy = self.path_pts[-1]
        if math.hypot(gx-x, gy-y) < self.goal_tol:
            self.publish(0.0, 0.0); return

        d2 = [((px-x)**2 + (py-y)**2) for (px,py) in self.path_pts]
        i_closest = int(min(range(len(d2)), key=lambda i: d2[i]))
        tgt = None
        for i in range(i_closest, len(self.path_pts)):
            px,py = self.path_pts[i]
            if math.hypot(px-x, py-y) >= self.Ld:
                tgt=(px,py); break
        if tgt is None: tgt = self.path_pts[-1]

        bx,by = self.world_to_base(tgt[0], tgt[1], x,y,yaw)
        kappa = 2.0*by / max(1e-3, self.Ld*self.Ld)
        steer = math.atan(self.L * kappa)

        v = self.v_max / (1.0 + self.k_curv*abs(kappa))
        v = max(self.v_min, min(self.v_max, v))

        if self.use_scan and self.min_front is not None:
            if self.min_front < self.stop_dist: v = 0.0
            elif self.min_front < self.slow_dist: v = min(v, 0.6*self.v_max)

        self.publish(steer, v)

    def publish(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.cmd_pub.publish(msg)

    def world_to_base(self, wx,wy, rx,ry,yaw):
        dx,dy = (wx-rx),(wy-ry)
        c,s = math.cos(-yaw), math.sin(-yaw)
        return (c*dx - s*dy, s*dx + c*dy)

    def lookup_pose(self) -> Optional[Tuple[float,float,float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            x = tf.transform.translation.x; y = tf.transform.translation.y
            q = tf.transform.rotation
            siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
            cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x,y,yaw)
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}')
            return None

def main():
    rclpy.init(); rclpy.spin(PurePursuit()); rclpy.shutdown()
