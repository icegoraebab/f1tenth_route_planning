#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, heapq
from typing import List, Tuple, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from std_srvs.srv import Empty

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    return q

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('clicked_topic', '/clicked_point')
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('inflate_radius_m', 0.25)
        self.declare_parameter('unknown_is_obstacle', True)

        self.map_frame  = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.clicked_topic = self.get_parameter('clicked_topic').get_parameter_value().string_value
        self.allow_diag = self.get_parameter('allow_diagonal').get_parameter_value().bool_value
        self.inflate_r_m = self.get_parameter('inflate_radius_m').get_parameter_value().double_value
        self.unknown_block = self.get_parameter('unknown_is_obstacle').get_parameter_value().bool_value

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.click_sub = self.create_subscription(PointStamped, self.clicked_topic, self.on_click, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 1)
        self.mark_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 1)
        self.clear_srv = self.create_service(Empty, '/waypoints_clear', self.on_clear)

        self.map: Optional[OccupancyGrid] = None
        self.grid: Optional[np.ndarray] = None
        self.w = self.h = None
        self.resolution = None
        self.origin_xy = None
        self.wps: List[Tuple[float,float]] = []
        self.full_path: List[Tuple[float,float]] = []

        self.get_logger().info('GlobalPlanner: RViz Publish Point로 웨이포인트를 찍어주세요.')

    def on_map(self, msg: OccupancyGrid):
        self.map = msg
        self.w, self.h = msg.info.width, msg.info.height
        self.resolution = msg.info.resolution
        self.origin_xy = (msg.info.origin.position.x, msg.info.origin.position.y)

        data = np.array(msg.data, dtype=np.int16).reshape((self.h,self.w))
        if self.unknown_block:
            data[data < 0] = 100
        occ = (data >= 50).astype(np.uint8)

        r = max(1, int(round(self.inflate_r_m / self.resolution)))
        if r>0:
            occ = self.inflate(occ, r)
        self.grid = (occ>0).astype(np.uint8)

    def inflate(self, occ: np.ndarray, r: int) -> np.ndarray:
        H,W = occ.shape
        inflated = occ.copy()
        for dy in range(-r, r+1):
            for dx in range(-r, r+1):
                if dx*dx + dy*dy > r*r: continue
                sy1,sy2 = max(0,dy), min(H,H+dy)
                sx1,sx2 = max(0,dx), min(W,W+dx)
                tmp = np.zeros_like(occ)
                tmp[sy1:sy2, sx1:sx2] = occ[sy1-dy:sy2-dy, sx1-dx:sx2-dx]
                inflated = np.maximum(inflated, tmp)
        return inflated

    def on_clear(self, req, res):
        self.wps.clear(); self.full_path.clear()
        self.publish_markers(); self.publish_path([])
        self.get_logger().info('Waypoints cleared')
        return res

    def on_click(self, msg: PointStamped):
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn('clicked point frame이 map이 아님 → 무시')
            return
        self.wps.append((msg.point.x, msg.point.y))
        self.publish_markers()
        self.replan()

    def replan(self):
        if self.grid is None or self.map is None or not self.wps: return
        pose = self.lookup_pose()
        if pose is None: return
        sx, sy, _ = pose
        pts: List[Tuple[float,float]] = []
        cur = (sx, sy)
        ok = True
        for goal in self.wps:
            seg = self.astar(cur, goal)
            if not seg:
                self.get_logger().warn('A* 실패(세그먼트)'); ok = False; break
            pts.extend(seg if not pts else seg[1:])
            cur = goal
        if ok:
            pts = self.smooth(pts, 5)
            self.full_path = pts
            self.publish_path(pts)
            self.get_logger().info(f'Global path pts={len(pts)}')

    def smooth(self, pts, k=5):
        if len(pts)<k: return pts
        xs = np.array([p[0] for p in pts]); ys = np.array([p[1] for p in pts])
        ker = np.ones(k)/k
        xs_s = np.convolve(xs, ker, 'same'); ys_s = np.convolve(ys, ker, 'same')
        xs_s[:k//2]=xs[:k//2]; xs_s[-k//2:]=xs[-k//2:]
        ys_s[:k//2]=ys[:k//2]; ys_s[-k//2:]=ys[-k//2:]
        return list(zip(xs_s.tolist(), ys_s.tolist()))

    def world_to_cell(self, x,y):
        ox,oy = self.origin_xy
        cx = int((x-ox)/self.resolution); cy = int((y-oy)/self.resolution)
        return cy,cx
    def cell_to_world(self, cy,cx):
        ox,oy = self.origin_xy
        wx = ox + (cx+0.5)*self.resolution; wy = oy + (cy+0.5)*self.resolution
        return wx,wy

    def neigh(self, cy,cx):
        steps = [(-1,0),(1,0),(0,-1),(0,1)]
        if self.allow_diag: steps += [(-1,-1),(-1,1),(1,-1),(1,1)]
        for dy,dx in steps:
            ny,nx = cy+dy, cx+dx
            if 0<=ny<self.h and 0<=nx<self.w: yield ny,nx

    def h(self,a,b): return math.hypot(a[0]-b[0], a[1]-b[1])

    def astar(self, start_w, goal_w):
        sy,sx = self.world_to_cell(*start_w); gy,gx = self.world_to_cell(*goal_w)
        if self.grid[sy,sx] or self.grid[gy,gx]: return None
        openq=[]; heapq.heappush(openq,(0.0,(sy,sx)))
        g={ (sy,sx):0.0 }; parent={ (sy,sx):None }
        while openq:
            _,cur = heapq.heappop(openq)
            if cur==(gy,gx):
                cells=[]; c=cur
                while c is not None: cells.append(c); c=parent[c]
                cells.reverse()
                return [self.cell_to_world(cy,cx) for cy,cx in cells]
            for ny,nx in self.neigh(*cur):
                if self.grid[ny,nx]: continue
                step = math.hypot(ny-cur[0], nx-cur[1])
                tentative = g[cur]+step
                if (ny,nx) not in g or tentative<g[(ny,nx)]:
                    g[(ny,nx)]=tentative
                    f = tentative + self.h((ny,nx),(gy,gx))
                    parent[(ny,nx)] = cur
                    heapq.heappush(openq,(f,(ny,nx)))
        return None

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

    def publish_path(self, pts):
        path = Path(); path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x,y in pts:
            ps = PoseStamped(); ps.header.frame_id = self.map_frame
            ps.pose.position.x = x; ps.pose.position.y = y
            ps.pose.orientation = yaw_to_quat(0.0)
            path.poses.append(ps)
        self.path_pub.publish(path)

    def publish_markers(self):
        ma = MarkerArray(); now = self.get_clock().now().to_msg()
        for i,(x,y) in enumerate(self.wps):
            m = Marker(); m.header.frame_id=self.map_frame; m.header.stamp=now
            m.ns='wps'; m.id=i; m.type=Marker.SPHERE; m.action=Marker.ADD
            m.pose.position.x=x; m.pose.position.y=y
            m.scale.x=m.scale.y=m.scale.z=0.25
            m.color.r,m.color.g,m.color.b,m.color.a = (0.9,0.3,0.1,0.9)
            ma.markers.append(m)
        self.mark_pub.publish(ma)

def main():
    rclpy.init()
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
