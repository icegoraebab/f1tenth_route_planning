#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import heapq
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
from tf_transformations import quaternion_from_euler

def world_to_grid(x, y, origin_x, origin_y, res, width, height):
    gx = int((x - origin_x) / res)
    gy = int((y - origin_y) / res)
    if gx < 0 or gy < 0 or gx >= width or gy >= height:
        return None
    return gx, gy

def grid_to_world(gx, gy, origin_x, origin_y, res):
    wx = origin_x + (gx + 0.5) * res
    wy = origin_y + (gy + 0.5) * res
    return wx, wy

def astar(grid: np.ndarray,
          start: Tuple[int,int],
          goal: Tuple[int,int],
          passable) -> Optional[List[Tuple[int,int]]]:
    """4-connected A* (grid[y,x])"""
    width = grid.shape[1]
    height = grid.shape[0]
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < width and 0 <= sy < height and 0 <= gx < width and 0 <= gy < height):
        return None

    def h(a,b):  # manhattan
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    openq = []
    heapq.heappush(openq, (h(start, goal), 0, start, None))
    came = {}
    gscore = {start: 0}
    closed = set()

    nb = [(1,0),(-1,0),(0,1),(0,-1)]
    while openq:
        _, g, cur, parent = heapq.heappop(openq)
        if cur in closed: 
            continue
        came[cur] = parent
        if cur == goal:
            # reconstruct
            path = []
            p = cur
            while p is not None:
                path.append(p)
                p = came[p]
            path.reverse()
            return path
        closed.add(cur)
        cx, cy = cur
        for dx,dy in nb:
            nx, ny = cx+dx, cy+dy
            if nx<0 or ny<0 or nx>=width or ny>=height: 
                continue
            nxt = (nx, ny)
            if nxt in closed: 
                continue
            if not passable(nx, ny): 
                continue
            ng = g + 1
            if ng < gscore.get(nxt, 1e18):
                gscore[nxt] = ng
                heapq.heappush(openq, (ng + h(nxt, goal), ng, nxt, cur))
    return None

class GlobalPlanner(Node):
    """
    - /map (OccupancyGrid) 수신
    - /clicked_point (PointStamped, map)로 웨이포인트 입력
    - map→base_link TF로 현재 포즈 취득
    - A*로 /global_path 발행 (실패시 직선 보간 fallback)
    """
    def __init__(self):
        super().__init__('global_planner')

        # params
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('clicked_topic', '/clicked_point')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        self.declare_parameter('occupied_thresh', 50)           # >= 50 : 장애물
        self.declare_parameter('unknown_is_obstacle', False)    # True면 -1도 장애물
        self.declare_parameter('inflate_radius_m', 0.03)        # 좁은 트랙 고려
        self.declare_parameter('min_path_points', 50)           # path 해상도

        self.map_topic     = self.get_parameter('map_topic').value
        self.clicked_topic = self.get_parameter('clicked_topic').value
        self.map_frame     = self.get_parameter('map_frame').value
        self.base_frame    = self.get_parameter('base_frame').value
        self.occ_thresh    = int(self.get_parameter('occupied_thresh').value)
        self.unknown_obs   = bool(self.get_parameter('unknown_is_obstacle').value)
        self.inflate_r     = float(self.get_parameter('inflate_radius_m').value)
        self.min_pts       = int(self.get_parameter('min_path_points').value)

        self.get_logger().info(f"GlobalPlanner: base_frame={self.base_frame}, inflate={self.inflate_r}, unknown_is_obstacle={self.unknown_obs}")

        qos = QoSProfile(depth=1)
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, qos)
        self.click_sub = self.create_subscription(PointStamped, self.clicked_topic, self.on_click, 10)

        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 1)

        self.tfbuf = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfbuf, self)

        self.map_msg = None
        self.waypoints: List[Tuple[float,float]] = []

        # 안내
        self.get_logger().info("RViz Publish Point로 웨이포인트를 찍어주세요. (두 점 이상)")

    # ---------------- callbacks ----------------
    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_click(self, msg: PointStamped):
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(f'clicked frame {msg.header.frame_id} != {self.map_frame}')
            return
        self.waypoints.append( (msg.point.x, msg.point.y) )
        self.publish_markers()
        if len(self.waypoints) >= 1:
            self.replan()

    # --------------- core ----------------------
    def replan(self):
        # 0) 맵 체크
        if self.map_msg is None:
            self.get_logger().warn('맵을 아직 못 받음(/map). 잠시 후 다시.')
            return
        m = self.map_msg
        width  = m.info.width
        height = m.info.height
        res    = m.info.resolution
        ox     = m.info.origin.position.x
        oy     = m.info.origin.position.y

        data = np.int16(np.array(m.data, dtype=np.int16)).reshape((height, width))
        # inflate
        inflate_cells = max(1, int(self.inflate_r / res))
        occ = np.zeros_like(data, dtype=np.uint8)
        if self.unknown_obs:
            occ_mask = (data >= self.occ_thresh) | (data < 0)
        else:
            occ_mask = (data >= self.occ_thresh)
        occ[occ_mask] = 1
        if inflate_cells > 0:
            from scipy.ndimage import maximum_filter
            try:
                occ = maximum_filter(occ, size=inflate_cells*2+1)
            except Exception:
                # scipy가 없다면 간단한 확장
                k = inflate_cells
                occ = np.pad(occ, ((k,k),(k,k)), mode='edge')
                for y in range(k, k+height):
                    for x in range(k, k+width):
                        window = occ[y-k:y+k+1, x-k:x+k+1]
                        occ[y,x] = 1 if np.any(window) else 0
                occ = occ[k:k+height, k:k+width]

        # 1) 시작 포즈(로봇 현재 위치)
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF 조회 실패: {e}')
            return
        sx = tf.transform.translation.x
        sy = tf.transform.translation.y

        # 2) 경유점(마지막 점을 목표로)
        if len(self.waypoints) == 0:
            self.get_logger().info('웨이포인트가 없음')
            return
        gx, gy = self.waypoints[-1]

        # 3) grid index로 변환
        s_idx = world_to_grid(sx, sy, ox, oy, res, width, height)
        g_idx = world_to_grid(gx, gy, ox, oy, res, width, height)
        if s_idx is None or g_idx is None:
            self.get_logger().warn(f'시작/목표가 맵 밖: start={s_idx}, goal={g_idx}')
            # 페일백: 직선 경로 생성
            self.publish_fallback(sx, sy, gx, gy)
            return

        def passable(x, y):
            return occ[y, x] == 0

        # 4) A*
        path_grid = astar(occ, (s_idx[0], s_idx[1]), (g_idx[0], g_idx[1]), passable)
        if path_grid is None or len(path_grid) < 2:
            self.get_logger().warn('A* 실패 → 직선 경로로 페일백')
            self.publish_fallback(sx, sy, gx, gy)
            return

        # 5) grid -> world
        pts: List[Tuple[float,float]] = [ grid_to_world(x, y, ox, oy, res) for (x,y) in path_grid ]

        # 6) Path 발행
        self.publish_path(pts)

    def publish_fallback(self, sx, sy, gx, gy):
        # 직선 보간 (최소 min_pts개)
        n = max(self.min_pts, 2)
        xs = np.linspace(sx, gx, n)
        ys = np.linspace(sy, gy, n)
        pts = list(zip(xs.tolist(), ys.tolist()))
        self.publish_path(pts, note='[fallback]')

    def publish_path(self, pts: List[Tuple[float,float]], note: str=''):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        yaw_prev = None
        for (x,y) in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            if yaw_prev is None:
                yaw = 0.0
            else:
                dx = x - yaw_prev[0]; dy = y - yaw_prev[1]
                yaw = math.atan2(dy, dx)
            q = quaternion_from_euler(0,0,yaw)
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q[0], q[1], q[2], q[3]
            yaw_prev = (x,y)
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.get_logger().info(f'{note} /global_path 발행: {len(path.poses)} pts')

    def publish_markers(self):
        ma = MarkerArray()
        for i,(x,y) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'wps'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.2, 1.0
            ma.markers.append(m)
        self.marker_pub.publish(ma)

def main():
    rclpy.init()
    node = GlobalPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()