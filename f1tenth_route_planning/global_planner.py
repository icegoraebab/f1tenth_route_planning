#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import deque
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
from transforms3d.euler import euler2quat  # 이미 gym_bridge에서 쓰는 라이브러리

# -------------------- 작은 유틸 -------------------- #
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

def inflate_occ_binary(occ: np.ndarray, k: int) -> np.ndarray:
    """8-연결 BFS로 k셀 만큼 팽창(빠르고 SciPy 불필요)"""
    if k <= 0:
        return occ.copy()
    h, w = occ.shape
    big = 10**9
    dist = np.full((h, w), big, dtype=np.int32)
    q = deque()
    ys, xs = np.where(occ > 0)
    for y, x in zip(ys, xs):
        dist[y, x] = 0
        q.append((x, y))
    nbrs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]
    while q:
        x, y = q.popleft()
        d = dist[y, x]
        if d >= k:
            continue
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and dist[ny, nx] > d + 1:
                dist[ny, nx] = d + 1
                q.append((nx, ny))
    return (dist <= k).astype(np.uint8)

def astar(grid: np.ndarray,
          start: Tuple[int,int],
          goal: Tuple[int,int],
          passable) -> Optional[List[Tuple[int,int]]]:
    """4-connected A* (grid[y,x])"""
    sx, sy = start
    gx, gy = goal
    hgt, wid = grid.shape
    if not (0 <= sx < wid and 0 <= sy < hgt and 0 <= gx < wid and 0 <= gy < hgt):
        return None

    def h(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    import heapq
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
            path = []
            p = cur
            while p is not None:
                path.append(p)
                p = came[p]
            path.reverse()
            return path
        closed.add(cur)
        cx, cy = cur
        for dx, dy in nb:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= wid or ny >= hgt:
                continue
            nxt = (nx, ny)
            if nxt in closed:
                continue
            if not passable(nx, ny):
                continue
            ng = g + 1
            if ng < gscore.get(nxt, 1_000_000_000):
                gscore[nxt] = ng
                heapq.heappush(openq, (ng + h(nxt, goal), ng, nxt, cur))
    return None
# -------------------------------------------------- #


class GlobalPlanner(Node):
    """
    - /map(OccupancyGrid) 수신(QoS: Transient Local)
    - /clicked_point(map) 수신 → 경유점 업데이트
    - map→base_link TF 조회 → A* 경로 생성
    - 실패 시 직선 보간 fallback으로라도 /global_path 발행
    """

    def __init__(self):
        super().__init__('global_planner')

        # parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('clicked_topic', '/clicked_point')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        self.declare_parameter('occupied_thresh', 50)            # >= 50: obstacle
        self.declare_parameter('unknown_is_obstacle', False)     # -1을 장애물로 취급?
        self.declare_parameter('inflate_radius_m', 0.03)         # 좁은 트랙 대응
        self.declare_parameter('min_path_points', 60)

        self.map_topic   = self.get_parameter('map_topic').value
        self.click_topic = self.get_parameter('clicked_topic').value
        self.map_frame   = self.get_parameter('map_frame').value
        self.base_frame  = self.get_parameter('base_frame').value
        self.occ_thresh  = int(self.get_parameter('occupied_thresh').value)
        self.unknown_obs = bool(self.get_parameter('unknown_is_obstacle').value)
        self.inflate_r   = float(self.get_parameter('inflate_radius_m').value)
        self.min_pts     = int(self.get_parameter('min_path_points').value)

        self.get_logger().info(
            f'[GlobalPlanner] base_frame={self.base_frame}, '
            f'inflate={self.inflate_r:.3f}m, unknown_is_obstacle={self.unknown_obs}'
        )

        # /map 은 Transient Local 로 구독해야 과거 샘플도 수신됨
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_map, map_qos
        )

        # 클릭은 기본으로 충분
        self.click_sub = self.create_subscription(
            PointStamped, self.click_topic, self.on_click, 10
        )

        self.path_pub   = self.create_publisher(Path, '/global_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 1)

        # clear 서비스 (웨이포인트/마커 초기화)
        self.clear_srv = self.create_service(Empty, 'waypoints_clear', self.on_clear)

        # TF
        self.tfbuf = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfbuf, self)

        self.map_msg = None
        self.waypoints: List[Tuple[float, float]] = []

        self.get_logger().info('RViz의 Publish Point로 웨이포인트를 찍으세요. (두 점 이상 권장)')

    # ----------------- callbacks ----------------- #
    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_click(self, msg: PointStamped):
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'clicked_point frame {msg.header.frame_id} != {self.map_frame}')
            return
        self.waypoints.append((msg.point.x, msg.point.y))
        self.publish_markers()
        self.replan()

    def on_clear(self, req, resp):
        self.waypoints.clear()
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.get_logger().info('웨이포인트 초기화')
        return resp

    # ------------------- core -------------------- #
    def replan(self):
        if self.map_msg is None:
            self.get_logger().warn('맵을 아직 못 받음(/map). 잠시 후 다시.')
            return
        if len(self.waypoints) == 0:
            self.get_logger().info('웨이포인트가 없음')
            return

        m = self.map_msg
        width, height = m.info.width, m.info.height
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y

        # occupancy (0~100, -1 unknown)
        raw = np.int16(np.array(m.data, dtype=np.int16)).reshape((height, width))
        occ = np.zeros_like(raw, dtype=np.uint8)
        if self.unknown_obs:
            mask = (raw >= self.occ_thresh) | (raw < 0)
        else:
            mask = (raw >= self.occ_thresh)
        occ[mask] = 1

        inflate_cells = max(0, int(round(self.inflate_r / res)))
        if inflate_cells > 0:
            occ = inflate_occ_binary(occ, inflate_cells)

        # start pose from TF
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF 조회 실패: {e}')
            return
        sx = tf.transform.translation.x
        sy = tf.transform.translation.y

        gx, gy = self.waypoints[-1]

        s_idx = world_to_grid(sx, sy, ox, oy, res, width, height)
        g_idx = world_to_grid(gx, gy, ox, oy, res, width, height)
        if s_idx is None or g_idx is None:
            self.get_logger().warn(f'시작/목표가 맵 밖: start={s_idx}, goal={g_idx} → 직선 페일백')
            self.publish_fallback(sx, sy, gx, gy)
            return

        def passable(x, y):
            return occ[y, x] == 0

        path_grid = astar(occ, (s_idx[0], s_idx[1]), (g_idx[0], g_idx[1]), passable)
        if path_grid is None or len(path_grid) < 2:
            self.get_logger().warn('A* 실패 → 직선 경로로 페일백')
            self.publish_fallback(sx, sy, gx, gy)
            return

        pts: List[Tuple[float, float]] = [grid_to_world(x, y, ox, oy, res) for (x, y) in path_grid]
        self.publish_path(pts)

    # ---------------- publishers ----------------- #
    def publish_fallback(self, sx, sy, gx, gy):
        n = max(self.min_pts, 2)
        xs = np.linspace(sx, gx, n)
        ys = np.linspace(sy, gy, n)
        pts = list(zip(xs.tolist(), ys.tolist()))
        self.publish_path(pts, note='[fallback]')

    def publish_path(self, pts: List[Tuple[float, float]], note: str = ''):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        prev = None
        for (x, y) in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            if prev is None:
                yaw = 0.0
            else:
                dx, dy = x - prev[0], y - prev[1]
                yaw = math.atan2(dy, dx)
            qw, qx, qy, qz = euler2quat(0.0, 0.0, yaw, axes='sxyz')
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
            prev = (x, y)

        self.path_pub.publish(path)
        self.get_logger().info(f'{note} /global_path 발행: {len(path.poses)} pts')

    def publish_markers(self):
        ma = MarkerArray()
        for i, (x, y) in enumerate(self.waypoints):
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
