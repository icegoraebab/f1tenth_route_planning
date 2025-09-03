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
from transforms3d.euler import euler2quat  # Foxy OK

# ---------- 유틸 ---------- #
def world_to_grid(x, y, ox, oy, res, w, h):
    gx = int((x - ox) / res); gy = int((y - oy) / res)
    if gx < 0 or gy < 0 or gx >= w or gy >= h: return None
    return gx, gy

def grid_to_world(gx, gy, ox, oy, res):
    return ox + (gx + 0.5) * res, oy + (gy + 0.5) * res

def inflate_occ_binary(occ: np.ndarray, k: int) -> np.ndarray:
    """8-연결로 k셀 팽창 (SciPy 없이)"""
    if k <= 0: return occ.copy()
    h, w = occ.shape
    BIG = 10**9
    dist = np.full((h, w), BIG, dtype=np.int32)
    q = deque()
    ys, xs = np.where(occ > 0)
    for y, x in zip(ys, xs):
        dist[y, x] = 0
        q.append((x, y))
    nbrs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]
    while q:
        x, y = q.popleft()
        d = dist[y, x]
        if d >= k: continue
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and dist[ny, nx] > d + 1:
                dist[ny, nx] = d + 1
                q.append((nx, ny))
    return (dist <= k).astype(np.uint8)

def astar(grid: np.ndarray, start: Tuple[int,int], goal: Tuple[int,int], passable) -> Optional[List[Tuple[int,int]]]:
    """4-connected A* (grid[y,x])"""
    sx, sy = start; gx, gy = goal
    hgt, wid = grid.shape
    if not (0 <= sx < wid and 0 <= sy < hgt and 0 <= gx < wid and 0 <= gy < hgt):
        return None
    def h(a,b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
    import heapq
    openq = []; heapq.heappush(openq, (h(start, goal), 0, start, None))
    came = {}; gscore = {start: 0}; closed = set()
    nb = [(1,0),(-1,0),(0,1),(0,-1)]
    while openq:
        _, g, cur, parent = heapq.heappop(openq)
        if cur in closed: continue
        came[cur] = parent
        if cur == goal:
            path = []
            p = cur
            while p is not None:
                path.append(p); p = came[p]
            return list(reversed(path))
        closed.add(cur)
        cx, cy = cur
        for dx, dy in nb:
            nx, ny = cx + dx, cy + dy
            if nx<0 or ny<0 or nx>=wid or ny>=hgt: continue
            nxt = (nx, ny)
            if nxt in closed: continue
            if not passable(nx, ny): continue
            ng = g + 1
            if ng < gscore.get(nxt, 1_000_000_000):
                gscore[nxt] = ng
                heapq.heappush(openq, (ng + h(nxt, goal), ng, nxt, cur))
    return None
# ------------------------- #


class GlobalPlanner(Node):
    """
    - /map(OccupancyGrid) 구독(Transient Local)
    - /clicked_point(map) 웨이포인트 입력
    - 모드
      * use_all_waypoints=true  → 현재포즈→wp1→...→wpN 한 번에 경로
      * hop_by_hop=true         → wp1 도착하면 자동으로 wp2로 넘어가며 재계획
    - A* 실패 시 직선 보간 페일백
    - /global_path 발행 + /waypoints_markers 시각화 + clear/pop 서비스
    """

    def __init__(self):
        super().__init__('global_planner')

        # ---- 파라미터 ----
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('clicked_topic', '/clicked_point')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        self.declare_parameter('occupied_thresh', 50)
        self.declare_parameter('unknown_is_obstacle', False)
        self.declare_parameter('inflate_radius_m', 0.03)
        self.declare_parameter('min_path_points', 60)
        self.declare_parameter('use_all_waypoints', True)   # 기본 ON으로 변경
        self.declare_parameter('hop_by_hop', True)          # ★ 추가: 기본 ON
        self.declare_parameter('wp_reach_tol', 0.35)        # ★ 추가: 도착 판정(m)

        # 캐시
        self.map_topic   = self.get_parameter('map_topic').value
        self.click_topic = self.get_parameter('clicked_topic').value
        self.map_frame   = self.get_parameter('map_frame').value
        self.base_frame  = self.get_parameter('base_frame').value
        self.occ_thresh  = int(self.get_parameter('occupied_thresh').value)
        self.unknown_obs = bool(self.get_parameter('unknown_is_obstacle').value)
        self.inflate_r   = float(self.get_parameter('inflate_radius_m').value)
        self.min_pts     = int(self.get_parameter('min_path_points').value)

        self.get_logger().info(
            f'[GlobalPlanner] base_frame={self.base_frame}, inflate={self.inflate_r:.3f}m, '
            f'unknown_is_obstacle={self.unknown_obs}'
        )

        # /map (래치) 구독
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)

        # 클릭 포인트 구독
        self.click_sub = self.create_subscription(PointStamped, self.click_topic, self.on_click, 10)

        # 퍼블리셔
        self.path_pub   = self.create_publisher(Path, '/global_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 1)

        # 서비스
        self.clear_srv = self.create_service(Empty, 'waypoints_clear', self.on_clear)
        self.pop_srv   = self.create_service(Empty, 'waypoints_pop',   self.on_pop)

        # TF
        self.tfbuf = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfbuf, self)

        # 상태
        self.map_msg = None
        self.waypoints: List[Tuple[float,float]] = []
        self.active_idx: Optional[int] = None   # ★ hop-by-hop 현재 타깃 인덱스

        # 주기 체크 (도착 판정 & 자동 진행)
        self.timer = self.create_timer(0.10, self._tick)  # 10Hz

        self.get_logger().info('RViz의 Publish Point로 웨이포인트를 차례로 찍으세요.')

    # -------- callbacks -------- #
    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_click(self, msg: PointStamped):
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(f'clicked_point frame {msg.header.frame_id} != {self.map_frame}')
            return
        self.waypoints.append((msg.point.x, msg.point.y))

        # hop-by-hop 인데 아직 타깃이 없으면 0번부터 시작
        if bool(self.get_parameter('hop_by_hop').value) and self.active_idx is None:
            self.active_idx = 0

        self.publish_markers()
        self.replan()

    def on_clear(self, req, resp):
        self.waypoints.clear()
        self.active_idx = None
        ma = MarkerArray(); m = Marker(); m.action = Marker.DELETEALL; ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.path_pub.publish(Path())  # 시각 초기화
        self.get_logger().info('웨이포인트 초기화')
        return resp

    def on_pop(self, req, resp):
        if self.waypoints:
            self.waypoints.pop()
            # active_idx 조정
            if self.active_idx is not None and self.active_idx >= len(self.waypoints):
                self.active_idx = None if not self.waypoints else len(self.waypoints)-1
            self.publish_markers()
            self.replan()
            self.get_logger().info('마지막 웨이포인트 제거')
        else:
            self.get_logger().info('제거할 웨이포인트가 없음')
        return resp

    # -------- periodic -------- #
    def _tick(self):
        """hop-by-hop 모드에서 도착 판정 → 다음 웨이포인트로 진행"""
        if not bool(self.get_parameter('hop_by_hop').value):
            return
        if self.map_msg is None or self.active_idx is None:
            return
        if self.active_idx < 0 or self.active_idx >= len(self.waypoints):
            return

        # 현재 포즈
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y

        gx, gy = self.waypoints[self.active_idx]
        tol = float(self.get_parameter('wp_reach_tol').value)
        if (rx - gx)**2 + (ry - gy)**2 <= tol*tol:
            self.get_logger().info(f'웨이포인트 #{self.active_idx} 도착 (tol={tol:.2f}m)')
            self.active_idx += 1
            if self.active_idx >= len(self.waypoints):
                self.get_logger().info('모든 웨이포인트 완료')
                self.active_idx = None
            self.publish_markers()
            self.replan()

    # -------- core -------- #
    def replan(self):
        if self.map_msg is None:
            self.get_logger().warn('맵을 아직 못 받음(/map).')
            return
        if len(self.waypoints) == 0:
            self.get_logger().info('웨이포인트가 없음')
            return

        m = self.map_msg
        W, H = m.info.width, m.info.height
        res  = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y

        raw = np.int16(np.array(m.data, dtype=np.int16)).reshape((H, W))
        occ = np.zeros_like(raw, dtype=np.uint8)
        mask = (raw >= self.occ_thresh) if not self.unknown_obs else ((raw >= self.occ_thresh) | (raw < 0))
        occ[mask] = 1

        inflate_cells = max(0, int(round(self.inflate_r / res)))
        if inflate_cells > 0:
            occ = inflate_occ_binary(occ, inflate_cells)

        # 현재 로봇 포즈 (map->base_frame)
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF 조회 실패: {e}')
            return
        sx = tf.transform.translation.x
        sy = tf.transform.translation.y

        # 모드 결정
        hop = bool(self.get_parameter('hop_by_hop').value)
        use_all = bool(self.get_parameter('use_all_waypoints').value)

        if hop and self.active_idx is not None:
            targets = [ self.waypoints[self.active_idx] ]
        else:
            targets = self.waypoints if use_all else [self.waypoints[-1]]

        prevx, prevy = sx, sy
        all_pts: List[Tuple[float,float]] = []

        for (gx, gy) in targets:
            s_idx = world_to_grid(prevx, prevy, ox, oy, res, W, H)
            g_idx = world_to_grid(gx, gy,    ox, oy, res, W, H)

            if s_idx is None or g_idx is None:
                self.get_logger().warn(f'세그먼트 맵 밖: start={s_idx}, goal={g_idx} → 직선 페일백')
                seg = self._segment_fallback(prevx, prevy, gx, gy)
            else:
                def passable(x, y): return occ[y, x] == 0
                pg = astar(occ, (s_idx[0], s_idx[1]), (g_idx[0], g_idx[1]), passable)
                if pg is None or len(pg) < 2:
                    self.get_logger().warn('A* 세그먼트 실패 → 직선 페일백')
                    seg = self._segment_fallback(prevx, prevy, gx, gy)
                else:
                    seg = [grid_to_world(x, y, ox, oy, res) for (x, y) in pg]

            if all_pts and seg:
                seg = seg[1:]  # 앞 세그먼트 마지막과 중복 제거
            all_pts.extend(seg)
            prevx, prevy = gx, gy

        if not all_pts:
            self.get_logger().warn('생성된 경로가 비어있음')
            return

        self.publish_path(all_pts, mode='HOP' if (hop and self.active_idx is not None) else ('ALL' if use_all else 'LAST'))

    # ------ helpers/pubs ------ #
    def _segment_fallback(self, sx, sy, gx, gy, n: int = None):
        if n is None: n = max(self.min_pts, 2)
        xs = np.linspace(sx, gx, n); ys = np.linspace(sy, gy, n)
        return list(zip(xs.tolist(), ys.tolist()))

    def publish_path(self, pts: List[Tuple[float,float]], mode: str):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        prev = None
        for (x, y) in pts:
            ps = PoseStamped(); ps.header = path.header
            ps.pose.position.x = x; ps.pose.position.y = y
            if prev is None: yaw = 0.0
            else:
                dx, dy = x - prev[0], y - prev[1]
                yaw = math.atan2(dy, dx)
            qw, qx, qy, qz = euler2quat(0.0, 0.0, yaw, axes='sxyz')
            ps.pose.orientation.x = qx; ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz; ps.pose.orientation.w = qw
            path.poses.append(ps); prev = (x, y)

        self.path_pub.publish(path)
        self.get_logger().info(f'/global_path 발행: {len(path.poses)} pts (mode={mode})')

    def publish_markers(self):
        ma = MarkerArray()
        hop = bool(self.get_parameter('hop_by_hop').value)
        now = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = 'wps'; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.18

            # 색상: 지난 점=녹색, 현재 타깃=노란색, 대기=주황색
            if hop and self.active_idx is not None:
                if i < self.active_idx:   # passed
                    m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.8, 0.2, 1.0
                elif i == self.active_idx:  # active
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.2, 1.0
                else:
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.2, 1.0
            else:
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
