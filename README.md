F1TENTH Route Planning (ROS 2 Foxy)

프레임 체인: map → ego_racecar/base_link
구성: 글로벌 경로계획(A*) + 로컬 추종(Pure Pursuit)
옵션: 멀티 웨이포인트(찍은 모든 점을 순서대로 잇기)



# 0) 빌드 & 오버레이
```
cd ~/sim_ws
colcon build --symlink-install --merge-install --packages-select f1tenth_route_planning
source /opt/ros/foxy/setup.bash
source ~/sim_ws/install/setup.bash
```
# 1) 터미널 1 — 시뮬
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
# 2A) 터미널 2 — 기본형(마지막 점 1개 목표)
```
ros2 launch f1tenth_route_planning path_plan.launch.py \
  global_planner.base_frame:=ego_racecar/base_link \
  global_planner.inflate_radius_m:=0.03 \
  global_planner.unknown_is_obstacle:=false \
  pure_pursuit.base_frame:=ego_racecar/base_link \
  pure_pursuit.use_scan_safety:=false \
  pure_pursuit.lookahead:=0.8
```
# 2B) 터미널 2 — 멀티 웨이포인트 ON(모든 점 잇기)
```
ros2 launch f1tenth_route_planning path_plan.launch.py \
  global_planner.base_frame:=ego_racecar/base_link \
  global_planner.use_all_waypoints:=true \
  global_planner.inflate_radius_m:=0.03 \
  global_planner.unknown_is_obstacle:=false \
  pure_pursuit.base_frame:=ego_racecar/base_link \
  pure_pursuit.use_scan_safety:=false \
  pure_pursuit.lookahead:=0.8
```

# RViz

Fixed Frame=map

Displays: Path(Topic=/global_path), MarkerArray(Topic=/waypoints_markers)

Toolbar Publish Point → Frame=map, Topic=/clicked_point

2A) 기본형: 점 2개 (차 앞 0.7~1.0 m 1점 + 그보다 앞 1점)

2B) 멀티: 원하는 만큼 점을 순서대로 클릭

# 개요

글로벌 경로계획: /map(OccupancyGrid) 기반 A* 경로 탐색

A* 실패 시 직선 보간(fallback) 으로라도 /global_path 발행

/map은 QoS: Transient Local로 구독 → 래치된 맵을 바로 수신

로컬 추종(Pure Pursuit): /global_path를 따라 주행

웨이포인트 관리: 클릭/초기화/Undo, 멀티 웨이포인트 모드 지원

# 요구 사항

Ubuntu 20.04 / ROS 2 Foxy

f1tenth_gym_ros (시뮬): gym_bridge_launch.py

Python 라이브러리: transforms3d (이미 사용 중)

메시지/패키지: ackermann_msgs, nav_msgs, sensor_msgs, visualization_msgs

구버전 코드에서 tf_transformations가 필요할 때만:
sudo apt install ros-foxy-tf-transformations


RViz 사용법

Fixed Frame: map

표시 추가

Path: /global_path

MarkerArray: /waypoints_markers (점 + 라인 스트립 표시)

Publish Point 도구

Frame=map, Topic=/clicked_point

기본형: 점 2개

멀티: 점 여러 개(S/ㄴ자처럼 구부려 테스트 권장)

# 자주 쓰는 명령어
1) 웨이포인트 전체 초기화
```
ros2 service call /waypoints_clear std_srvs/srv/Empty {}
```
2) 마지막 웨이포인트만 취소(Undo)
```
ros2 service call /waypoints_pop std_srvs/srv/Empty {}
```
(멀티 모드) 실행 중 토글
```
ros2 param set /global_planner use_all_waypoints true   # ON
ros2 param set /global_planner use_all_waypoints false  # OFF
```

현재 경로 확인(poses가 나와야 정상)
```
timeout 2 ros2 topic echo /global_path | head -n 20
```

파라미터 확인
```
ros2 param get /global_planner use_all_waypoints
ros2 param get /global_planner base_frame
```

TF 확인 (숫자 갱신되면 OK)
```
ros2 run tf2_ros tf2_echo map ego_racecar/base_link | head -n 10
```

# 주의
frame id 제대로 확인.

ego_racecar/base_link 이므로 
node 이름이 base_link로 나오면 안됨.
```
ros2 param get /global_planner base_frame  # → ego_racecar/base_link
```


# 파라미터 요약
노드	파라미터	설명	기본
global_planner	base_frame	로봇 프레임	ego_racecar/base_link

〃	inflate_radius_m	장애물 팽창 반경(m)	0.03

〃	unknown_is_obstacle	-1(미지) 영역을 장애물 처리	false

〃	use_all_waypoints	모든 점 잇기(멀티)	false

〃	min_path_points	직선 페일백 샘플 수	60

pure_pursuit	lookahead	전방 주시거리(m)	0.8

〃	use_scan_safety	AEB 유사 안전정지	false

디버깅 단계에서는 AEB(안전정지)를 꺼두는 것을 권장 (use_scan_safety:=false).
