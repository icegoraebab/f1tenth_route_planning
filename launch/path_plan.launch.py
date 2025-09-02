from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_route_planning',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': 'ego_racecar/base_link',  # 필요시 'ego_racecar/base_link'
                'inflate_radius_m': 0.25,
                'allow_diagonal': True,
                'unknown_is_obstacle': True,
                'clicked_topic': '/clicked_point',
                'path_topic': '/global_path',
            }]
        ),
        Node(
            package='f1tenth_route_planning',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': 'ego_racecar/base_link',
                'cmd_topic': '/drive',
                'lookahead': 1.0,
                'wheelbase': 0.33,
                'v_max': 2.0,
                'v_min': 0.2,
                'k_curv': 2.0,
                'goal_tol': 0.4,
                'use_scan_safety': True,
                'stop_dist': 0.6,
                'slow_dist': 1.0,
                'front_angle_deg': 60.0,
            }]
        ),
    ])
