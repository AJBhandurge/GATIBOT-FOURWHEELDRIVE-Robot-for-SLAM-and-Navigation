import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('smorphi_bringup')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_path, 'config', 'nav2_params2.yaml'),
            description='Full path to nav2 params file'
        ),

        # ── SLAM TOOLBOX ──────────────────────────────────────────────
        # NOT in lifecycle manager — manages its own lifecycle
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'solver_plugin': 'solver_plugins::CeresSolver',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',  # ← FIXED: match yaml
                'scan_topic': '/scan',
                'mode': 'mapping',
                'map_update_interval': 2.0,
                'resolution': 0.05,
                'max_laser_range': 5.0,
                'minimum_time_interval': 0.5,
                'transform_publish_period': 0.02,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 30.0,
                'stack_size_to_use': 40000000,
                'enable_interactive_mode': False,
            }]
        ),

        # ── NAV2 NODES ────────────────────────────────────────────────
        # DO NOT add costmap nodes here separately —
        # lifecycle_manager handles costmaps via node_names in yaml
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', '/cmd_vel')]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),

        # ── LIFECYCLE MANAGER ─────────────────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'waypoint_follower',
                ]
            }]
        ),
    ])
