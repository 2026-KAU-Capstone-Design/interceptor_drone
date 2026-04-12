"""
Offboard Hover 미션 Launch 파일.

사용법:
  ros2 launch interceptor_control hover.launch.py
  ros2 launch interceptor_control hover.launch.py altitude:=-10.0 duration:=60.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('altitude', default_value='-5.0',
                              description='목표 고도 (NED z, 음수=위)'),
        DeclareLaunchArgument('duration', default_value='30.0',
                              description='호버 유지 시간(초)'),
        DeclareLaunchArgument('wind_warn', default_value='8.0',
                              description='풍속 경고 기준 (m/s)'),
        DeclareLaunchArgument('wind_critical', default_value='15.0',
                              description='강풍 RTL 기준 (m/s)'),

        Node(
            package='interceptor_control',
            executable='offboard_hover',
            name='offboard_hover',
            output='screen',
            parameters=[{
                'target_altitude': LaunchConfiguration('altitude'),
                'hover_duration': LaunchConfiguration('duration'),
                'wind_speed_warn': LaunchConfiguration('wind_warn'),
                'wind_speed_critical': LaunchConfiguration('wind_critical'),
                'tilt_threshold_deg': 30.0,
                'attitude_recovery_timeout': 2.0,
                'position_threshold': 0.5,
            }],
        ),
    ])
