"""
Intercept Mission Launch
========================

카메라 브리지 + 요격 제어 노드를 한 번에 실행합니다.
YOLO 감지기(tools/yolo_live_detect.py)는 별도 터미널에서 실행해야 합니다.

사전 조건 (별도 터미널):
  T1: MicroXRCEAgent udp4 -p 8888
  T2: cd ~/dev/PX4-Autopilot
      PX4_GZ_WORLD=simple_windy_balloon make px4_sitl gz_x500_mono_cam
  T3: (이 런치 파일)
  T4: python3 tools/yolo_live_detect.py --model <model.pt> --show

사용법:
  ros2 launch interceptor_control intercept.launch.py
  ros2 launch interceptor_control intercept.launch.py target_altitude:=-4.0
  ros2 launch interceptor_control intercept.launch.py \\
      target_altitude:=-4.0 v_approach:=1.5 v_intercept:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)

    gz_image_topic = (
        f'/world/{world}/model/{model}'
        f'/link/camera_link/sensor/imager/image'
    )
    gz_caminfo_topic = (
        f'/world/{world}/model/{model}'
        f'/link/camera_link/sensor/imager/camera_info'
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        output='screen',
        arguments=[gz_image_topic],
        remappings=[(gz_image_topic, '/camera/image')],
    )

    caminfo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='caminfo_bridge',
        output='screen',
        arguments=[
            f'{gz_caminfo_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        remappings=[(gz_caminfo_topic, '/camera/camera_info')],
    )

    target_altitude = LaunchConfiguration('target_altitude').perform(context)
    v_approach = LaunchConfiguration('v_approach').perform(context)
    v_intercept = LaunchConfiguration('v_intercept').perform(context)
    Kp_yaw = LaunchConfiguration('Kp_yaw').perform(context)
    Kp_vz = LaunchConfiguration('Kp_vz').perform(context)
    search_yaw_rate = LaunchConfiguration('search_yaw_rate').perform(context)

    intercept_node = Node(
        package='interceptor_control',
        executable='intercept',
        name='intercept',
        output='screen',
        parameters=[{
            'target_altitude': float(target_altitude),
            'v_approach': float(v_approach),
            'v_intercept': float(v_intercept),
            'Kp_yaw': float(Kp_yaw),
            'Kp_vz': float(Kp_vz),
            'search_yaw_rate': float(search_yaw_rate),
        }],
    )

    return [image_bridge, caminfo_bridge, intercept_node]


def generate_launch_description():
    return LaunchDescription([
        # Gazebo 환경
        DeclareLaunchArgument('world', default_value='simple_windy_balloon'),
        DeclareLaunchArgument('model', default_value='x500_mono_cam_0'),

        # 비행 파라미터
        # 풍선이 Gazebo ENU Z=4m → NED z=-4.0 → target_altitude 기본값 -4.0
        DeclareLaunchArgument('target_altitude', default_value='-4.0',
                              description='이륙 목표 고도 NED (음수=위)'),
        DeclareLaunchArgument('v_approach',     default_value='1.5',
                              description='TRACK 접근 속도 m/s'),
        DeclareLaunchArgument('v_intercept',    default_value='4.0',
                              description='INTERCEPT 돌진 속도 m/s'),
        DeclareLaunchArgument('Kp_yaw',         default_value='0.8',
                              description='yaw 제어 이득'),
        DeclareLaunchArgument('Kp_vz',          default_value='1.0',
                              description='고도 제어 이득'),
        DeclareLaunchArgument('search_yaw_rate', default_value='0.3',
                              description='탐색 회전 속도 rad/s'),

        OpaqueFunction(function=launch_setup),
    ])
