"""
Camera Bridge Launch — Gazebo 카메라 토픽을 ROS2로 브리지

Gazebo Harmonic의 카메라 센서가 publish하는 gz transport 토픽을
ros_gz_bridge / ros_gz_image 를 통해 ROS2 sensor_msgs/Image 토픽으로 변환한다.

사용법:
  # T1
  MicroXRCEAgent udp4 -p 8888

  # T2 — 카메라 장착 X500 + 풍선 월드
  cd $PX4_AUTOPILOT_DIR && PX4_GZ_WORLD=simple_windy_balloon make px4_sitl gz_x500_mono_cam

  # T3 — 카메라 브리지 실행
  ros2 launch interceptor_control camera_bridge.launch.py

  # T4 — ROS2 측에서 이미지 확인
  ros2 run interceptor_control image_probe
  # 또는: ros2 run rqt_image_view rqt_image_view /camera/image

기본 인자(다른 월드/모델로 바꾸려면):
  ros2 launch interceptor_control camera_bridge.launch.py \
      world:=simple_windy_balloon model:=x500_mono_cam_0

토픽 매핑:
  Gazebo (gz transport):
    /world/<world>/model/<model>/link/camera_link/sensor/imager/image
    /world/<world>/model/<model>/link/camera_link/sensor/imager/camera_info
       ↓ (ros_gz_image / ros_gz_bridge + remap)
  ROS2:
    /camera/image        (sensor_msgs/Image)
    /camera/camera_info  (sensor_msgs/CameraInfo)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration 을 런타임에 문자열로 평가
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

    # ros_gz_image: Gazebo image → ROS2 sensor_msgs/Image (이미지 전용 브리지)
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        output='screen',
        arguments=[gz_image_topic],
        remappings=[(gz_image_topic, '/camera/image')],
    )

    # ros_gz_bridge: camera_info (일반 메시지 브리지)
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

    return [image_bridge, caminfo_bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='simple_windy_balloon',
            description='Gazebo 월드 이름 (PX4_GZ_WORLD 와 일치해야 함)',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='x500_mono_cam_0',
            description='PX4 SITL이 spawn한 드론 모델 이름 (보통 x500_mono_cam_0)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
