from glob import glob
from setuptools import find_packages, setup

package_name = 'interceptor_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='jazzskript',
    maintainer_email='jazzskript@gmail.com',
    description='Interceptor Drone offboard control and mission nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard_hover = interceptor_control.offboard_hover:main',
            'point_nav = interceptor_control.missions.mission2_nav.point_nav:main',
            'hover_land = interceptor_control.missions.mission1_hover.hover_land:main',
            # 카메라 이미지 수신 검증 노드
            'image_probe = interceptor_control.perception.image_probe:main',
        ],
    },
)