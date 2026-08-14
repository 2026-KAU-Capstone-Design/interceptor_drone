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
            'point_mission = interceptor_control.missions.point_to_point.point_mission:main',
            'hover_mission = interceptor_control.missions.hover.hover_mission:main',
            'point_nav = interceptor_control.missions.mission2_nav.point_nav:main',
            'hover_land = interceptor_control.missions.mission1_hover.hover_land:main',
            'circle_mission = interceptor_control.missions.circle.circle_mission:main',
            'figure8_mission = interceptor_control.missions.figure8.figure8_mission:main',
	    'high_speed_mission = interceptor_control.missions.high_speed.high_speed_mission:main',
	    'high_speed_velocity_mission = interceptor_control.missions.high_speed.high_speed_velocity_mission:main',
	    'transition_1_mission = interceptor_control.missions.transition.transition_1_mission:main',
        ],
    },
)
