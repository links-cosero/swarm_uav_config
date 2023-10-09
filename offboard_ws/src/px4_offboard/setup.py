import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ronald/davide',
    maintainer_email='test@gmail.com',
    description='Package to run several tests using one drone and two drone respectively',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'visualizer = px4_offboard.visualizer:main',
                'visualizer2 = px4_offboard.visualizer2:main',
                'offboard_uav_test1 = px4_offboard.offboard_uav_test1:main',
                'offboard_uav_test2 = px4_offboard.offboard_uav_test2:main',
                'offboard_uav2_test = px4_offboard.offboard_uav2_test:main',
                'offboard_arming_altitude = px4_offboard.offboard_arming_altitude:main',      
                'offboard_uav_fake_mocap = px4_offboard.offboard_uav_fake_mocap:main', 
                'cpp_uav_base = px4_offboard.cpp_uav_base:main',
                'cpp_uav_real_sim = px4_offboard.offboard_uav_fake_mocap:main',    
        ],
    },
)
