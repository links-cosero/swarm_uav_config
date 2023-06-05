from setuptools import setup

package_name = 'vicon_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='judocero',
    maintainer_email='morazzo.davide@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vicon_ros2 = vicon_ros2.vicon_ros2:main',
            'px4_offboard = vicon_ros2.offboard_uav:main'
        ],
    },
)
