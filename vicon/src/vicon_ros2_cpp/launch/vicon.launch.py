from launch import LaunchDescription
from launch_ros.actions import Node
import launch

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vicon_ros2_cpp',
            executable='vicon',
            name='sim'
        ),
        launch.actions.ExecuteProcess(
        cmd=[[
            'ros2 ',
            'bag ',
            'record ',
            '/fmu/in/vehicle_visual_odometry ',
            '/vicon/pose ',
            '/fmu/out/vehicle_local_position '
        ]],
        shell=True)
    ])