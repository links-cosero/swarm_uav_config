import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import (
    PoseArray, 
    Pose,
    PoseStamped)

import subprocess
from math import sqrt

class MocapNode (Node):
    def __init__(self) -> None:
        super().__init__('mocap_node')
        # Subscribers
        self.subscriber_ = self.create_subscription(PoseArray, '/world/default/pose/info', self.listener_cb, 10)
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/x500/pose", 10)
        self.path_pub = self.create_publisher(Path, "/x500/path", 10)
        # Messages
        self.vehicle_path_msg = Path()
        self.vehicle_pose_msg = PoseStamped()
        self.gz_pose_msg = Pose()

        self.launch_ros_gz_bridge(
            "/world/default/pose/info",
            "geometry_msgs/msg/PoseArray",
            "gz.msgs.Pose_V"
        )

    def listener_cb(self, msg: PoseArray):
        # Nell'array di oggetti Pose il secondo è quello del drone a cui siamo interessati. L'ho
        # trovato a tentativi guardando i numeri più convincenti e sono sicuro sia quello perchè 
        # coincide con la visuale nella simulazione. E' ripetibile tutte le simulazioni e funziona sempre
        # ma penso valga solo per la specifica simulazione 'make px4_sitl gz_x500'. 
        self.gz_pose_msg : Pose = msg.poses[1]

        # Change ref fram to FLU and publish msg
        self.vehicle_pose_msg = PoseStamped()
        self.vehicle_pose_msg.header.frame_id='map'
        # Rotation from ENU to FLU reference frame -> Rot_z(-90) done both to 
        # position and to orientation. 

        # Position
        self.vehicle_pose_msg.pose.position.x =  self.gz_pose_msg.position.y
        self.vehicle_pose_msg.pose.position.y = -self.gz_pose_msg.position.x
        self.vehicle_pose_msg.pose.position.z =  self.gz_pose_msg.position.z
        # Orientation
        x_gz = self.gz_pose_msg.orientation.x
        y_gz = self.gz_pose_msg.orientation.y
        z_gz = self.gz_pose_msg.orientation.z
        w_gz = self.gz_pose_msg.orientation.w
        self.vehicle_pose_msg.pose.orientation.x = (- x_gz - y_gz) * sqrt(2)/2
        self.vehicle_pose_msg.pose.orientation.y = (- y_gz + x_gz) * sqrt(2)/2
        self.vehicle_pose_msg.pose.orientation.z = (- z_gz + w_gz) * sqrt(2)/2
        self.vehicle_pose_msg.pose.orientation.w = (- w_gz - z_gz) * sqrt(2)/2

        self.pose_pub.publish(self.vehicle_pose_msg)

        self.vehicle_path_msg.header.frame_id = 'map'
        self.vehicle_path_msg.poses.append(self.vehicle_pose_msg)
        self.path_pub.publish(self.vehicle_path_msg)

    def launch_ros_gz_bridge(self, topic:str, ros_type:str, gz_type:str):
        """ Launch subprocess to bridge Gazebo and ROS2 """

        argument = f"{topic}@{ros_type}[{gz_type}"
        command = "ros2 run ros_gz_bridge parameter_bridge "+argument
        subprocess.Popen(command.split(' '))

def main(args=None):
    rclpy.init(args=args)
    sub = MocapNode()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
