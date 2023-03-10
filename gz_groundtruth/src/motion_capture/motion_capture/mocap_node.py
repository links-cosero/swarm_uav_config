import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from nav_msgs.msg import Path
from geometry_msgs.msg import (
    PoseArray, 
    Pose,
    PoseStamped)
from px4_msgs.msg import VehicleOdometry

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
        self.mocap_odom_pub = self.create_publisher(VehicleOdometry, "/fmu/in/vehicle_mocap_odometry", 10)
        # Messages
        self.vehicle_path_msg = Path()
        self.vehicle_path_msg.header.frame_id = 'map'
        self.vehicle_pose_msg = PoseStamped()
        self.gz_pose_msg = Pose()
        self.mocap_odom_msg = None
        # Timers
        self.get_timestamp = lambda : int(Clock().now().nanoseconds / 1000)

        self.launch_ros_gz_bridge("/world/default/pose/info","geometry_msgs/msg/PoseArray","gz.msgs.Pose_V")

    def listener_cb(self, msg: PoseArray):
        # Nell'array di oggetti Pose il secondo è quello del drone a cui siamo interessati. L'ho
        # trovato a tentativi guardando i numeri più convincenti e sono sicuro sia quello perchè 
        # coincide con la visuale nella simulazione. E' ripetibile tutte le simulazioni e funziona sempre
        # ma penso valga solo per la specifica simulazione 'make px4_sitl gz_x500'. 
        self.gz_pose_msg : Pose = msg.poses[1]

        # Change ref fram to FLU and publish msg
        self.vehicle_pose_msg = self.gazebo_to_flu_tf(self.gz_pose_msg)
        self.pose_pub.publish(self.vehicle_pose_msg)
        # Publish path msg
        self.vehicle_path_msg.poses.append(self.vehicle_pose_msg)
        self.path_pub.publish(self.vehicle_path_msg)
        # Create mocap odometry msg and publish to PX4
        self.mocap_odom_msg = self.create_odometry_msg(self.vehicle_pose_msg)
        # self.mocap_odom_pub.publish(self.mocap_odom_msg)

    def launch_ros_gz_bridge(self, topic:str, ros_type:str, gz_type:str):
        """ Launch subprocess to bridge Gazebo and ROS2 """

        argument = f"{topic}@{ros_type}[{gz_type}"
        command = "ros2 run ros_gz_bridge parameter_bridge "+argument
        subprocess.Popen(command.split(' '))


    def gazebo_to_flu_tf(self, gz_pose: Pose) -> PoseStamped:
        """
        Transform from gazebo Pose to FLU pose for ROS2. Is done doing
        a rotation of -90° arounf z axis (global frame). 
        """
        vehicle_pose_msg = PoseStamped()
        # vehicle_pose_msg.header.stamp = self.get_timestamp()
        vehicle_pose_msg.header.frame_id='map'
        # Position
        vehicle_pose_msg.pose.position.x =  gz_pose.position.y
        vehicle_pose_msg.pose.position.y = -gz_pose.position.x
        vehicle_pose_msg.pose.position.z =  gz_pose.position.z
        # Orientation
        x_gz = gz_pose.orientation.x
        y_gz = gz_pose.orientation.y
        z_gz = gz_pose.orientation.z
        w_gz = gz_pose.orientation.w
        vehicle_pose_msg.pose.orientation.x = (- x_gz - y_gz) * sqrt(2)/2
        vehicle_pose_msg.pose.orientation.y = (- y_gz + x_gz) * sqrt(2)/2
        vehicle_pose_msg.pose.orientation.z = (- z_gz + w_gz) * sqrt(2)/2
        vehicle_pose_msg.pose.orientation.w = (- w_gz - z_gz) * sqrt(2)/2
        return vehicle_pose_msg
    

    def flu_to_ned_tf(self, pose : Pose) -> Pose:
        """
        Transform a FLU frame into NED
        """
        # TODO: da scrivere
        pass 

    
    def create_odometry_msg(self, pose : Pose) -> VehicleOdometry:
        """
        Create VehicleOdometry message. Not include velocities
        """
        odom_msg = VehicleOdometry()
        odom_msg.timestamp = self.get_timestamp()
        odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        ned_pose = self.flu_to_ned_tf(pose)
        odom_msg.position = ned_pose.position
        odom_msg.q = ned_pose.orientation
        return odom_msg

def main(args=None):
    rclpy.init(args=args)
    sub = MocapNode()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
