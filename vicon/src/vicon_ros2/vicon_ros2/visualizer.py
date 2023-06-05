from re import M
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path

def vector2PoseMsg(frame_id, position, attitude):
    pose_msg = PoseStamped()
    # msg.header.stamp = Clock().now().nanoseconds / 1000
    pose_msg.header.frame_id=frame_id
    pose_msg.pose.orientation.w = attitude[0]
    pose_msg.pose.orientation.x = attitude[1]
    pose_msg.pose.orientation.y = attitude[2]
    pose_msg.pose.orientation.z = attitude[3]
    pose_msg.pose.position.x = position[0]
    pose_msg.pose.position.y = position[1]
    pose_msg.pose.position.z = position[2]
    return pose_msg

class PX4Visualizer(Node):

    def __init__(self):
        super().__init__('px4_visualizer')

        ## Configure subscritpions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.trajectory_setpoint_callback,
            qos_profile)

        self.vehicle_pose_pub = self.create_publisher(PoseStamped, '/px4_visualizer/vehicle_pose', 10)
        self.vehicle_path_pub = self.create_publisher(Path, '/px4_visualizer/vehicle_path', 10)
        self.setpoint_path_pub = self.create_publisher(Path, '/px4_visualizer/setpoint_path', 10)

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_path_msg = Path()
        self.setpoint_path_msg = Path()
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    def vehicle_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z

    def trajectory_setpoint_callback(self, msg):
        self.setpoint_position[0] = msg.position[0]
        self.setpoint_position[1] = -msg.position[1]
        self.setpoint_position[2] = -msg.position[2]

    def cmdloop_callback(self):
        vehicle_pose_msg = vector2PoseMsg('map', self.vehicle_local_position, self.vehicle_attitude)
        self.vehicle_pose_pub.publish(vehicle_pose_msg)

        # Publish time history of the vehicle path
        self.vehicle_path_msg.header = vehicle_pose_msg.header
        self.vehicle_path_msg.poses.append(vehicle_pose_msg) 
        self.vehicle_path_pub.publish(self.vehicle_path_msg)

        # Publish time history of the vehicle path
        setpoint_pose_msg = vector2PoseMsg('map', self.setpoint_position, self.vehicle_attitude)
        self.setpoint_path_msg.header = setpoint_pose_msg.header
        self.setpoint_path_msg.poses.append(setpoint_pose_msg)
        self.setpoint_path_pub.publish(self.setpoint_path_msg)

def main(args=None):
    rclpy.init(args=args)

    px4_visualizer = PX4Visualizer()

    rclpy.spin(px4_visualizer)

    px4_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
