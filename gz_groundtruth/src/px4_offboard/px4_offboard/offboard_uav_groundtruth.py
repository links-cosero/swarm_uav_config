"""
Python implementation of Offboard Control

"""
import rclpy
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy import utilities
import launch
from geometry_msgs.msg import Pose
from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy)

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleOdometry

from math import sqrt


class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,"/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,"/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand,"/fmu/in/vehicle_command", 10)

        self.vehicle_status_sub = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.store_msg_cb, qos_profile) 
        self.vehicle_visual_odometry_sub = self.create_subscription(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", self.store_msg_cb, qos_profile)

        self.vehicle_status_msg : VehicleStatus = None
        self.vehicle_visual_odometry_msg : VehicleOdometry = None
        self.offboard_setpoint_counter_ = 0
        # Stato missione per macchina a stati
        self.mission_state = 0
        # Waypoint corrente pubblicato da timer_offboard_cb()
        self.wpt_cnt = 0
        # [x, y, z, yaw]
        self.waypoint_list = [
            ( 0.0, 0.0, 0.0, 0.0),
            ( 4.0, 4.0, -5.0, 3.14/2),
            ( 4.0,-4.0, -5.0, 3.14),
            (-4.0,-4.0, -5.0, -3.14/2),
            (-4.0, 4.0, -5.0, -3.14),
            ( 0.0, 0.0, -5.0, 0.0)
        ]
        self.current_waypoint = self.waypoint_list[self.wpt_cnt][0:3]
        self.current_yaw = self.waypoint_list[self.wpt_cnt][3]

        # Timers
        self.timer_offboard = self.create_timer(0.1, self.timer_offboard_cb)
        self.timer_mission = self.create_timer(1, self.mission)

         
    
    def mission(self):
        """ Funzione mission organizzata come macchina a stati """

        if self.mission_state == 0:
            """ Arming and takeoff to 1st waypoint """
            self.get_logger().info("Mission started")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.arm()
            if self.vehicle_status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                # If vehicle armed
                self.get_logger().info("Vehicle armed")
                self.mission_state = 1
            else:
                self.mission_state = 0

        elif self.mission_state == 1:
            # Imposta il primo waypoint
            self.current_waypoint = self.waypoint_list[self.wpt_cnt][0:3]
            self.current_yaw = self.waypoint_list[self.wpt_cnt][3]
            if distance(self.vehicle_visual_odometry_msg, self.current_waypoint) < 0.3: 
                self.get_logger().info(f"Next waypoint {self.current_waypoint}")
                self.wpt_cnt += 1 # Arrived to waypoint
            
            if self.wpt_cnt >= len(self.waypoint_list):
                self.mission_state = 2
            else:
                self.mission_state = 1

        elif self.mission_state == 2:
            """Landing"""
            self.get_logger().info("Landing request")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.mission_state = 4
        
        elif self.mission_state == 4:
            self.timer_offboard.cancel()
            self.get_logger().info("Mission finished")
            self.timer_mission.cancel()
            exit()
            
    
    def timer_offboard_cb(self):
        # Funzione richiamata ogni 20ms e invia i seguenti messaggi
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()


    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    '''
	Publish the offboard control mode.
	For this example, only position and altitude controls are active.
    '''

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    '''
	Publish a trajectory setpoint
	For this example, it sends a trajectory setpoint to make the
	vehicle hover at 5 meters with a yaw angle of 180 degrees.
    '''

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = self.current_waypoint# [0.0, 0.0, -5.0] 
        msg.yaw = self.current_yaw  # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)

    '''
    Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
    '''
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 10  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)
    
    def store_msg_cb(self, msg) -> None:
        """ Callback for multiple subscriptions. Detect message type and store it """

        if isinstance(msg, VehicleStatus):
            self.vehicle_status_msg = msg
        elif isinstance(msg, VehicleOdometry):
            self.vehicle_visual_odometry_msg = msg
        else:
            self.get_logger().info("Unknown message type")


def distance(drone_pose:VehicleOdometry, waypoint:list) -> float:
    """Distance between poses positions"""
    delta_x = drone_pose.position[0] - waypoint[0] 
    delta_y = drone_pose.position[1] - waypoint[1]
    delta_z = drone_pose.position[2] - waypoint[2]

    return sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z)




def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
