#!/bin/python3

"""
SCRIPT THAT PERFORMS SIMPLE MISSION FOR 1 DRONE
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import numpy as np

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand

from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy)

class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,"/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,"/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand,"/fmu/in/vehicle_command", 10)

        # Mission organized as a state machine
        self.mission_state = 0
        # Starting value of the waypoint updated within each iteration 
        # of the state machine inside timer_offbaord callback
        self.current_waypoint = [0.0, 0.0, 0.0]

        # Timers
        self.timer_offboard = self.create_timer(0.1, self.timer_offboard_cb)
        self.timer_mission = self.create_timer(10, self.mission)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

    def mission(self):

        if self.mission_state == 0:

            self.get_logger().info("Mission started")
            
            """ Arming and takeoff to 1st waypoint """
            """
            Python implementation of Offboard Control
            param1 is set to 1 to enable the custom mode.
            param2 is set to 6 to indicate the offboard mode.
            Other options for param2 include:
            param2 = 1 (MANUAL)
            param2 = 2 (ALTCTL)
            param2 = 3 (POSCTL)
            param2 = 4 (AUTO)
            param2 = 5 (ACRO)
            param2 = 6 (OFFBOARD)
            param2 = 7 (STABILIZED)
            param2 = 8 (RATTITUDE)
            """
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.mission_state = 1

        elif self.mission_state == 1:    
            self.arm()
            self.get_logger().info("Vehicle armed")
          
            self.get_logger().info("First waypoint")
            self.current_waypoint = np.asfarray([-2.5, -2.5, 2], dtype = np.float32)
            self.ENU2NED_vector_converter()
            print(self.current_waypoint)
            self.mission_state = 2

        elif self.mission_state == 2:
            """Waypoint 2"""
            self.get_logger().info("Second waypoint")
            # Imposta secondo waypoint, 
            self.current_waypoint = np.asfarray([-2.5, 2.5, 2], dtype = np.float32)
            self.ENU2NED_vector_converter()
            print(self.current_waypoint)
            
            self.mission_state = 3
        
        elif self.mission_state == 3:
            """Waypoint 2"""
            self.get_logger().info("Third waypoint")
            # Imposta secondo waypoint
            self.current_waypoint = np.asfarray([2.5, 2.5, 2], dtype = np.float32)
            self.ENU2NED_vector_converter()
            print(self.current_waypoint)
            self.mission_state = 4
        
        elif self.mission_state == 4:
            """Waypoint 2"""
            self.get_logger().info("Forth waypoint")
            # Imposta secondo waypoint
            self.current_waypoint = np.asfarray([2.5, -2.5, 2], dtype = np.float32)
            self.ENU2NED_vector_converter()
            print(self.current_waypoint)
            self.mission_state = 5

        elif self.mission_state == 5:
            """Landing"""
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.mission_state = 6
        
        elif self.mission_state == 6:
            self.timer_offboard.cancel()
            self.get_logger().info("Mission finished")
            self.timer_mission.cancel()
            exit()        
    
    def timer_offboard_cb(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Disarm the vehicle
    # def disarm(self):
    #   self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    #   self.get_logger().info("Disarm command send")


    ''' Publish the offboard control mode.
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
        msg.position = self.current_waypoint

        #The drone starts with a yaw of pi/2
        msg.yaw = np.pi/2  # [-PI:PI]
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
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def ENU2NED_vector_converter(self):

        # Rotation around Z-axis of 90° Degrees
        rotZ = np.asfarray(np.array([[np.cos(np.pi/2), -np.sin(np.pi/2), 0],
                                    [np.sin(np.pi/2), np.cos(np.pi/2), 0], 
                                    [0, 0, 1]]), dtype = np.float32)
        self.current_waypoint = np.dot(rotZ,self.current_waypoint)

        # Rotation around X-axis of 180° Degrees
        rotX = np.asfarray(np.array([[1, 0, 0],
                                    [0, np.cos(np.pi), -np.sin(np.pi)], 
                                    [0, np.sin(np.pi), np.cos(np.pi)]]), dtype = np.float32)
        self.current_waypoint = np.dot(rotX,self.current_waypoint)
    


def main(args=None):
    rclpy.init(args=args)
    print("Starting First test of an Offboard Mission...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()
    print("Clean exit")
    exit(0)

if __name__ == '__main__':
    main()
    
    
