"""
Python implementation of Offboard Control

"""


import rclpy 
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy import utilities

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleAttitudeSetpoint



class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,"/fmu/in/offboard_control_mode", 10)
        self.vehicle_attitude_setpoint_publisher_ = self.create_publisher(VehicleAttitudeSetpoint,"/fmu/in/vehicle_attitude_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand,"/fmu/in/vehicle_command", 10)

        self.offboard_setpoint_counter_ = 0
        # Stato missione per macchina a stati
        self.mission_state = 0

        # Timers
        self.timer_offboard = self.create_timer(0.5, self.timer_offboard_cb)
        self.timer_mission = self.create_timer(10, self.mission)

         
    
    def mission(self):
        """ Funzione mission organizzata come macchina a stati """

        if self.mission_state == 0:
            """ Arming and no moving setpoint """
            self.get_logger().info("Mission started")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.mission_state = 1

        elif self.mission_state == 1:    
            self.arm()
            self.get_logger().info("Vehicle armed")
            self.mission_state = 2
        
        elif self.mission_state == 2:
            self.get_logger().info("Attitude Setpoint sent")
            self.publish_vehicle_attitude_setpoint()
            self.mission_state = 3 
    
    def timer_offboard_cb(self):
        self.publish_offboard_control_mode()
        


    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")

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
	Publish a Vehicle Attitude setpoint
    with no rotation and half throttle speed in order to arm the drone
    '''
    def ENU2NED_vector_converter(self, vect):
        vect = np.array(vect)
        #Rotation around Z-axis
        rotZ = np.asfarray(np.array([[np.cos(np.pi/2), -np.sin(np.pi/2), 0],
                                     [np.sin(np.pi/2), np.cos(np.pi/2), 0], 
                                     [0, 0, 1]]), dtype = np.float32)
        var = np.dot(rotZ,vect)
        rotX = np.asfarray(np.array([[1, 0, 0],
                                    [0, np.cos(np.pi/2), -np.sin(np.pi/2)], 
                                    [0, np.sin(np.pi/2), -np.cos(np.pi/2)]]), dtype = np.float32)
        var = np.dot(rotZ,var)
        return var

    def publish_vehicle_attitude_setpoint(self):
        msg = VehicleAttitudeSetpoint()
        msg.q_d  = np.asfarray([1, 0, 0, 0], dtype = np.float32)# no rotation
        msg.thrust_body = np.asfarray([0, 0, -1], dtype = np.float32) # half-throttle
        msg.thrust_body = self.ENU2NED_vector_converter(msg.thrust_body)
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_attitude_setpoint_publisher_.publish(msg)

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
