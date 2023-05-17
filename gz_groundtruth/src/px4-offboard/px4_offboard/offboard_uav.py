"""
Python implementation of Offboard Control

"""


import rclpy
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy import utilities
import launch

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand



class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,"/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,"/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand,"/fmu/in/vehicle_command", 10)

        self.offboard_setpoint_counter_ = 0
        # Stato missione per macchina a stati
        self.mission_state = 0
        # Waypoint corrente pubblicato da timer_offboard_cb()
        self.current_waypoint = [0.0, 0.0, -5.0]

        # Timers
        self.timer_offboard = self.create_timer(0.1, self.timer_offboard_cb)
        self.timer_mission = self.create_timer(8, self.mission)

         
    
    def mission(self):
        """ Funzione mission organizzata come macchina a stati """

        if self.mission_state == 0:
            """ Arming and takeoff to 1st waypoint """
            self.get_logger().info("Mission started")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.mission_state = 1

        elif self.mission_state == 1:
            self.arm()
            self.get_logger().info("Vehicle armed")
            # Imposta il primo waypoint
            self.get_logger().info("First waypoint")
            self.current_waypoint = [0.0, 0.0, -5.0]
            self.mission_state = 2

        elif self.mission_state == 2:
            """Waypoint 2"""
            self.get_logger().info("Second waypoint")
            # Imposta secondo waypoint
            self.current_waypoint = [3.0, 3.0, -4.0]
            self.mission_state = 3

        elif self.mission_state == 3:
            """Landing"""
            self.get_logger().info("Landing request")
            self.current_waypoint = [0.0, 0.0, 0.0]
            # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.mission_state = 4
        
        elif self.mission_state == 4:
            self.disarm()
            self.timer_offboard.cancel()
            self.get_logger().info("Mission finished")
            self.timer_mission.cancel()
            
    
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
        msg.yaw = -3.14  # [-PI:PI]
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
