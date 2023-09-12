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
import rclpy
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy import utilities

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitude

from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy,
    QoSDurabilityPolicy)

qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )


class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,"/drone2/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,"/drone2/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand,"/drone2/fmu/in/vehicle_command", 10)
        self.vehicle_status = self.create_subscription(VehicleStatus, "/drone2/fmu/out/vehicle_status", self.v_status_cb, qos_profile)
        self.vehicle_status_msg = None

        self.offboard_setpoint_counter_ = 0
        # Stato missione per macchina a stati
        self.mission_state = 0
        # Waypoint corrente pubblicato da timer_offboard_cb()
        self.current_waypoint = [0.0, 0.0, 0.0]

        # Timers
        self.timer_offboard = self.create_timer(0.1, self.timer_offboard_cb)
        self.timer_mission = self.create_timer(8, self.mission)

        ## ------- FAKE MOCAP --------- ##
        # qos_profile = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1
        # )
        # self.attitude_sub =     self.create_subscription(VehicleAttitude,'/fmu/out/vehicle_attitude', self.attitude_cb, qos_profile)
        # self.fake_mocap_publisher_ = self.create_publisher(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", 10)
        # self.timer_fake_mocap = self.create_timer(0.033, self.fake_mocap_cb)
        # self.timestamp = 0
        # self.timestamp_sample = 0     
    
    def v_status_cb(self, msg: VehicleStatus):
        self.vehicle_status_msg = msg

    def mission(self):
        """ Funzione mission organizzata come macchina a stati """
        if self.vehicle_status_msg == None:
            return

        
        # if self.mission_state == -1:
        #     if not self.vehicle_status_msg.nav_state == VehicleStatus.NAVIGATION_STATE_STAB:
        #         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 7.)
        #         self.mission_state = -1
        #     else:
        #         self.mission_state = 0


        if self.mission_state == 0:
            """ Arming and takeoff to 1st waypoint """
            if not self.vehicle_status_msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info("Mission started")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
                self.mission_state = 0
            else:
                if self.vehicle_status_msg.pre_flight_checks_pass:
                    self.mission_state = 1                    
           

        elif self.mission_state == 1:    
            if not self.vehicle_status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.arm()
                self.mission_state =1 
            else:
                self.get_logger().info("Vehicle armed")
                # Imposta il primo waypoint
                self.get_logger().info("First waypoint x=0 y=0 z=-0.3")
                self.current_waypoint = [0.0, 0.0, -0.3]
                self.mission_state = 2

        elif self.mission_state == 2:
            """Waypoint 2"""
            self.get_logger().info("Second waypoint x=0 y=1 z=-0.5")
            # Imposta secondo waypoint
            self.current_waypoint = [0.0, 0.0, -1.0]
            self.mission_state = 3

        elif self.mission_state == 3:
            """Landing"""
            self.get_logger().info("Landing request")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, float('nan'), float('nan'))
            self.timer_offboard.cancel()
            self.get_logger().info("Mission finished")
            self.timer_mission.cancel()
            self.mission_state = 4
        
        elif self.mission_state == 4:
            exit()        
    
    def timer_offboard_cb(self):
        # Funzione richiamata ogni 10ms e invia i seguenti messaggi
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
        msg.position = self.current_waypoint
        msg.yaw = 0.0  # [-PI:PI]
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

    def attitude_cb(self, msg : VehicleAttitude):
        self.attitude_msg = msg
        # Store PX4 timestamp
        self.timestamp = msg.timestamp
        self.timestamp_sample = msg.timestamp_sample
        # Store drone attitude quaternion
        self.vehicle_q = msg.q
    

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
