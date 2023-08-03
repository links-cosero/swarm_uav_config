#!/bin/python3

"""
SCRIPT THAT PERFORMS COVERAGE PATH PLANNING OF 1 DRONE
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import numpy as np

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus

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
        self.vehicle_status_publisher_ = self.create_publisher(VehicleStatus,"/fmu/in/vehicle_status", 10)

        # Stato missione per macchina a stati
        self.mission_state = 0
        # Waypoint corrente pubblicato da timer_offboard_cb()
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
            """ Arming and takeoff to 1st waypoint """
            self.get_logger().info("Mission started")
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
            self.current_waypoint = np.asfarray([2.5, -2.5, 2], dtype = np.float32)
            self.ENU2NED_vector_converter()
            print(self.current_waypoint)
            self.mission_state = 4
        
        elif self.mission_state == 4:
            """Waypoint 2"""
            self.get_logger().info("Forth waypoint")
            # Imposta secondo waypoint
            self.current_waypoint = np.asfarray([2.5, 2.5, 2], dtype = np.float32)
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
        # Funzione richiamata ogni 20ms e invia i seguenti messaggi
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Disarm the vehicle
    # def disarm(self):
    #         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    #         self.get_logger().info("Disarm command send")


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
    
class GridMap:

    def __init__(self, data, width, height, resolution,
                 center_x, center_y, origin):
        """__init__
        :data: value of each cell either 0 (free) or 100 (occupied)
        :param width: number of grid for width
        :param height: number of grid for height
        :param resolution: grid resolution [m]
        :param center_x: center x position  [m]
        :param center_y: center y position [m]
        :origin: position of starting point
        """

        self.data = data
        self.width = width
        self.height = height
        self.resolution = resolution
        self.center_x = center_x
        self.center_y = center_y
        self.origin = origin

        self.n_data = self.width * self.height

    """ Defines the grid map area to cover"""
    def area_definition(x, y):
        resolution = 1.0 #meters

        # calculate dimensions
        x_max = int((max(x) - min(x))/resolution)
        y_max = int((max(y) - min(y))/resolution)
        
        data = []
        cell_free = 0
        for i in range(y_max):
            for j in range(x_max):
                data.append(cell_free)
        
        width = x_max
        height = y_max
        center_x = 0
        center_y = 0
        origin = [min(x),min(y)]
        gridmap = GridMap(data, width, height, resolution, center_x, center_y, origin)

        return gridmap

    """ Divide the grid map area to cover with multiple CPS"""
    def area_division(grid_map, uav_pose):
    
        # convert swarm pose to grid
        # (i,j) are the coordinates of the position of the drone inside the grid
        print(grid_map.width, grid_map.height, grid_map.origin[:])
        i = (grid_map.width-1) - int((uav_pose[0] - grid_map.origin[0])/grid_map.resolution)
        j = (grid_map.height-1) - int((uav_pose[1] - grid_map.origin[1])/grid_map.resolution)

        return i, j

def main(args=None):
    rclpy.init(args=args)
    print("Starting Offboard Mission...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    # Edges of the squared surface that needs to be covered
    # Defined in coordinates w.r.t gazebo center
    x = [3, 3, -3, -3]
    y = [3, -3, 3, -3]

    offboard_control.destroy_node()
    rclpy.shutdown()
    print("Clean exit")
    exit(0)

if __name__ == '__main__':
    main()
    
    
