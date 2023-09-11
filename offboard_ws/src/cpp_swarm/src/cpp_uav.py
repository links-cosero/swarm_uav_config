#!/bin/python3

"""
SCRIPT THAT PERFORMS COVERAGE PATH PLANNING OF 1 DRONE
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import numpy as np
from enum import Enum

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus

from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy)

MAP_WIDTH = 3
MAP_HEIGHT = 3
CELL_UNIT = 1
SUBUNIT_CELL = 0.5

class Dir_neighbours(Enum):
    POS_X = 1, 
    NEG_Y = 2, 
    NEG_X = 3, 
    POS_Y = 4

class Dir_subcells(Enum):
    TOP_RIGHT = 1, 
    TOP_LEFT = 2, 
    BOTTOM_LEFT = 3, 
    BOTTOM_RIGHT = 4

class SubCell:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def display(self):
         print("X= "+ str(self.x) + " Y = " + str(self.y))

    def compareSubCells(self, sub1, sub2):
        if (sub1.x == sub2.x and sub1.y == sub1.y):
            return True
        else:
            return False

class SubCellGroup(SubCell):
    
    def __init__(self, parent_x=0, parent_y=0):
        
        self.parent_x = parent_x
        self.parent_y = parent_y

        self.subcells[Dir_subcells.TOP_RIGHT].x=parent_x+SUBUNIT_CELL/2
        self.subcells[Dir_subcells.TOP_RIGHT].y=parent_y-SUBUNIT_CELL/2

        self.subcells[Dir_subcells.TOP_LEFT].x=parent_x-SUBUNIT_CELL/2
        self.subcells[Dir_subcells.TOP_LEFT].y=parent_y-SUBUNIT_CELL/2

        self.subcells[Dir_subcells.BOTTOM_LEFT].x=parent_x-SUBUNIT_CELL/2
        self.subcells[Dir_subcells.BOTTOM_LEFT].y=parent_y+SUBUNIT_CELL/2

        self.subcells[Dir_subcells.BOTTOM_RIGHT].x=parent_x+SUBUNIT_CELL/2
        self.subcells[Dir_subcells.BOTTOM_RIGHT].y=parent_y+SUBUNIT_CELL/2
        

class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,"/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,"/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand,"/fmu/in/vehicle_command", 10)
        self.vehicle_status_publisher_ = self.create_publisher(VehicleStatus,"/fmu/in/vehicle_status", 10)

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

        # The drone starts with a yaw of pi/2
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
    
class Graph:
    def __init__(self, vertex=0):
        self.V = vertex
        self.graph = []
 
    def add_edge(self, u, v, w):
        self.graph.append([u, v, w])
 
 
    def search(self, parent, i):
        if parent[i] == i:
            return i
        return self.search(parent, parent[i])
 
    def apply_union(self, parent, rank, x, y):
        xroot = self.search(parent, x)
        yroot = self.search(parent, y)
        if rank[xroot] < rank[yroot]:
            parent[xroot] = yroot
        elif rank[xroot] > rank[yroot]:
            parent[yroot] = xroot
        else:
            parent[yroot] = xroot
            rank[xroot] += 1
 
  
    def kruskal(self):
        result = []
        i, e = 0, 0
 
        # Sorts the graph based on the weight key (item = [u,v,w] so item[2] is the weight)
        # The sorting is in ascending order so the first edge is of minimum weight
        self.graph = sorted(self.graph, key=lambda item: item[2])
        parent = []
        rank = []

        # Every vertice is initialized as parent of itself and with rank 0
        for node in range(self.V):
            parent.append(node)
            rank.append(0)

        while e < self.V - 1:
            u, v, w = self.graph[i]
            i = i + 1
            # Find the parent of u
            x = self.search(parent, u)

            # Find the parent of v
            y = self.search(parent, v)

            # If they have same parent means that the edge forms a cycle
            if x != y:
                e = e + 1
                result.append([u, v, w])
                self.apply_union(parent, rank, x, y)

        for u, v, weight in result:
            print("Edge:",u, v,end =" ")
            print("-",weight)

        return result
   
class GridMap(Graph):

    def __init__(self, data=[], width=0, height=0, resolution=0,
                 center_x=0, center_y=0, origin=[]):
        
        """ Parameters that define the class:
        :data: vector defining if each cell is either 0 (free) or 100 (occupied)
        :param width: number of grid for width
        :param height: number of grid for height
        :param resolution: grid resolution [m]
        :param center_x: center of the grid in drone coords x position [m]
        :param center_y: center of the grid in drone coords y position [m]
        :origin: position of the first cell in drone coords
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
    def area_definition(self, x, y):
        resolution = 2.0 # meters

        # calculate dimensions of the squared surface
        # x_max = int((max(x) - min(x))/resolution)
        # y_max = int((max(y) - min(y))/resolution)

        x_max = 3
        y_max = 3

        data = []
        cell_free = 0
        for i in range(y_max):
            for j in range(x_max):
                # for now all cells are free to cover
                data.append(cell_free)
        
        width = x_max
        height = y_max
        center_x = 0
        center_y = 0
        origin = [min(x),min(y)]
        # TO DO: valutare se serve di memorizzare anche per ogni cella che posizine su i,j è
        # insieme al numero di cella
        gridmap = GridMap(data, width, height, resolution, center_x, center_y, origin)

        return gridmap

    """ Divide the grid map area to cover with multiple CPS"""
    def area_division(self, grid_map, uav_pose):
    
        # convert swarm pose to grid position (i,j)
        # (i,j) are the coordinates of the position of the drone inside the grid
        print(grid_map.width, grid_map.height, grid_map.origin[:])
        i = (grid_map.width-1) - int((uav_pose[0] - grid_map.origin[0])/grid_map.resolution)
        j = (grid_map.height-1) - int((uav_pose[1] - grid_map.origin[1])/grid_map.resolution)

        return i, j
    
    # TO DO: retrieve the path to the drone as a sequence of waypoints using a service
    # Kruskal algorithm taken from https://www.pythonpool.com/kruskals-algorithm-python/
    def mst_generation(self):

        rows = self.width
        cols = self.height
        
        num_nodes = 9
        edges = Graph(num_nodes)

        # Define the nodes based on the middle point of each gridmap
        for i in range(rows) :
            for j in range(cols) :

                # Scanning the grid map as a matrix but stored in a vector
                # create all the edges that connect adjancent vertices
                if (self.data[i*cols+j] == 0):
                
                    if (i>0 and self.data[(i-1)*cols+j] == 0):
                        edges.add_edge(i*cols+j, (i-1)*cols+j, 1)
                
                    if (i<rows-1 and self.data[(i+1)*cols+j] == 0):
                        edges.add_edge(i*cols+j, (i+1)*cols+j, 1)
                
                    if (j>0 and self.data[i*cols+j-1] == 0):
                        edges.add_edge(i*cols+j, i*cols+j-1, 1)
                
                    if (j<cols-1 and self.data[i*cols+j+1] == 0):
                        edges.add_edge(i*cols+j, i*cols+j+1, 1)
    
        mst = edges.kruskal() 
        print(edges.graph[:])
        return mst         


def main(args=None):
    # rclpy.init(args=args)
    # print("Starting Offboard Mission...\n")
    # offboard_control = OffboardControl()
    # rclpy.spin(offboard_control)

    # Edges of the squared surface that needs to be covered
    # Defined in coordinates w.r.t gazebo center
    x = [1, 1, -1, -1]
    y = [1, -1, -1, 1]

    G = GridMap()
    G = G.area_definition(x,y)
    mst = G.mst_generation()

    # offboard_control.destroy_node()
    # rclpy.shutdown()
    # print("Clean exit")
    # exit(0)

if __name__ == '__main__':
    main()
    
    
