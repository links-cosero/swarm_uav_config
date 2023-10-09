#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

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
from visualization_msgs.msg import Marker

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
        
        #SUBSCRIPTION FIRST DRONE
        self.attitude1_sub = self.create_subscription(
            VehicleAttitude,
            '/drone_1/fmu/out/vehicle_attitude',
            self.vehicle1_attitude_callback,
            qos_profile)
        self.local_position1_sub = self.create_subscription(
            VehicleLocalPosition,
            '/drone_1/fmu/out/vehicle_local_position',
            self.vehicle1_local_position_callback,
            qos_profile)
        self.setpoint1_sub = self.create_subscription(
            TrajectorySetpoint,
            '/drone_1/fmu/in/trajectory_setpoint',
            self.trajectory1_setpoint_callback,
            qos_profile)
            
	    # SUBSCRIPTIONS SECOND DRONE
        self.attitude2_sub = self.create_subscription(
            VehicleAttitude,
            '/drone_2/fmu/out/vehicle_attitude',
            self.vehicle2_attitude_callback,
            qos_profile)
        self.local_position2_sub = self.create_subscription(
            VehicleLocalPosition,
            '/drone_2/fmu/out/vehicle_local_position',
            self.vehicle2_local_position_callback,
            qos_profile)
        self.setpoint2_sub = self.create_subscription(
            TrajectorySetpoint,
            '/drone_2/fmu/in/trajectory_setpoint',
            self.trajectory2_setpoint_callback,
            qos_profile)

	    # FIRST VEHICLE PUBLISHERS
        self.vehicle1_pose_pub = self.create_publisher(PoseStamped, '/px4_visualizer1/vehicle_pose', 10)
        self.vehicle1_vel_pub = self.create_publisher(Marker, '/px4_visualizer1/vehicle_velocity', 10)
        self.vehicle1_path_pub = self.create_publisher(Path, '/px4_visualizer1/vehicle_path', 10)
        self.setpoint1_path_pub = self.create_publisher(Path, '/px4_visualizer1/setpoint_path', 10)

	    # SECOND VEHICLE PUBLISHERS
        self.vehicle2_pose_pub = self.create_publisher(PoseStamped, '/px4_visualizer2/vehicle_pose', 10)
        self.vehicle2_vel_pub = self.create_publisher(Marker, '/px4_visualizer2/vehicle_velocity', 10)
        self.vehicle2_path_pub = self.create_publisher(Path, '/px4_visualizer2/vehicle_path', 10)
        self.setpoint2_path_pub = self.create_publisher(Path, '/px4_visualizer2/setpoint_path', 10)

        self.vehicle1_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle1_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle1_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint1_position = np.array([0.0, 0.0, 0.0])
        self.vehicle1_path_msg = Path()
        self.setpoint1_path_msg = Path()
        
        self.vehicle2_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle2_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle2_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint2_position = np.array([0.0, 0.0, 0.0])
        self.vehicle2_path_msg = Path()
        self.setpoint2_path_msg = Path()
       
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
    
    def FRD2FLU_vector_converter(self, position):

        # Rotation around X-axis of 180° Degrees
        rotX = np.asfarray(np.array([[1, 0, 0],
                                    [0, np.cos(np.pi), -np.sin(np.pi)], 
                                    [0, np.sin(np.pi), np.cos(np.pi)]]), dtype = np.float32)
        position = np.dot(rotX, position)

        return position

    def vehicle1_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle1_attitude[0] = msg.q[0]
        self.vehicle1_attitude[1] = msg.q[1]
        self.vehicle1_attitude[2] = -msg.q[2]
        self.vehicle1_attitude[3] = -msg.q[3]

    def vehicle1_local_position_callback(self, msg):

        self.vehicle1_local_position[0] = msg.x
        self.vehicle1_local_position[1] = msg.y
        self.vehicle1_local_position[2] = msg.z
        self.vehicle1_local_velocity[0] = msg.vx
        self.vehicle1_local_velocity[1] = -msg.vy
        self.vehicle1_local_velocity[2] = -msg.vz

        self.vehicle1_local_position = self.FRD2FLU_vector_converter(self.vehicle1_local_position)

    def trajectory1_setpoint_callback(self, msg):
        self.setpoint1_position[0] = msg.position[0]
        self.setpoint1_position[1] = msg.position[1]
        self.setpoint1_position[2] = msg.position[2]

        self.setpoint1_position = self.FRD2FLU_vector_converter(self.setpoint1_position)

        
    def vehicle2_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle2_attitude[0] = msg.q[0]
        self.vehicle2_attitude[1] = msg.q[1]
        self.vehicle2_attitude[2] = -msg.q[2]
        self.vehicle2_attitude[3] = -msg.q[3]


    def vehicle2_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle2_local_position[0] = msg.x
        self.vehicle2_local_position[1] = msg.y
        self.vehicle2_local_position[2] = msg.z
        self.vehicle2_local_velocity[0] = msg.vx
        self.vehicle2_local_velocity[1] = -msg.vy
        self.vehicle2_local_velocity[2] = -msg.vz

        self.vehicle2_local_position = self.FRD2FLU_vector_converter(self.vehicle2_local_position)


    def trajectory2_setpoint_callback(self, msg):
        self.setpoint1_position[0] = msg.position[0]
        self.setpoint1_position[1] = msg.position[1]
        self.setpoint1_position[2] = msg.position[2]

        self.setpoint2_position = self.FRD2FLU_vector_converter(self.setpoint2_position)


    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = 'map'
        # msg.header.stamp = Clock().now().nanoseconds / 1000
        msg.ns = 'arrow'
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        dt = 0.3
        tail_point = Point()
        tail_point.x = tail[0]
        tail_point.y = tail[1]
        tail_point.z = tail[2]
        head_point = Point()
        head_point.x = tail[0] + dt * vector[0]
        head_point.y = tail[1] + dt * vector[1]
        head_point.z = tail[2] + dt * vector[2]
        msg.points = [tail_point, head_point]
        return msg

    def cmdloop_callback(self):
        vehicle1_pose_msg = vector2PoseMsg('map', self.vehicle1_local_position, self.vehicle1_attitude)
        vehicle2_pose_msg = vector2PoseMsg('map', self.vehicle2_local_position, self.vehicle2_attitude)
        self.vehicle1_pose_pub.publish(vehicle1_pose_msg)
        self.vehicle2_pose_pub.publish(vehicle2_pose_msg)

        # Publish time history of the two vehicle paths
        self.vehicle1_path_msg.header = vehicle1_pose_msg.header
        self.vehicle1_path_msg.poses.append(vehicle1_pose_msg) 
        self.vehicle1_path_pub.publish(self.vehicle1_path_msg)

        self.vehicle2_path_msg.header = vehicle2_pose_msg.header
        self.vehicle2_path_msg.poses.append(vehicle2_pose_msg) 
        self.vehicle2_path_pub.publish(self.vehicle2_path_msg)

        # Publish time history of the two vehicle paths
        setpoint1_pose_msg = vector2PoseMsg('map', self.setpoint1_position, self.vehicle1_attitude)
        self.setpoint1_path_msg.header = setpoint1_pose_msg.header
        self.setpoint1_path_msg.poses.append(setpoint1_pose_msg)
        self.setpoint1_path_pub.publish(self.setpoint1_path_msg)

        setpoint2_pose_msg = vector2PoseMsg('map', self.setpoint2_position, self.vehicle2_attitude)
        self.setpoint2_path_msg.header = setpoint2_pose_msg.header
        self.setpoint2_path_msg.poses.append(setpoint2_pose_msg)
        self.setpoint2_path_pub.publish(self.setpoint2_path_msg)

        # Publish arrow markers for velocity
        velocity1_msg = self.create_arrow_marker(1, self.vehicle1_local_position, self.vehicle1_local_velocity)
        self.vehicle1_vel_pub.publish(velocity1_msg)

        velocity2_msg = self.create_arrow_marker(1, self.vehicle2_local_position, self.vehicle2_local_velocity)
        self.vehicle2_vel_pub.publish(velocity2_msg)

def main(args=None):
    rclpy.init(args=args)

    px4_visualizer = PX4Visualizer()

    rclpy.spin(px4_visualizer)

    px4_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()