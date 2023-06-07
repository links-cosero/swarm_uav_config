import rclpy
from rclpy.node import Node
from math import cos, sin, sqrt
import numpy as np

from pyvicon_datastream import tools
import pyvicon_datastream as pv


from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped

class ViconRos2(Node):

    def __init__(self):
        super().__init__('ViconRos2')
        self.vicon_tracker_ip = "192.168.50.56"
        self.declare_parameter("object_name", "drone1")
        self.object_name = self.get_parameter("object_name").get_parameter_value().string_value
        # VICON client setup
        self.vicon_tracker = tools.ObjectTracker(self.vicon_tracker_ip)
        self.vicon_tracker.vicon_client.set_stream_mode(pv.StreamMode.ClientPull)
        self.vicon_tracker.vicon_client.set_axis_mapping(
				pv.Direction.Forward,
				pv.Direction.Right,
				pv.Direction.Down)
        

        # -------- PUBLISHER --------- #
        self.px4_odometry_pub = self.create_publisher(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", 10)
        self.vicon_pose_pub =     self.create_publisher(PoseStamped, "px4_visualizer/" + self.object_name+"_vicon_pose", 10)

        # -------- TIMERS ------------ #
        data_timer = self.create_timer(1/40.0, self.data_timer_cb)


    def data_timer_cb(self):
        """Get frame from VICON and publish odometry to PX4"""
        position = self.vicon_tracker.get_position(self.object_name)
        if not position:
            return
        
        latency, frame_num, object_pos = position

        for item in object_pos: # THis list can be empty if object is not in VICON range
            subject_name, segment_name, px, py, pz, rx, ry, rz = tuple(item)
            if subject_name == self.object_name:
                rot_matrix = self.create_rot_matrix(rx, ry, rz)

                now = int(self.get_clock().now().nanoseconds / 1000)
                msg = self.generate_odom_msg(
                    [float(px)/1000, float(py)/1000, float(pz)/1000],
                    self.get_quaternion_from_mat(rot_matrix),
                    now,
                    now - int(latency/1E-6)
                )
                # Publish to PX4 vehicle_visual_odometry
                self.px4_odometry_pub.publish(msg)
                # Publish for RVIZ
                vicon_stamped = self.generate_pose_stamped(
                    px, 
                    py, 
                    pz, 
                    self.get_quaternion_from_mat(rot_matrix))
                self.vicon_pose_pub.publish(vicon_stamped)
                return
        
    def create_rot_matrix(self, x : float, y : float, z : float):
        """Create rotation matrix as described in VICON manual"""
        return [
            [cos(y)*cos(z),                     -cos(y)*sin(z),                         sin(y)],
            [cos(x)*sin(z)+sin(x)*sin(y)*cos(z), cos(x)*cos(z)-sin(x)*sin(y)*sin(z),    -sin(x)*cos(y)],
            [sin(x)*sin(z)-cos(x)*sin(y)*cos(z), sin(x)*cos(z)+cos(x)*sin(y)*sin(z),    cos(x)*cos(y)]
        ]

    def get_quaternion_from_mat(self, m):
        """Extract quaternion from a rotation matrix"""
        # w = sqrt(1 + m[0][0] + m[1][1] + m[2][2]) / 2.0
        # x = (m[2][1] - m[1][2]) / (4*w)
        # y = (m[0][2] - m[2][0]) / (4*w)
        # z = (m[1][0] - m[0][1]) / (4*w)

        # Di seguito trasformazione da matrice di rotazione a quaternione non singolare
        # Preso dalle slide del corso di Modelling
        w = sqrt(1 + m[0][0] + m[1][1] + m[2][2]) / 2.0
        x = np.sign(m[2][1] - m[1][2])*sqrt(1 + m[0][0] - m[1][1] - m[2][2]) / 2.0
        y = np.sign(m[0][2] - m[2][0])*sqrt(1 - m[0][0] + m[1][1] - m[2][2]) / 2.0
        z = np.sign(m[1][0] - m[0][1])*sqrt(1 - m[0][0] - m[1][1] + m[2][2]) / 2.0

        return [float(w), float(x), float(y), float(z)]
    
    def generate_pose_stamped(self, x,y,z,q):
        vehicle_pose_msg = PoseStamped()
        vehicle_pose_msg.header.frame_id='map'
        # Position NED -> FLU
        vehicle_pose_msg.pose.position.x =  x / 1000
        vehicle_pose_msg.pose.position.y = -y / 1000
        vehicle_pose_msg.pose.position.z = -z / 1000
        # Orientation
        vehicle_pose_msg.pose.orientation.w =  float(q[0])
        vehicle_pose_msg.pose.orientation.x =  float(q[1])
        vehicle_pose_msg.pose.orientation.y = -float(q[2])
        vehicle_pose_msg.pose.orientation.z = -float(q[3])
        return vehicle_pose_msg
    
    def generate_odom_msg(self, pos:list, q:list, timestamp:int, timestamp_sample:int) -> VehicleOdometry:
        """Return a VehicleOdometry message"""
        odom_msg = VehicleOdometry()
        odom_msg.timestamp = timestamp
        odom_msg.timestamp_sample = timestamp_sample
        odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        odom_msg.position = pos
        odom_msg.q = q
        odom_msg.velocity =         [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.angular_velocity = [float('NaN'), float('Nan'), float('Nan')]
        
        return odom_msg

            

def main(args=None):
    rclpy.init(args=args)
    vicon_ros = ViconRos2()
    rclpy.spin(vicon_ros)
    vicon_ros.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
