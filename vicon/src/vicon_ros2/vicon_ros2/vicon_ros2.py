import rclpy
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy import utilities
from math import cos, sin, sqrt

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
        self.vicon_tracker.vicon_client.set_axis_mapping(
				pv.Direction.Forward,
				pv.Direction.Right,
				pv.Direction.Down)
        

        # -------- PUBLISHER ---------
        self.odometry_pub = self.create_publisher(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", 10)
        self.pose_pub =         self.create_publisher(PoseStamped,      '/'+self.object_name+"/pose", 10)

        while(True):
            try:
                position = self.vicon_tracker.get_position(self.object_name)
                latency, frame_num, object_pos = position
                subject_name, segment_name, px, py, pz, rx, ry, rz = tuple(object_pos[0])

                rot_matrix = self.create_rot_matrix(rx, ry, rz)

                now = int(self.get_clock().now().nanoseconds / 1000)
                odom_msg = VehicleOdometry()
                odom_msg.timestamp = now
                # FIXME: fare che sia il vero momento di campionamento
                odom_msg.timestamp_sample = now
                odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD
                odom_msg.position = [float(px)/1000, float(py)/1000, float(pz)/1000]
                odom_msg.q = self.get_quaternion_from_mat(rot_matrix)
                odom_msg.velocity =         [float('NaN'), float('Nan'), float('Nan')]
                odom_msg.angular_velocity = [float('NaN'), float('Nan'), float('Nan')]

                self.odometry_pub.publish(odom_msg)

                self.pose_pub.publish(self.generate_pose_stamped(px, py, pz, odom_msg.q))

            except Exception as e:
                print(str(e))
        
    def create_rot_matrix(self, x : float, y : float, z : float):
        return [
            [cos(y)*cos(z),                     -cos(y)*sin(z),                         sin(y)],
            [cos(x)*sin(z)+sin(x)*sin(y)*cos(z), cos(x)*cos(z)-sin(x)*sin(y)*sin(z),    -sin(x)*cos(y)],
            [sin(x)*sin(z)-cos(x)*sin(y)*cos(z), sin(x)*cos(z)+cos(x)*sin(y)*sin(z),    cos(x)*cos(y)]
        ]

    def get_quaternion_from_mat(self, m):
        w = sqrt(1 + m[0][0] + m[1][1] + m[2][2]) / 2.0
        x = (m[2][1] - m[1][2]) / (4*w)
        y = (m[0][2] - m[2][0]) / (4*w)
        z = (m[1][0] - m[0][1]) / (4*w)

        return [float(w), float(x), float(y), float(z)]
    
    def generate_pose_stamped(self, x,y,z,q):
        vehicle_pose_msg = PoseStamped()
        vehicle_pose_msg.header.frame_id='map'
        # Position
        vehicle_pose_msg.pose.position.x = x / 1000
        vehicle_pose_msg.pose.position.y = y / 1000
        vehicle_pose_msg.pose.position.z = z / 1000
        # Orientation
        vehicle_pose_msg.pose.orientation.w = float(q[0])
        vehicle_pose_msg.pose.orientation.x = float(q[1])
        vehicle_pose_msg.pose.orientation.y = float(q[2])
        vehicle_pose_msg.pose.orientation.z = float(q[3])
        return vehicle_pose_msg
            

def main(args=None):
    rclpy.init(args=args)
    vicon_ros = ViconRos2()
    rclpy.spin(vicon_ros)
    vicon_ros.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
