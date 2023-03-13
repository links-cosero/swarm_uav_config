import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy)

from nav_msgs.msg import Path
from geometry_msgs.msg import (
    PoseArray, 
    Pose,
    PoseStamped)
from px4_msgs.msg import (
    VehicleOdometry,
    VehicleLocalPosition,
    VehicleAttitude)

import subprocess
from math import sqrt
import numpy as np



class MocapNode (Node):
    def __init__(self) -> None:
        super().__init__('mocap_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        # Subscribers
        self.subscriber_ =      self.create_subscription(PoseArray, '/world/default/pose/info', self.listener_cb, 10)
        # self.px4_odom_sub =     self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, qos_profile) # DEBUG
        self.attitude_sub =     self.create_subscription(VehicleAttitude,'/fmu/out/vehicle_attitude',self.attitude_cb,qos_profile)
        # Publishers
        self.pose_pub =         self.create_publisher(PoseStamped,      "/x500/pose", 10)
        # self.ned_pose_pub =     self.create_publisher(PoseStamped,      "/x500/ned_pose", 10)     # DEBUG               
        # self.ned_px4_pose_pub = self.create_publisher(PoseStamped,      "/x500/ned_px4_pose", 10) # DEBUG           
        self.path_pub =         self.create_publisher(Path,             "/x500/path", 10)
        self.mocap_odom_pub =   self.create_publisher(VehicleOdometry,  "/fmu/in/vehicle_visual_odometry", 10)
        # Messages
        self.vehicle_path_msg = Path()
        self.vehicle_path_msg.header.frame_id = 'map'
        self.vehicle_pose_msg = PoseStamped()
        self.gz_pose_msg = Pose()
        self.mocap_odom_msg = None
        self.attitude_msg = None
        self.timestamp = 0
        self.timestamp_sample = 0

        self.path_cnt = 0
        self.launch_ros_gz_bridge("/world/default/pose/info","geometry_msgs/msg/PoseArray","gz.msgs.Pose_V")

    def listener_cb(self, msg: PoseArray):
        # Nell'array di oggetti Pose il secondo è quello del drone a cui siamo interessati. L'ho
        # trovato a tentativi guardando i numeri più convincenti e sono sicuro sia quello perchè 
        # coincide con la visuale nella simulazione. E' ripetibile tutte le simulazioni e funziona sempre
        # ma penso valga solo per la specifica simulazione 'make px4_sitl gz_x500'. 
        self.gz_pose_msg : Pose = msg.poses[1]

        # Change ref fram to FLU and publish msg
        self.vehicle_pose_msg = self.gazebo_to_flu_tf(self.gz_pose_msg)
        self.pose_pub.publish(self.vehicle_pose_msg)
        # Publish path msg
        if self.path_cnt == 0:
            self.vehicle_path_msg.poses.append(self.vehicle_pose_msg)
            self.path_pub.publish(self.vehicle_path_msg)
        self.path_cnt = (self.path_cnt + 1) % 10
        # Create mocap odometry msg and publish to PX4
        self.mocap_odom_msg = self.create_odometry_msg(self.vehicle_pose_msg.pose)
        self.mocap_odom_pub.publish(self.mocap_odom_msg)

        # ned_pos = PoseStamped()
        # ned_pos.header.frame_id = 'map'
        # ned_pos.pose.position.x = float(self.mocap_odom_msg.position[0])
        # ned_pos.pose.position.y = float(self.mocap_odom_msg.position[1])
        # ned_pos.pose.position.z = float(self.mocap_odom_msg.position[2])
        # ned_pos.pose.orientation.x = float(self.mocap_odom_msg.q[0])
        # ned_pos.pose.orientation.y = float(self.mocap_odom_msg.q[1])
        # ned_pos.pose.orientation.z = float(self.mocap_odom_msg.q[2])
        # ned_pos.pose.orientation.w = float(self.mocap_odom_msg.q[3])        
        # self.ned_pose_pub.publish(ned_pos)

    def launch_ros_gz_bridge(self, topic:str, ros_type:str, gz_type:str):
        """ Launch subprocess to bridge Gazebo and ROS2 """

        argument = f"{topic}@{ros_type}[{gz_type}"
        command = "ros2 run ros_gz_bridge parameter_bridge "+argument
        subprocess.Popen(command.split(' '))


    def gazebo_to_flu_tf(self, gz_pose: Pose) -> PoseStamped:
        """
        Transform from gazebo Pose to FLU pose for ROS2. Is done doing
        a rotation of -90° arounf z axis (global frame). 
        """
        vehicle_pose_msg = PoseStamped()
        vehicle_pose_msg.header.frame_id='map'
        # Position
        vehicle_pose_msg.pose.position.x =  gz_pose.position.y
        vehicle_pose_msg.pose.position.y = -gz_pose.position.x
        vehicle_pose_msg.pose.position.z =  gz_pose.position.z
        # Orientation
        x_gz = gz_pose.orientation.x
        y_gz = gz_pose.orientation.y
        z_gz = gz_pose.orientation.z
        w_gz = gz_pose.orientation.w
        vehicle_pose_msg.pose.orientation.x = (- x_gz - y_gz) * sqrt(2)/2
        vehicle_pose_msg.pose.orientation.y = (- y_gz + x_gz) * sqrt(2)/2
        vehicle_pose_msg.pose.orientation.z = (- z_gz + w_gz) * sqrt(2)/2
        vehicle_pose_msg.pose.orientation.w = (- w_gz - z_gz) * sqrt(2)/2
        return vehicle_pose_msg
    

    def flu_to_ned_tf(self, pose : Pose) -> Pose:
        """
        Transform a FLU frame into NED => Rot_x(180°)
        Rotate quaternion: (i)*(xi+yj+zk+w) = -x +yk -zj +wi
        (xi+yj+zk+w)
        """
        pose_quat = (pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
        rot_quat = (.5, .5, .5, .5)
        result = self.quaternion_multiply(pose_quat, rot_quat)

        ned_pose = Pose()
        ned_pose.position.x =  pose.position.x
        ned_pose.position.y = -pose.position.y
        ned_pose.position.z = -pose.position.z
        ned_pose.orientation.x =  result[0]
        ned_pose.orientation.y =  result[1]
        ned_pose.orientation.z =  result[2]
        ned_pose.orientation.w =  result[3]
        return ned_pose

    
    def create_odometry_msg(self, pose : Pose) -> VehicleOdometry:
        """
        Create VehicleOdometry message. Not include velocities
        """
        odom_msg = VehicleOdometry()
        odom_msg.timestamp =  self.timestamp
        odom_msg.timestamp_sample = self.timestamp_sample
        odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        ned_pose = self.flu_to_ned_tf(pose)
        odom_msg.position = [ned_pose.position.x, 
                            ned_pose.position.y, 
                            ned_pose.position.z]
        # odom_msg.q =        [ned_pose.orientation.x, # FIXME: transformation not working
        #                     ned_pose.orientation.y,                      
        #                     ned_pose.orientation.z,
        #                     ned_pose.orientation.w] 
        odom_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        odom_msg.velocity =         [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.angular_velocity = [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.position_variance =    [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.orientation_variance = [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.velocity_variance =    [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.quality = 1
        return odom_msg
    
    def local_pos_cb(self, msg : VehicleLocalPosition):
        """
        Remap the VehicleLocalPosition to PoseStamped to visualize in Rviz2
        """
        if self.attitude_msg != None:
            new_pose = PoseStamped()
            new_pose.header.frame_id = 'map'
            new_pose.pose.position.x = msg.x
            new_pose.pose.position.y = msg.y
            new_pose.pose.position.z = msg.z
            # FIXME: Rotation transformation not working
            new_pose.pose.orientation.x = float(self.attitude_msg.q[0])            
            new_pose.pose.orientation.y = float(self.attitude_msg.q[1])
            new_pose.pose.orientation.z = float(self.attitude_msg.q[2])
            new_pose.pose.orientation.w = float(self.attitude_msg.q[3])
            self.ned_px4_pose_pub.publish(new_pose)        

    def attitude_cb(self, msg : VehicleAttitude):
        self.attitude_msg = msg
        # Store PX4 timestamp
        self.timestamp = msg.timestamp
        self.timestamp_sample = msg.timestamp_sample

    def quaternion_multiply(self, quaternion1, quaternion0) -> np.ndarray:
        """
        Multiply two quaternions in the form (x, y, z, w)
        """
        x0, y0, z0, w0 = quaternion0
        x1, y1, z1, w1 = quaternion1
        return np.array([  x0*w1 + y0*z1 - z0*y1 + w0*x1,
                         - x0*z1 + y0*w1 + z0*x1 + w0*y1,
                           x0*y1 - y0*x1 + z0*w1 + w0*z1,
                         - x0*x1 - y0*y1 - z0*z1 + w0*w1], dtype=np.float64)

def main(args=None):
    rclpy.init(args=args)
    sub = MocapNode()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
