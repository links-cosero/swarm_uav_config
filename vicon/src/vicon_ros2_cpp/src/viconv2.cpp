#include <cstdio>
#include <memory>
#include <thread>
#include <chrono>
#include <thread>

#include "DataStreamRetimingClient.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace ViconDataStreamSDK::CPP;
using std::placeholders::_1;

/* Hold informations about a single segment */
struct segment_info{
  std::string object_name;
  std::string seg_name;
  float px = 0;
  float py = 0;
  float pz = 0;
  float qx = 0;
  float qy = 0;
  float qz = 0;
  float qw = 0;
};

/* Hold informations about all segments inside a single frame */
struct frame_info{
  bool valid=false;
  double latency_seconds;
  int frame_number;
  float frame_rate;
  std::vector<segment_info> segments;
};

rclcpp::QoS pub_qos = rclcpp::QoS(0)
		.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
		.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
		.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  
rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

class ViconRos2_v2 : public rclcpp::Node
{
public:
    ViconRos2_v2()
    : Node("vicon")
    {
    RCLCPP_INFO(get_logger(), "Connecting client to VICON ...");
		Output_Connect result = this->vicon_client.Connect(vicon_tracker_ip, 30);
    if (result.Result == Result::Success){
      RCLCPP_INFO(get_logger(), "Client successfully connected!");
    }else{
      RCLCPP_ERROR(get_logger(), "Client failed to connect!");
      throw -1;
    }

    this->vicon_client.SetAxisMapping(
        Direction::Forward,
        Direction::Right,
        Direction::Down);

    this->vicon_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vicon/drone1", sensor_qos);
    this->px4_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", sensor_qos);

    // this->vicon_thread = std::thread(bind(& ViconRos2_v2::vicon_rcv, this));
    this->timer_20ms = create_wall_timer(20ms, std::bind(& ViconRos2_v2::vicon_rcv, this));

    /*TF2 setup*/
    this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	}

	void vicon_rcv(){
    Output_WaitForFrame WaitOutput = vicon_client.WaitForFrame();
    if( WaitOutput.Result == Result::Success )
    {
      this->vicon_client.UpdateFrame();
      int millis_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
      frame_info new_frame = this->get_position("drone1");
      if(! new_frame.segments.empty()){
        int diff = millis_since_epoch - this->last_rcv_time;
        if (diff > 65){
          RCLCPP_WARN(get_logger(), "Inter frame time = %d", diff);
        }
        RCLCPP_INFO(get_logger(), "Frame [%d] : x= %.4f y= %.4f z= %.4f  delta_time=%d ms buf_len=%ld",
          millis_since_epoch, new_frame.segments[0].px, new_frame.segments[0].py, new_frame.segments[0].pz, diff, filter_buffer.size());
        this->last_rcv_time = millis_since_epoch;
        this->ros_pub(new_frame);
        this->publish_px4_msg(new_frame.segments[0]);
        this->broadcast_drone_tf(new_frame.segments[0]);
        this->broadcast_camera_tf();
      }else{
        // RCLCPP_WARN(get_logger(),"No segments :(");
        this->vicon_rcv();
      }

    }else{
      RCLCPP_INFO(get_logger(),"WaitOutput.Result error :(");
    }
	}

  void broadcast_drone_tf(segment_info segment){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "drone1";
    t.transform.translation.x =  segment.px / 1000;
    t.transform.translation.y = -segment.py / 1000;
    t.transform.translation.z = -segment.pz / 1000;
    t.transform.rotation.x =  segment.qx;
    t.transform.rotation.y = -segment.qy;
    t.transform.rotation.z = -segment.qz;
    t.transform.rotation.w =  segment.qw;
    this->tf_broadcaster->sendTransform(t);
  }
  void broadcast_camera_tf(){
		geometry_msgs::msg::TransformStamped t;
		tf2::Quaternion q;
		q.setEuler(3.1415, 3.1415, 0.0);
		t.header.stamp = this->get_clock()->now();
		t.header.frame_id = "drone1";
		t.child_frame_id = "camera";
		t.transform.translation.x = 0.07;
		t.transform.translation.y = 0.0;
		t.transform.translation.z = 0.0;
		t.transform.rotation.x = q.x();
		t.transform.rotation.y = q.y();
		t.transform.rotation.z = q.z();
		t.transform.rotation.w = q.w();
		this->tf_broadcaster->sendTransform(t);
	}

  frame_info get_position(std::string obj_name){
    frame_info new_frame;
    /*Get all objects*/
    Output_GetSubjectCount sub_cnt = this->vicon_client.GetSubjectCount();
    
    for(int i = 0; i<int(sub_cnt.SubjectCount); i++){
      Output_GetSubjectName name = this->vicon_client.GetSubjectName(i);
      
      /*get all segments for each object*/
      if(std::string(name.SubjectName) == obj_name){      
        std::vector<segment_info> segments = this->get_segments_info(obj_name);
        new_frame.segments = segments;
        new_frame.valid = true;
      }
    }
    return new_frame;

  }
  
  void ros_pub(frame_info frame){
    geometry_msgs::msg::Point translation;
        geometry_msgs::msg::Quaternion q;

        if((frame.segments.size() != 0)){
          // send only the info about the first segment on the list
          translation.x = frame.segments[0].px / 1000;
          translation.y = frame.segments[0].py / 1000;
          translation.z = frame.segments[0].pz / 1000;
          q.x = frame.segments[0].qx;        
          q.y = frame.segments[0].qy;
          q.z = frame.segments[0].qz;
          q.w = frame.segments[0].qw;        
          
          geometry_msgs::msg::PoseStamped last_pose;
          last_pose.header.frame_id = "map";
          last_pose.header.stamp = this->get_clock()->now();
          last_pose.pose.position = translation;
          last_pose.pose.orientation = q;

          this->vicon_pub_->publish(last_pose);
        }
  }

  std::vector<segment_info> get_segments_info(std::string obj_name){
    std::vector<segment_info> result;
    Output_GetSegmentCount seg_cnt = this->vicon_client.GetSegmentCount(obj_name.c_str());
    if (1){ //(seg_cnt.Result == Result::Success){
        for(int j = 0; j<int(seg_cnt.SegmentCount); j++){
          Output_GetSegmentName seg_name = this-> vicon_client.GetSegmentName(obj_name, j);
          Output_GetSegmentGlobalTranslation seg_trasl = this->vicon_client.GetSegmentGlobalTranslation(obj_name, seg_name.SegmentName);
          Output_GetSegmentGlobalRotationQuaternion seg_quat = this-> vicon_client.GetSegmentGlobalRotationQuaternion(obj_name, seg_name.SegmentName);

          if(!seg_trasl.Occluded){
            segment_info new_seg;
            new_seg.object_name = obj_name;
            new_seg.seg_name = seg_name.SegmentName;
            new_seg.px = seg_trasl.Translation[0]; 
            new_seg.py = seg_trasl.Translation[1];
            new_seg.pz = seg_trasl.Translation[2];
            new_seg.qx = seg_quat.Rotation[0];
            new_seg.qy = seg_quat.Rotation[1];
            new_seg.qz = seg_quat.Rotation[2];
            new_seg.qw = seg_quat.Rotation[3];

            // segment_info filtered_segment = this->filter_segment_data(new_seg);
            // result.push_back(filtered_segment);            
            result.push_back(new_seg);            

          }
        }
    }else{
      RCLCPP_INFO(this->get_logger(), "Result = %d", seg_cnt.Result);
    }

    return result;
  }

  segment_info filter_segment_data(segment_info new_segment){
    // Keep buffer size limited
    this->filter_buffer.push_back(new_segment);
    while(filter_buffer.size() > 10){this->filter_buffer.pop_front();}
    
    segment_info average_seg = this->compute_buffer_average();
    float x_diff = abs(new_segment.px - average_seg.px);
    return average_seg;
    if (x_diff > 30.0){
      this->filter_buffer.pop_back();
      return average_seg;
    } else{
      return new_segment;
    }
  }

  segment_info compute_buffer_average(){
    segment_info average_seg;
    int n_samples = this->filter_buffer.size();
    for(auto sample : this->filter_buffer){
      average_seg.px += sample.px;
      average_seg.py += sample.py;
      average_seg.pz += sample.pz;
      average_seg.qx += sample.qx;
      average_seg.qy += sample.qy;
      average_seg.qz += sample.qz;
      average_seg.qw += sample.qw;
    }
    average_seg.px = average_seg.px / float(n_samples);
    average_seg.py = average_seg.py / float(n_samples);
    average_seg.pz = average_seg.pz / float(n_samples);
    average_seg.qx = average_seg.qx / float(n_samples);
    average_seg.qy = average_seg.qy / float(n_samples);
    average_seg.qz = average_seg.qz / float(n_samples);
    average_seg.qw = average_seg.qw / float(n_samples);
    return average_seg;
  }

  void publish_px4_msg(segment_info segment){
    px4_msgs::msg::VehicleOdometry new_msg;
    new_msg.timestamp = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    new_msg.timestamp_sample = new_msg.timestamp;
    new_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    new_msg.position = {
      segment.px / float(1000),        
      segment.py / float(1000),        
      segment.pz / float(1000)          
    };
    new_msg.q = {
      segment.qw,      
      segment.qx,      
      segment.qy,      
      segment.qz        
    };
    new_msg.position_variance = {0.001, 0.001, 0.001};
    this->px4_pub_->publish(new_msg);
  }

private:
	RetimingClient vicon_client;
	std::string vicon_tracker_ip = "192.168.50.56";   // VICON local ip addr.
  int last_rcv_time = 0;
  std::thread vicon_thread;
  bool publish = 0;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
  std::list<segment_info> filter_buffer;
  rclcpp::TimerBase::SharedPtr timer_20ms;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try{
    rclcpp::spin(std::make_shared<ViconRos2_v2>());
  }catch (int var){

  };
  rclcpp::shutdown();
  return 0;
}