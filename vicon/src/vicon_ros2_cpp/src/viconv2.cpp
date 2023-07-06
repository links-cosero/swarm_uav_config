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


using namespace std::chrono_literals;
using namespace std::chrono;
using namespace ViconDataStreamSDK::CPP;
using std::placeholders::_1;

/* Hold informations about a single segment */
struct segment_info{
  std::string object_name;
  std::string seg_name;
  float px;
  float py;
  float pz;
  float qx;
  float qy;
  float qz;
  float qw;
};

/* Hold informations about all segments inside a single frame */
struct frame_info{
  bool valid=false;
  double latency_seconds;
  int frame_number;
  float frame_rate;
  std::vector<segment_info> segments;
};

class ViconRos2_v2 : public rclcpp::Node
{
public:
    ViconRos2_v2()
    : Node("vicon")
    {
    RCLCPP_INFO(get_logger(), "Connecting client to VICON ...");
		Output_Connect result = this->vicon_client.Connect(vicon_tracker_ip, 50);
    if (result.Result == Result::Success){
      RCLCPP_INFO(get_logger(), "Client successfully connected!");
    }else{
      RCLCPP_ERROR(get_logger(), "Client failed to connect!");
      return;
    }

    this->vicon_client.SetAxisMapping(
        Direction::Forward,
        Direction::Right,
        Direction::Down);

    this->vicon_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vicon/drone1", 10);

    this->vicon_thread = std::thread(bind(& ViconRos2_v2::vicon_rcv, this));

	}

	void vicon_rcv(){
    while(true){
      Output_WaitForFrame WaitOutput = vicon_client.WaitForFrame();
      if( WaitOutput.Result == Result::Success )
      {
        this->vicon_client.UpdateFrame();
        int millis_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        frame_info new_frame = this->get_position("drone1");
        if(! new_frame.segments.empty()){
          int diff = millis_since_epoch - this->last_rcv_time;
          RCLCPP_INFO(get_logger(), "Frame [%d] : x= %.4f y= %.4f z= %.4f  delta_time=%d ms",
            millis_since_epoch, new_frame.segments[0].px, new_frame.segments[0].py, new_frame.segments[0].pz, diff);
          this->last_rcv_time = millis_since_epoch;
          if(this->publish){
            this->ros_pub(new_frame);
          }
          this->publish = ! this->publish;
        }else{
          RCLCPP_WARN(get_logger(),"No segments :(");
        }

      }else{
        RCLCPP_INFO(get_logger(),"WaitOutput.Result error :(");
      }
    }
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

            result.push_back(new_seg);

          }
        }
    }else{
      RCLCPP_INFO(this->get_logger(), "Result = %d", seg_cnt.Result);
    }

    return result;
  }

private:
	RetimingClient vicon_client;
	std::string vicon_tracker_ip = "192.168.50.56";   // VICON local ip addr.
  int last_rcv_time = 0;
  std::thread vicon_thread;
  bool publish = 0;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViconRos2_v2>());
  rclcpp::shutdown();
  return 0;
}