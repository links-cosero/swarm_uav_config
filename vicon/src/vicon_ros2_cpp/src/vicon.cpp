#include <cstdio>
#include <memory>

#include "DataStreamClient.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/timesync_status.hpp"

using namespace std::chrono_literals;
using namespace ViconDataStreamSDK::CPP;
using std::placeholders::_1;

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

struct frame_info{
  bool valid=false;
  double latency_seconds;
  int frame_number;
  float frame_rate;
  std::vector<segment_info> segments;
};


class ViconRos2 : public rclcpp::Node
{
  public:
    ViconRos2()
    : Node("vicon")
    {
      this->start_time = this->now();
      /*ViconSDK client init*/
      this->vicon_client.Connect(this->vicon_tracker_ip);
      this->vicon_client.EnableSegmentData();
      this->vicon_client.SetStreamMode(StreamMode::ClientPull);
      this->vicon_client.SetAxisMapping(
        Direction::Forward,
        Direction::Right,
        Direction::Down);

      /*--- SUBSCRIPTIONS ---*/
      this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", 10, 
        std::bind(&ViconRos2::timesync_cb, this, _1));

      /*--- PUBLISHERS ---- */
      this->vicon_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vicon/pose", 10);
      this->px4_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry",10);

      timer_ = this->create_wall_timer(
        10ms, std::bind(&ViconRos2::timer_callback, this)); //100 Hz
    }

  private:
    void timesync_cb(const px4_msgs::msg::TimesyncStatus & msg){
      this->px4_timestamp = msg.timestamp;
    }

    void timer_callback()
    {
      geometry_msgs::msg::PoseStamped new_pose;
      geometry_msgs::msg::Point translation;
      geometry_msgs::msg::Quaternion q;
      frame_info new_frame = this->get_position("drone1");

      if((new_frame.segments.size() != 0) && new_frame.valid){
        // send only the info about the first segment on the list
        translation.x = new_frame.segments[0].px / 1000;
        translation.y = new_frame.segments[0].py / 1000;
        translation.z = new_frame.segments[0].pz / 1000;
        q.x = new_frame.segments[0].qx;        
        q.y = new_frame.segments[0].qy;
        q.z = new_frame.segments[0].qz;
        q.w = new_frame.segments[0].qw;        
        
        new_pose.header.frame_id = "map";
        new_pose.header.stamp = this->get_clock()->now();
        new_pose.pose.position = translation;
        new_pose.pose.orientation = q;

        this->vicon_pub_->publish(new_pose); //100 Hz
        px4_msgs::msg::VehicleOdometry px4_msg = this->create_odom_msg(new_pose, new_frame.latency_seconds);
        this->px4_pub_->publish(px4_msg); //100 Hz
      }


    }

    px4_msgs::msg::VehicleOdometry create_odom_msg(geometry_msgs::msg::PoseStamped pose, float latency){
      px4_msgs::msg::VehicleOdometry new_msg;
      new_msg.timestamp = this->px4_timestamp; //this->get_timestamp();
      new_msg.timestamp_sample = this->px4_timestamp;
      new_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
      new_msg.position = {
        float(pose.pose.position.x),
        float(pose.pose.position.y),
        float(pose.pose.position.z)
      };
      new_msg.q = {
        float(pose.pose.orientation.w),
        float(pose.pose.orientation.x),
        float(pose.pose.orientation.y),
        float(pose.pose.orientation.z)
      };

      return new_msg;
    }

    frame_info get_position(std::string obj_name){
      frame_info new_frame;
      this->vicon_client.GetFrame();
      Output_GetLatencyTotal latency = this->vicon_client.GetLatencyTotal();
      Output_GetFrameNumber frame_number = this->vicon_client.GetFrameNumber();
      Output_GetFrameRate frame_rate = this->vicon_client.GetFrameRate();
      // RCLCPP_INFO(this->get_logger(), "Rate= %f (Hz) Result= %d", frame_rate.FrameRateHz, frame_rate.Result);
      /*Get all objects*/
      Output_GetSubjectCount sub_cnt = this->vicon_client.GetSubjectCount();
      
      for(int i = 0; i<int(sub_cnt.SubjectCount); i++){
        Output_GetSubjectName name = this->vicon_client.GetSubjectName(i);
        /*get all segments for each object*/
        if(std::string(name.SubjectName) == obj_name){
      
          std::vector<segment_info> segments = this->get_segments_info(obj_name);
          new_frame.latency_seconds = latency.Total;
          new_frame.frame_number = int(frame_number.FrameNumber);
          new_frame.frame_rate = float(frame_rate.FrameRateHz);

          new_frame.segments = segments;
          new_frame.valid = true;
        }
      }
      return new_frame;

    }

    std::vector<segment_info> get_segments_info(std::string obj_name){
      std::vector<segment_info> result;
      Output_GetSegmentCount seg_cnt = this->vicon_client.GetSegmentCount(obj_name);
      // RCLCPP_INFO(this->get_logger(), "Number of Segments = %d", seg_cnt.SegmentCount);
      if (seg_cnt.Result == Result::Success){
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

    int get_timestamp(){
      rclcpp::Duration diff = this->now() - this->start_time;
      std::chrono::microseconds num_of_us = diff.to_chrono<std::chrono::microseconds>();
      return num_of_us.count();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string vicon_tracker_ip = "192.168.50.56";
    ViconDataStreamSDK::CPP::Client vicon_client;
    rclcpp::Time start_time;
    int px4_timestamp = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViconRos2>());
  rclcpp::shutdown();
  return 0;
}