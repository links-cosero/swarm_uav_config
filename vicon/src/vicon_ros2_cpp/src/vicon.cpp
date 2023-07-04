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
      /*ViconSDK client init*/
      Output_Connect result =  this->vicon_client.Connect(this->vicon_tracker_ip);
      if (result.Result == Result::NotConnected){
        RCLCPP_WARN(get_logger(), "Client not connected!");
      }
      this->vicon_client.EnableSegmentData();
      this->vicon_client.SetStreamMode(StreamMode::ServerPush);
      this->vicon_client.SetAxisMapping(
        Direction::Forward,
        Direction::Right,
        Direction::Down);
      this->vicon_client.SetBufferSize(1000);

      /*Purge first 10 frames (???????)
      It makes the frame count consistent*/
      for(int i=0; i<10;i++){
        frame_info first_frame = this->get_position("drone1");
      }
      /* Align frames timestamp with system time */
      frame_info first_frame = this->get_position("drone1");
      this->vicon_frame_offset = first_frame.frame_number;//this->frame_to_timestamp(first_frame.frame_number);
      this->start_time = this->now();

      /*--- SUBSCRIPTIONS ---*/
      this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", 10, 
        std::bind(&ViconRos2::timesync_cb, this, _1));

      /*--- PUBLISHERS ---- */
      this->vicon_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vicon/pose", 10);
      this->px4_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry",10);

      timer_ = this->create_wall_timer(
        10ms, std::bind(&ViconRos2::vicon_rcv_cb, this)); // 100 Hz
      timer_px4 = this->create_wall_timer(
        20ms, std::bind(& ViconRos2::pub_px4_odom, this)); // 50 Hz
    }

  private:
    /* VICON frame number to timestamp in usec */
    uint64_t frame_to_timestamp(int frame_num){
      float delta_time_sec = 1.0/100.0; // [sec] time between frames
      float frame_timestamp_sec = float(frame_num - vicon_frame_offset) * delta_time_sec;
      uint64_t frame_timestamp_usec = uint64_t(frame_timestamp_sec * 1E6);
      // RCLCPP_INFO(this->get_logger(), "Delta sec= %f (s) frame_num= %d, frame_ts_sec=%f, frame_ts_usec=%ld", 
      //         delta_time_sec, this->last_frame->frame_number, frame_timestamp_sec, frame_timestamp_usec);
      return frame_timestamp_usec;
    }

    /* Create and publish a VehicleOdometry message for PX4 */
    void pub_px4_odom(){

      if (this->frame_buffer.size() >= 4){
        float frame_latency = 0;
        frame_info last_frame;
        /* Discard frames too old */
        do{
          last_frame = this->frame_buffer.front(); 
          frame_buffer.pop_front();
          frame_latency = get_frame_delay_ms(last_frame);
        }while(frame_latency > max_pub_latency && frame_buffer.size() > 0);
        
        // Removes 2 elements from the list: 100 Hz -> 50 Hz
        if (frame_buffer.size() > 0){
          // this->frame_buffer.pop_front();
        }
        RCLCPP_WARN(this->get_logger(), "Frame latency = %.2f ms", frame_latency);

        uint64_t frame_timestamp_usec = this->frame_to_timestamp(last_frame.frame_number);
        px4_msgs::msg::VehicleOdometry px4_msg = this->create_odom_msg(last_frame, frame_timestamp_usec);
        
        if (frame_latency > max_pub_latency){
          RCLCPP_WARN(this->get_logger(), "Frame latency too high! %.2f ms", frame_latency);
        }

        this->px4_pub_->publish(px4_msg);
        // RCLCPP_INFO(this->get_logger(), "Framerate=%.2f, Timestamp=%ld, timestamp_sample=%ld, diff=%ld",
        //       this->last_frame->frame_rate, px4_msg.timestamp, px4_msg.timestamp_sample, px4_msg.timestamp-px4_msg.timestamp_sample);
        // RCLCPP_INFO(this->get_logger(), "Frame number=%ld, Frame delta =%ld, timestamp_sample=%ld",
        //       this->last_frame->frame_number, this->last_frame->frame_number - this->vicon_frame_offset, px4_msg.timestamp_sample);
      }else{
        // RCLCPP_WARN(this->get_logger(), "Buffer of frames is empty!");
      }
    }

    void timesync_cb(const px4_msgs::msg::TimesyncStatus & msg){
      this->px4_timestamp = msg.timestamp;
    }

    void vicon_rcv_cb()
    {
      geometry_msgs::msg::Point translation;
      geometry_msgs::msg::Quaternion q;
      float frame_latency = 0;
      frame_info new_frame;

      do{
        new_frame = this->get_position("drone1");
        frame_latency = get_frame_delay_ms(new_frame);
        // RCLCPP_INFO(get_logger(), "Frame latency: %.2f ms", frame_latency);
      }while(frame_latency > max_rcv_latency);


      if((new_frame.segments.size() != 0) && new_frame.valid){
        geometry_msgs::msg::PoseStamped last_pose;
        // send only the info about the first segment on the list
        translation.x = new_frame.segments[0].px / 1000;
        translation.y = new_frame.segments[0].py / 1000;
        translation.z = new_frame.segments[0].pz / 1000;
        q.x = new_frame.segments[0].qx;        
        q.y = new_frame.segments[0].qy;
        q.z = new_frame.segments[0].qz;
        q.w = new_frame.segments[0].qw;        
        
        last_pose.header.frame_id = "map";
        last_pose.header.stamp = this->get_clock()->now();
        last_pose.pose.position = translation;
        last_pose.pose.orientation = q;

        this->vicon_pub_->publish(last_pose);
        this->frame_buffer.push_back(new_frame);

        // RCLCPP_INFO(this->get_logger(), "Frame number=%ld, Frame delta =%ld",
        //       this->last_frame->frame_number, this->last_frame->frame_number - this->vicon_frame_offset);
      }


    }

    px4_msgs::msg::VehicleOdometry create_odom_msg(frame_info frame, uint64_t ts_sample){
      px4_msgs::msg::VehicleOdometry new_msg;
      new_msg.timestamp = this->get_timestamp();
      new_msg.timestamp_sample = ts_sample;
      new_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
      new_msg.position = {
        float(frame.segments[0].px) / 1000,
        float(frame.segments[0].py) / 1000,
        float(frame.segments[0].pz) / 1000
      };
      new_msg.q = {
        float(frame.segments[0].qw),
        float(frame.segments[0].qx),
        float(frame.segments[0].qy),
        float(frame.segments[0].qz)
      };
      new_msg.position_variance = {0.001, 0.001, 0.001};
      // float norm = sqrt(pow(new_msg.q[0],2.0)+ pow(new_msg.q[1],2.0)+ pow(new_msg.q[2],2.0), pow(new_msg.q[3],2.0));
      // RCLCPP_INFO(this->get_logger(), "Quaternion norm=%f", norm);

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

    /* Get microseconds from startup */
    uint64_t get_timestamp(){
      rclcpp::Duration diff = this->now() - this->start_time;
      std::chrono::microseconds num_of_us = diff.to_chrono<std::chrono::microseconds>();
      return num_of_us.count();
    }

    /* Get the milliseconds from the sampling to now */
    float get_frame_delay_ms(frame_info frame){
      uint64_t now = get_timestamp();
      uint64_t frame_ts = frame_to_timestamp(frame.frame_number);
      return float(float(now) - float(frame_ts)) / 1E3;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_px4;
    std::string vicon_tracker_ip = "192.168.50.56";
    ViconDataStreamSDK::CPP::Client vicon_client;
    rclcpp::Time start_time;
    std::list<frame_info> frame_buffer = {};
    int px4_timestamp = 0;
    uint64_t vicon_frame_offset = 0;
    float max_rcv_latency = 30; // FIXME: valore da rivedere;
    float max_pub_latency = 60; // FIXME: valore da rivedere;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViconRos2>());
  rclcpp::shutdown();
  return 0;
}