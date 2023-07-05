#include <cstdio>
#include <memory>
#include <thread>

#include "DataStreamRetimingClient.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/timesync_status.hpp"

using namespace std::chrono_literals;
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

class ViconRos2 : public rclcpp::Node
{
public:
    ViconRos2()
    : Node("vicon")
    {
		this->vicon_client.Connect(vicon_tracker_ip, 50);

	}

	void vicon_rcv(){
	Output_WaitForFrame WaitOutput = vicon_client.WaitForFrame();
		if( WaitOutput.Result == Result::Success )
		{
			RCLCPP_INFO(get_logger(), "Frame ", frame_latency, new_frame.frame_number);
		}
	}

private:
	RetimingClient vicon_client;
	std::string vicon_tracker_ip = "192.168.50.56";   // VICON local ip addr.
};