#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <array>
#include <list>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std;
using std::placeholders::_1;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    vehicle_status_sub_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&OffboardControl::vehicle_status_cb, this, _1));
    vehicle_odometry_sub_ = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&OffboardControl::vehicle_odometry_cb, this, _1));

		mission_state = -1;

    current_waypoint = {0.0, 0.0, -0.5};
    printf("First Waypoint in PX4 FRD Reference Frame: ");
    for(int i=0;i<3;i++)
        printf("%f ",current_waypoint[i]);
    printf("\n");


		timer_mission = this->create_wall_timer(3s, std::bind(&OffboardControl::mission_cb, this));
    timer_offboard = this->create_wall_timer(100ms, std::bind(&OffboardControl::offboard_cb, this));
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_mission;
  rclcpp::TimerBase::SharedPtr timer_offboard;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

  uint8_t nav_state;
  uint8_t arming_state;

  std::string drone_name;
  std::array<float,3UL> drone_position;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped


	int64_t mission_state;   // counter to define mission as state machine
  Eigen::Matrix<double,3,1> current_waypoint;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, int param1 = 0.0, int param2 = 0.0);
  void mission_cb();
  void offboard_cb();
  void vehicle_status_cb(const VehicleStatus::SharedPtr msg);
  void vehicle_odometry_cb(const VehicleOdometry::SharedPtr msg);
  Eigen::Matrix<double,3,1> FLU2FRD_vector_converter(Eigen::Matrix<double,3,1> vector);
};

/**
 * @brief Define a mission composed by a set of waypoints for the drone to reach
*/
void OffboardControl::mission_cb()
{

    if(mission_state == -1)
    {
        RCLCPP_INFO(this->get_logger(), "Switching to Offboard");
        /*  ENABLE OFFBOARD MODE
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
        */
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1., 6.);
        if(nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD)
          mission_state = 0;

    }else if (mission_state == 0)
    {
    
        RCLCPP_INFO(this->get_logger(), "Arming");
		    this->arm();
        if (arming_state == VehicleStatus::ARMING_STATE_ARMED)
          mission_state = 1;

    }else if (mission_state == 1)
    {
      // Reach initial waypoint current_waypoint = {0.0, 0.0, 0.5}
      // if the drone is below 5 cm range then compute next waypoint 

        printf("\nDrone position: ");
        for(int i=0;i<3;i++)
            printf("%f ",drone_position[i]);
        printf("\nSetpoint position: ");    
        for(int i=0;i<3;i++)
            printf("%f ",current_waypoint[i]);

        float result = abs(drone_position[2] - current_waypoint[2]);
        printf("Result: %f", result);
        
    
        if(result<=0.05)
        {
        
           Eigen::Matrix<double,3,1> c_w = {-0.6,-0.6, 0.5};

          current_waypoint = FLU2FRD_vector_converter(c_w);

          // current_waypoint = {-0.6, -0.6, 2.0};
        
            printf("\nSecond Waypoint in PX4 FRD Reference Frame: ");
            for(int i=0;i<3;i++)
              printf("%f ",current_waypoint[i]);
            printf("\n");
          
               mission_state = 2;
        }  
             
    }else if (mission_state == 2)
	  { 
      printf("\nDrone position: ");
        for(int i=0;i<3;i++)
            printf("%f ",drone_position[i]);
        printf("\nSetpoint position: ");    
        for(int i=0;i<3;i++)
            printf("%f ",current_waypoint[i]);

      //check if second waypoint is reached
      if(abs(drone_position[0] - current_waypoint[0]) <= 0.05 &&
              abs(drone_position[1] - current_waypoint[1]) <= 0.05)
      {
	  	  RCLCPP_INFO(this->get_logger(), "Landing");
	  	  
        current_waypoint[2] = 0.0;

        mission_state = 3; 
      }
    }else if (mission_state == 3)
    {
      printf("\nDrone position: ");
      for(int i=0;i<3;i++)
          printf("%f ",drone_position[i]);
      printf("\nSetpoint position: ");    
      for(int i=0;i<3;i++)
          printf("%f ",current_waypoint[i]);

      float result = abs(drone_position[2] - current_waypoint[2]);
      printf("\nDifference = %f", result);
      if(result <= 0.10)
      {
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0);
	  	      mission_state = 4;
      }
	  }else if (mission_state == 4)
	  {
	  	RCLCPP_INFO(this->get_logger(), "Mission finished");
	  	timer_offboard->cancel();
	  	timer_mission->cancel();
	  	exit(0);
	  }
}

/**
 * @brief Define a mission composed by a set of waypoints for the drone to reach
*/
void OffboardControl::offboard_cb()
{
	publish_offboard_control_mode();
	publish_trajectory_setpoint();
} 

void OffboardControl::vehicle_status_cb(const VehicleStatus::SharedPtr msg)
{
  nav_state = msg->nav_state;
  arming_state = msg->arming_state;
}

void OffboardControl::vehicle_odometry_cb(const VehicleOdometry::SharedPtr msg)
{
  drone_position = msg->position;
}

/**
* @brief Function that transforms ROS2 ENU coords to NED coords as defined in https://docs.px4.io/main/en/ros/external_position_estimation.html#reference-frames-and-ros
*/
Eigen::Matrix<double,3,1> OffboardControl::FLU2FRD_vector_converter(Eigen::Matrix<double,3,1> vector)
{
	// Rotation around X-axis of 180° Degrees
	Eigen::Matrix<double,3,3> rotX { {1, 0, 0}, 
									{0, cos(M_PI), -sin(M_PI)},
									{0, sin(M_PI), cos(M_PI)} };

  //matrix*column vector
	vector = rotX*vector;

	return vector;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};

	std::array<float, 3UL> floatWaypoint;
	for (int i = 0; i < 3; ++i) {
        floatWaypoint[i] = static_cast<float>(current_waypoint(i));
    }

	msg.position = floatWaypoint;
	// The drone starts with a yaw of pi/2
	msg.yaw = M_PI; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, int param1, int param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}


int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}