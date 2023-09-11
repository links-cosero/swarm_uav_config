#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <array>
#include <cmath>
#include <eigen3/Eigen/Eigen>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		mission_state = 0;
        current_waypoint = {0.0,0.0,0.0};
		timer_mission = this->create_wall_timer(10s, std::bind(&OffboardControl::mission_cb, this));
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

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t mission_state;   // counter to define mission as state machine
    Eigen::Matrix<double,3,1> current_waypoint;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void mission_cb();
    void offboard_cb();
	Eigen::Matrix<double,3,1> ENU2NED_vector_converter(Eigen::Matrix<double,3,1> vector);
};

/**
 * @brief Define a mission composed by a set of waypoints for the drone to reach
*/
void OffboardControl::mission_cb()
{
    if(mission_state == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Mission started");
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
        mission_state = 1;
    }else if (mission_state == 1)
    {
        /*  ARMING THE VEHICLE AND REACHING THE FIRST WAYPOINT*/
        RCLCPP_INFO(this->get_logger(), "Arming");
		this->arm();
        RCLCPP_INFO(this->get_logger(), "Reaching First Waypoint");

        current_waypoint = {-2.5,-2.5,2.0};
		current_waypoint = ENU2NED_vector_converter(current_waypoint);
		mission_state = 2;
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

Eigen::Matrix<double,3,1> OffboardControl::ENU2NED_vector_converter(Eigen::Matrix<double,3,1> vector)
{
	// Rotation around Z-axis of 90° Degrees
	Eigen::Matrix<double,3,3> rotZ { {cos(M_PI/2), -sin(M_PI/2), 0},
									{sin(M_PI/2), cos(M_PI/2), 0}, 
									{0,0,1}};

	vector = rotZ*vector;

	// Rotation around X-axis of 180° Degrees
	Eigen::Matrix<double,3,3> rotX { {1,0,0}, 
									{0, cos(M_PI), -sin(M_PI)},
									{0, sin(M_PI), cos(M_PI)}};

	vector = rotX*vector;

	return vector;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
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
	msg.yaw = M_PI/2; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
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