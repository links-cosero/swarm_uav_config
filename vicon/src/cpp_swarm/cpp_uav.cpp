#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:

	OffboardControl() : Node("offboard_control")
	{
		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		int mission_state = 0;
        float current_waypoint[3] = {0.0, 0.0, 0.0};

        void timer_offboard_cb();
        void mission();
        void arm();
        void disarm();
        void publish_trajectory_setpoint();
        void publish_offboard_control_mode();
        void publish_vehicle_command();

        rclcpp::TimerBase::SharedPtr timer_offboard = this->create_wall_timer(0.1s, timer_offboard_cb);
        rclcpp::TimerBase::SharedPtr timer_mission = this->create_wall_timer(10s, mission);

        std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    }

    void OffboardControl::timer_offboard_cb()
    {
        this->publish_trajectory_setpoint();
        this->publish_offboard_control_mode();
    }
		
    void OffboardControl::mission(){
        if (mission_state == 0){
            // Arming and takeoff to 1st waypoint
            /**
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
            **/
            RCLCPP_INFO(this->get_logger(), "Mission Started");
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            mission_state = 1;

        }else if (mission_state == 1)
        {
            this->arm();
            RCLCPP_INFO(this->get_logger(), "Vehicle Armed");
            RCLCPP_INFO(this->get_logger(), "First Waypoint");
            current_waypoint = {-2.5, -2.5, 2}
            current_waypoint = ENU2NED_vector_converter(current_waypoint)
            mission_state = 2

        }else if (mission_state == 2)
        {
            RCLCPP_INFO(this->get_logger(), "Second Waypoint");
            current_waypoint = {-2.5, 2.5, 2}
            current_waypoint = ENU2NED_vector_converter(current_waypoint)
            mission_state = 3
        }else if (mission_state == 3)
        {
            RCLCPP_INFO(this->get_logger(), "Vehicle Landing");
            current_waypoint = {-2.5, 2.5, 2}
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            mission_state = 4
        }else if (mission_state == 4)
        {
            rclcpp::TimerBase::cancel(this->timer_offboard);
            rclcpp::TimerBase::cancel(this_>timer_mission);
            RCLCPP_INFO(this->get_logger(), "Mission Finished");
        }
            
        }

        /**
         * @brief Send a command to Arm the vehicle
         */
        void arm()
        {
        	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        }

        /**
         * @brief Send a command to Disarm the vehicle
         */
        void disarm()
        {
        	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        
        	RCLCPP_INFO(this->get_logger(), "Disarm command send");
        }

        /**
         * @brief Publish the offboard control mode.
         *        For this example, only position and altitude controls are active.
         */
        void publish_offboard_control_mode()
        {
        	OffboardControlMode msg{};
        	msg.position = true;
        	msg.velocity = false;
        	msg.acceleration = false;
        	msg.attitude = false;
        	msg.body_rate = false;
        	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_->publish(msg);
        }

        /**
         * @brief Publish a trajectory setpoint
         *        For this example, it sends a trajectory setpoint to make the
         *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
         */
        void publish_trajectory_setpoint()
        {
        	TrajectorySetpoint msg{};
        	msg.position = {0.0, 0.0, -5.0};
        	msg.yaw = -3.14; // [-PI:PI]
        	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        	rclcpp::Publisher<OffboardControlMode>::SharedPtr rajectory_setpoint_publisher_->publish(msg);
        }

        /**
         * @brief Publish vehicle commands
         * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
         * @param param1    Command parameter 1
         * @param param2    Command parameter 2
         */
        void publish_vehicle_command(uint16_t command, float param1, float param2)
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
        	rclcpp::Publisher<OffboardControlMode>::SharedPtr vehicle_command_publisher_->publish(msg);
        }
    }
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