#include "vehicle.hpp"
#include "rclcpp/rclcpp.hpp"
#include <math.h>

using namespace px4_msgs::msg;
using std::placeholders::_1;

Vehicle::Vehicle() : Node("vehicle_node"){
	if (std::string(this->get_namespace()) == "/"){
		this->ros_namespace = "";
	}else{
		this->ros_namespace = this->get_namespace();
	}
	
	/*Offboard control mode message*/
	this->offboard_control_mode = std::make_shared<OffboardControlMode>();
	this->offboard_control_mode->timestamp = this->get_now_timestamp();
	this->offboard_control_mode->body_rate = false;
	this->offboard_control_mode->attitude = false;
	this->offboard_control_mode->position = true;
	this->offboard_control_mode->velocity = false;
	this->offboard_control_mode->acceleration = false;

	/*Creating publishers*/
	this->vehicle_command_pub = this->create_publisher<VehicleCommand>(
		std::string(this->ros_namespace + "/fmu/in/vehicle_command"), pub_qos);
	this->offboard_control_mode_pub = this->create_publisher<OffboardControlMode>(
		std::string(this->ros_namespace + "/fmu/in/offboard_control_mode"), pub_qos);
	this->trajectory_setpoint_pub = this->create_publisher<TrajectorySetpoint>(
		std::string(ros_namespace + "/fmu/in/trajectory_setpoint"), pub_qos);

	/*Creating subscriptions*/
	this->vehicle_status_sub = this->create_subscription<VehicleStatus>(
		std::string(this->ros_namespace + "/fmu/out/vehicle_status"), sub_qos, 
		std::bind(& Vehicle::vehicle_status_cb, this, _1));
	this->vehicle_control_mode_sub = this->create_subscription<VehicleControlMode>(
		std::string(this->ros_namespace + "/fmu/out/vehicle_control_mode"), sub_qos, 
		std::bind(& Vehicle::vehicle_control_mode_cb, this, _1));
	this->timesync_status_sub = this->create_subscription<TimesyncStatus>(
		std::string(this->ros_namespace + "/fmu/out/timesync_status"), sub_qos, 
		std::bind(& Vehicle::timesync_status_cb, this, _1));
	this->vehicle_odometry_sub = this->create_subscription<VehicleOdometry>(
		std::string(this->ros_namespace + "/fmu/out/vehicle_odometry"), sub_qos, 
		std::bind(& Vehicle::vehicle_odometry_cb, this, _1));
	this->acceleration_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
		std::string(this->ros_namespace + "/acceleration_cmd"), 10, 
		std::bind(& Vehicle::estimator_cb, this, _1));
	this->flocking_start_sub = this->create_subscription<std_msgs::msg::Bool>(
		std::string("/start_flocking"), 10, 
		std::bind(& Vehicle::flocking_start_cb, this, _1));

	this->current_state = MissionState::IDLE;
	this->next_state = MissionState::IDLE;

	this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
	this->tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

	// this->broadcast_camera_tf();

	RCLCPP_INFO(this->get_logger(), "Vehicle initialized");
}

void Vehicle::send_arm_command(){
	/* Send arm command to the drone via DDS */
	VehicleCommand command_msg = this->create_vehicle_command(
		VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	this->vehicle_command_pub->publish(command_msg);
}

void Vehicle::send_disarm_command(){
	VehicleCommand command_msg = this->create_vehicle_command(
		VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	this->vehicle_command_pub->publish(command_msg);
}

VehicleCommand Vehicle::create_vehicle_command(int command, float param1, float param2,
	float param3, float param4, float param5, float param6, float param7){
	/*Create vehicle command message*/
	auto msg = VehicleCommand();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;  					// command ID
	msg.target_system = this->system_id();  	// system which should execute the command
	msg.target_component = this->component_id();// component which should execute the command, 0 for all components
	msg.source_system = 10; 					// system sending the command
	msg.source_component = 1; 					// component sending the command
	msg.from_external = true;
	msg.timestamp = this->get_now_timestamp(); 	// time in microseconds
	return msg;
}

void Vehicle::offboard_flight_mode() {
	/*Change to offboard flight mode*/
	VehicleCommand command_msg = this->create_vehicle_command(
		VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
	this->vehicle_command_pub->publish(command_msg);
}

void Vehicle::publish_offboard_control_signal(){
	// This message have to be published > 2 Hz
	this->offboard_control_mode->timestamp = this->get_now_timestamp();
	this->offboard_control_mode_pub->publish(*this->offboard_control_mode);
}

void Vehicle::publish_trajectory_setpoint(px4_msgs::msg::TrajectorySetpoint message){
	this->trajectory_setpoint_pub->publish(message);
}

bool Vehicle::xrce_connected(){
	return (this->vehicle_status != nullptr) &&
		(this->vehicle_control_mode != nullptr) &&
		(this->timesync_status != nullptr) &&
		(this->vehicle_odometry != nullptr);
}

void Vehicle::mission_update(){

	current_state = next_state;
	VehicleCommand cmd;

	switch(current_state){
		case IDLE:
			{if (xrce_connected()){
				if(this->nav_state() == VehicleStatus::NAVIGATION_STATE_OFFBOARD){
					RCLCPP_INFO(this->get_logger(), "IDLE=>PREFLIGHT_CHECK");
					next_state = MissionState::PREFLIGHT_CHECK;
				}else{
					this->offboard_flight_mode(); /*Set offboard flight mode*/
					next_state = MissionState::IDLE;
				}
			}}
			break;
		//-----------------------------------------------------
		case PREFLIGHT_CHECK:
			{if(this->preflight_check())	{
				if(!this->is_armed()){
					this->vehicle_starting_position[0] = this->vehicle_odometry->position[0];
					this->vehicle_starting_position[1] = this->vehicle_odometry->position[1];
					this->vehicle_starting_position[2] = this->vehicle_odometry->position[2];
					this->send_arm_command();
					}
				else{
					next_state = MissionState::TAKEOFF;
					RCLCPP_INFO(this->get_logger(), "PREFLIGHT_CHECK=>TAKEOFF");
				}
			}else{
				next_state = MissionState::PREFLIGHT_CHECK;
			}}
		break;
		//-----------------------------------------------------
		case TAKEOFF:
			{if(this->flocking_start)
			{
				RCLCPP_INFO(get_logger(), "TAKEOFF=>MISSION");
				next_state = MissionState::LANDING;
			}
			else
			{
				publish_pos_setpoint(0.0, 0.0, -1, 0);
				next_state = MissionState::TAKEOFF;
			}}
			break;
		//-----------------------------------------------------	
		case MISSION:
			{if(! this->flocking_start)
			{
				RCLCPP_INFO(get_logger(), "MISSION=>LANDING");
				next_state = MissionState::LANDING;
				this->land_pos_x = vehicle_odometry->position[0] - vehicle_starting_position[0]; 
				this->land_pos_y = vehicle_odometry->position[1] - vehicle_starting_position[1]; 				
			}
			else
			{
				// See Vehicle::estimator_cb() for acceleration publication
				this->flocking_start = false;
				next_state = MissionState::MISSION;
			}}
			break;
		//-----------------------------------------------------
		case LANDING:
			{this->publish_pos_setpoint(
					this->land_pos_x,
					this->land_pos_y,
					-0.0, 0.0);
			if (vehicle_odometry->position[2] > -0.2){
				// this->offboard_timer->cancel();
				next_state = MissionState::POWEROFF;
				RCLCPP_INFO(get_logger(), "Landing detected! Performing poweroff ...");
				return;
			}
			next_state = MissionState::LANDING;}
			break;
		//-----------------------------------------------------
		case POWEROFF:
		{	
			// this->send_disarm_command();
			if (this->is_armed()){
				/* Send flight termination command after landing*/
				cmd = this->create_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0);
				this->vehicle_command_pub->publish(cmd);
			}
		}
		break;
		
		default:
			next_state = MissionState::IDLE;
			break;
	}
}

void Vehicle::publish_hor_acc_setpoint(float acc_x, float acc_y, float height, float yaw){
	/* Control drone in acceleration in XY and set a height */
	TrajectorySetpoint msg = TrajectorySetpoint();
	offboard_control_mode->acceleration=true;
	offboard_control_mode->velocity = false;
	offboard_control_mode->position = false;
	msg.position = {NAN, NAN, height};				// Important set the correct NAN values
	msg.velocity = {NAN, NAN, NAN};
	msg.acceleration = {acc_x, acc_y, NAN};
	msg.timestamp = this->get_now_timestamp();
	msg.yaw = yaw;
	trajectory_setpoint_pub->publish(msg);
}

void Vehicle::publish_pos_setpoint(float x, float y, float z, float yaw){
	TrajectorySetpoint msg = TrajectorySetpoint();
	offboard_control_mode->position = true;
	msg.position = {
		x + this->vehicle_starting_position[0], 
		y + this->vehicle_starting_position[1], 
		z};
	msg.timestamp = this->get_now_timestamp();
	msg.yaw = yaw;
	trajectory_setpoint_pub->publish(msg);
}

/* When a new acceleration command is available from the estimator, it is published to the autopilot */
void Vehicle::estimator_cb(const std_msgs::msg::Float64MultiArray & message){
	// ay is inverted to manage FLU=>FRD transform
	this->flocking_ax = float(message.data[0]);
	this->flocking_ay = float(-message.data[1]);

	/*Send the setpoint to the drone*/
	if(this->current_state == MissionState::MISSION){
		publish_hor_acc_setpoint(this->flocking_ax, this->flocking_ay, -2.0, 0);
	}
}

void Vehicle::vehicle_odometry_cb(const VehicleOdometry & message) {
	this->vehicle_odometry = std::make_shared<VehicleOdometry>(std::move(message));
	this->broadcast_drone_tf(message);
	};

void Vehicle::broadcast_drone_tf(VehicleOdometry msg){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = this->ros_namespace;
    t.transform.translation.x =  msg.position[0];
    t.transform.translation.y = -msg.position[1];
    t.transform.translation.z = -msg.position[2];
    t.transform.rotation.x =  msg.q[1];
    t.transform.rotation.y = -msg.q[2];
    t.transform.rotation.z = -msg.q[3];
    t.transform.rotation.w =  msg.q[0];
    this->tf_broadcaster->sendTransform(t);
  }

void Vehicle::broadcast_camera_tf(){
	geometry_msgs::msg::TransformStamped t;
	tf2::Quaternion q;
	q.setEuler(3.1415, 3.1415, 0.0);
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = this->ros_namespace;
	t.child_frame_id = this->ros_namespace + "/camera";
	t.transform.translation.x = 0.07;
	t.transform.translation.y = 0.0;
	t.transform.translation.z = 0.0;
	t.transform.rotation.x = q.x();
	t.transform.rotation.y = q.y();
	t.transform.rotation.z = q.z();
	t.transform.rotation.w = q.w();
	this->tf_static_broadcaster_->sendTransform(t);
	}