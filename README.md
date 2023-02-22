# uav_config
Repository containing docs to install and run the uav simulation using px4, ros2 and gazebo

## Offboard Example

Open Terminator app and start 4 terminals. In each one of them first source the setup file of px4_ros2 workspace.
```
cd px4_ros2_demo_ws
source install/local_setup.sh
```
On the first one, start the simulation of Gx 500 quadrotor inside Gazebo Garden.
```
cd PX4-Autopilot
make px4_sitl gz_x500
```
On the second one, start the micro-ROS agent:
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
On third one check first the topic list and then check that the vehicle_status publishes something:
```
ros2 topic list
ros2 topic echo /fmu/out/vehicle_status
```
Ctrl+C to stop the topic echo. On the same terminal, run the offboard example:
```
ros2 launch px4_offboard offboard_position_control.launch.py
```

On the forth terminal, open QGroundControl which should connect automatically.
```
./QGroundControl.AppImage
```
To allow arming, we need to tell PX4 that it's fine to do offboard control without RC connected.
Go to **Vehicle Setup**, **Parameters**, and set [COM_RCL_EXCEPT](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#COM_RCL_EXCEPT) to 4 which means Offboard is ignored.
Then go back and click on the mode and switch it to **Offboard**. Then, click on **Ready to fly** and click **Arm**.

It should now take off and start flying a circle, as commanded by the offboard Python script.
