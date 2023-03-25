# Offboard Control example
**WIP: The gazebo simulator gives me notify negative when trying to set Offboard Mode in QGroundControl in order to make the drone follow the script mission**
This `repository` contains a python examples for offboard control on ROS2 with PX4 and below you can find the steps to make it works with PX4 Firmware version 1.13.3, Gazebo version 11.10.2, ROS2 Humble, Micro-XRCE Agent and Fast-RTPS-Gen.

The `px4_offboard` package contains the following nodes
- `offboard_control.py`: Example of offboard position control using position setpoints
- `visualizer.py`: Used for visualizing vehicle states in Rviz

The source code is released under a BSD 3-Clause license.

- **Author**: Jaeyoung Lim
- **Affiliation**: Autonomous Systems Lab, ETH Zurich

## Setup
Create a workspace and source the needed packages.
```
mkdir -p offboard_ws/src
cd src
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
git clone https://github.com/PX4/px4_msgs.git -b release/1.13
```
Next use the colcon build tool in order to build the workspace.
```
cd ..
colcon build
```
Notice that several errors will come that will make the building stop unsuccessfully, so look at the **Troubleshooting** section for details about the procedure to fix errors. 

## Running
You will make use of 3 different terminals to run the offboard demo.

On the first terminal, run a SITL instance from the PX4 Autopilot firmware.
```
source /opt/ros/humble/setup.bash
make px4_sitl_rtps gazebo
```
Inside the PX4 console type ```microdds_client start -t udp -p 8888```

On a second terminal terminal, run the Micro XRCE-DDS. So that ROS2 Nodes are able to communicate with the PX4 microdds_client.
```
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

In order to run the offboard position control example, open a third terminal and run the the node.
This runs two ros nodes, which publishes offboard position control setpoints and the visualizer.
```
source /opt/ros/humble/setup.bash
ros2 launch px4_offboard offboard_position_control.launch.py
```
![offboard](https://user-images.githubusercontent.com/5248102/194742116-64b93fcb-ec99-478d-9f4f-f32f7f06e9fd.gif)

In order to just run the visualizer,
```
ros2 launch px4_offboard visualize.launch.py
```

## Troubleshooting
When building px4_msgs a stderr will arise and stop the build of the workspace, like follows.
```
...
--- stderr: px4_msgs
offboard_ws/build/px4_msgs/rosidl_generator_c/px4_msgs/msg/detail/gimbal_v1_command__struct.h:356:3: error: expected identifier or ‘(’ before ‘/’ token
offboard_ws/build/px4_msgs/rosidl_generator_c/px4_msgs/msg/detail/gimbal_v1_command__struct.h:366:3: error: expected identifier or ‘(’ before ‘/’ token
...
```
In order to fix, go inside the mentioned file and erase one ```*/```, then rebuild.
The same fix has to be implemented as a subsequent build will give the following error.
```
offboard_ws/build/px4_msgs/rosidl_generator_c/px4_msgs/msg/detail/vehicle_command__struct.h:356:3: error: expected identifier or ‘(’ before ‘/’ token
```

Then, another error will arise now regarding px4-offboard package.
```
...
--- stderr: px4-offboard
/usr/lib/python3.10/site-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
...
```
It means that you have installed a version of setuptools that is above 58.2.0, to fix you need to downgrade to this version. Type the following.
```
pip install setuptools==58.2.0
```

Finally, another stderr that will occur is the following one.
```
...
UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
UserWarning: Usage of dash-separated 'install-scripts' will not be supported in future versions. Please use the underscore name 'install_scripts' 
--- stderr: px4-offboard     
```
To fix, go inside the package px4-offboard and change script-dir to script_dir and install-scripts to install_scripts.
Next rebuild and everything should work without errors. The setup is completed.
