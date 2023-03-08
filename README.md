This repository was build as a reference for the setup and simulation of a mission done by a drone using PX4 and ROS2. In order to correctly launch the mission scripts for the simulated UAV, we need to first install and configure some programs and packages.

## ROS2 Humble installation
See the [docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for more details. Below there are listed all the needed command to install ROS2 humble.

Setup locale
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
Verify settings with `locale` command

Setup resources and repositories
```
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Install of ROS2 packages 
```
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

Configure the environment
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
printenv | grep -i ROS 
```
Check that:
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```
Finally, export the following two variables
```
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```
## PX4 and Gazebo installation
See the [docs](https://docs.px4.io/main/en/sim_gazebo_gz/) for more details. Below there are the steps to follow in order to correctly install PX4 and its dependencies.

Install development enviroment for Ubuntu 22.04 LTS. Clone the repository of PX4 in any folder
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
The last command automaticaly install Gazebo Garden and its dependencies on Ubuntu 22.04. Restart the PC on completition.

Create a test simulation with PX4 and Gazebo. It should open Gazebo with a 4 rotor drone and can be controlled from the interactive PX4 console.
```
cd /path/to/PX4-Autopilot
make clean
make px4_sitl gz_x500
```

## ROS-PX4 bridge: MicroXRCEAgent installation
See the [docs](https://docs.px4.io/main/en/ros/ros2_comm.html) for more details. Below are the steps to install the bridge.

PX4 communicates with MicroXRCEAgent, that is a middleware executed on the offboard computer. XRCEAgent communicate with PX4 with a UPD port and then transforms the uORB messages in suitable ROS2 topics.

Install needed python packages
```
sudo pip3 install -U empy pyros-genmsg setuptools
```
Clone the repository and build the source code. Clone the repository in any folder (outside PX4 folder)
```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

##QGroundControl installation
Enter the following commands to install properly the program.
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
chmod +x ./QGroundControl.AppImage
```

## ROS2 workspace setup
We need to create a workspace to let ROS know the structure of PX4 messages. We need to clone two repositories [PX4-msgs](https://github.com/PX4/px4_msgs#PX4-msgs) and [PX4-ros-com](https://github.com/PX4/px4_ros_com#PX4-ros-com) 

```
mkdir -p ~/uav_config_ws/src/
cd ~/uav_config_ws/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```
Install Colcon if not present
```
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions
```
Compile the source code with Colcon
```
cd ..
colcon build
```

All the programs, packages and their dependencies are now fully installed. You can proceed to the following steps in order to setup and launch the simulation demo.

## Offboard Example

First, clone the offboard package inside the workspace folder and build it.
```
cd ~/uav_config_ws/src/
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git src/px4-offboard
cd ..
colcon build
```
Close the terminal.
Install [Terminator](https://github.com/gnome-terminator/terminator/blob/master/INSTALL.md) from the link. Then, open the app and start 4 terminals. In each one of them first source the local setup file of the workspace or instead write the following command to source automaticaly everytime a new shell is opened.
```
cd ~/uav_config_ws
source /home/$user/uav_config_ws/install/local_setup.bash
```
**Warning** if instead local_setup.sh is sourced the ros2 topic list command will not see the published topic so be careful!

On the first one, start the simulation of Gx 500 quadrotor inside Gazebo Garden.
```
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
On the second one, execute the XRCEAgent as follows:
```
MicroXRCEAgent udp4 -p 8888
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

## Troubleshooting
When building px4_ros_com a warning may arise like follows.
```
...
warning: format ‘%llu’ expects argument of type ‘long long unsigned int’, but argument 5 has type ‘px4_msgs::msg::DebugVect_<std::allocator<void> >::_timestamp_type’ {aka ‘long unsigned int’}
--- stderr: px4_ros_com
...
```

In order to solve the warning go inside px4_ros_com/src/examples/advertisers/debug_vect_advertiser.cpp and change %llu to %lu in Line 66.
Now build again:
```
cd ~/uav_config_ws
colcon build --packages-select px4_ros_com 
```
And the package should build without stderr.

When building px4_offboard a warning may arise as follows.
```
...
UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
UserWarning: Usage of dash-separated 'install-scripts' will not be supported in future versions. Please use the underscore name 'install_scripts' 
--- stderr: px4_offboard     
```

Go inside the package px4-offboard and change script-dir to script_dir and install-scripts to install_scripts.
Next do:
```
cd ~/uav_config_ws
colcon build --packages-select px4_offboard 
```
And the package should build without stderr.
