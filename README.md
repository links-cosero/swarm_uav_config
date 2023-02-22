# ROS2 Humble installation
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

# PX4 and Gazebo installation
https://docs.px4.io/main/en/sim_gazebo_gz/

Install development enviroment for Ubuntu 22.04 LTS. Clone the repository of PX4 in any folder
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
Install Gazebo Garden
```
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```
Create a test simulation with PX4 and Gazebo. It should open Gazebo with a 4 rotor drone and can be controlled from the interactive PX4 console.
```
cd /path/to/PX4-Autopilot
make clean
make px4_sitl gz_x500
```

# ROS-PX4 bridge
https://docs.px4.io/main/en/ros/ros2_comm.html

PX4 communicates with MicroXRCEAgent, that is a middleware executed on the offboard computer. XRCEAgent communicate with PX4 with a UPD port and then transforms the uORB messages in suitable ROS2 topics.

## MicroXRCEAgent installation
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

## ROS2 workspace setup
We need to create a workspace to let ROS know the structure of PX4 messages. We need to clone two repositories [PX4-msgs](https://github.com/PX4/px4_msgs#PX4-msgs) and [PX4-ros-com](https://github.com/PX4/px4_ros_com#PX4-ros-com) 
Example of creation of workspace:
```
mkdir ~/workspace/src/
cd workspace/src
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
cd ~/workspace/
source /opt/ros/humble/setup.bash
colcon build
```
Finally source the setup file
```
source install/local_setup.bash
```

## Offboard Example

First, clone the offboard package inside the workspace folder and build it.
```
cd ~/workspace/src/
mkdir src
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git src/px4-offboard
colcon
```
Open [Terminator](https://github.com/gnome-terminator/terminator/blob/master/INSTALL.md) app and start 4 terminals. In each one of them first source the setup file of px4_ros2 workspace.
```
cd ~/workspace
source install/local_setup.sh
```
On the first one, start the simulation of Gx 500 quadrotor inside Gazebo Garden.
```
cd PX4-Autopilot
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
