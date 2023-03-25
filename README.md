This repository was build as a reference for the setup and simulation of a mission done by a drone using PX4 and ROS2. In order to correctly launch the mission scripts for the simulated UAV, we need to first install and configure some programs and packages.
This repo uses Px4 v1.13.3 and gazebo classic 11.10.2 in order to match the firmware mounted on the drone.

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

Install the development environment for Ubuntu 22.04 LTS. Clone the repository of PX4 in any folder
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
The last command automaticaly install Gazebo Garden and its dependencies on Ubuntu 22.04. We do this step in order to be sure that all the dependencies that px4 has with gazebo are installed, we will not use Gazebo Garden itself.
In order to install the firmware version same as the one on the Flight controller, follow the procedure below.

For our configuration, we need to first downgrade to 1.13 then install gazebo 11.10.2.
Clean up first the existing version.
```
cd ~/PX4-Autopilot
make clean
make distclean
```
Fetch and checkout then add the modules to match the px4 v1.13 version.
```
git fecth origin release/1.13
git checkout release/1.13
make submodulesclean
```

Install Gazebo v11 and its dependencies and start the simulation.
Download the script from [here](https://raw.githubusercontent.com/gazebo-tooling/release-tools/master/one-line-installations/gazebo.sh). Then, source it using the following command. This will overwrite Gazebo Garden and install Gazebo 11.10.
```
bash gazebo.sh
```
Finally, run the following command to lanch the simulation.
```
make px4_sitl_rtps gazebo
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

To see if everything until now installed correctly follow the below steps in order to run a quick test.

## Test the configuration

In one terminal, start the px4 simulation.
```
source /opt/ros/humble/setup.bash
cd PX4_Autopilot
make px4_sitl_rtps gazebo
```
On the px4 terminal write ```microdds_client start -t udp -p 8888``` to run client that will interface with Micro-XRCE-DDS Agent.

In another terminal, source the ROS2 workspace and start the agent
```
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```
Additionaly the last step is to install the QGroundControl GCS app.

## QGroundControl installation
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

## Conclusion
All the necessary steps are done and now the simulation setup is completed. Next, we can move to the config of the workspace in which multiple scripts are built in order to command the drone in Offboard Mode. See [Offboard control example](px4_offboard/README.md) for details on the implementation.
