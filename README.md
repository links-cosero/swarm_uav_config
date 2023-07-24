This repository was build as a reference for the setup and simulation of a mission done by a drone using PX4 and ROS2. In order to correctly launch the mission scripts for the simulated UAV, we need to first install and configure some programs and packages.
This repo uses Px4 v1.14 (main version), MicroXRCE and Gazebo Garden.

1. [Installation](offboard_ws/README.md) 
2. [Offboard control example](offboard_ws/src/px4_offboard/README.md)
3. [Groundtruth Example](gz_groundtruth/README.md)
4. [ROS 2 cross compilation](ROS2_cross_compile/cross_compilation.md)
4. [Vicon](vicon/vicon_docs.md)

## Quickstart
After all the flight controller and the companion computer are fully setup, just plug in the main battery of the drone: it should automatically connect to the wifi network and automatically start the XRCE-Agent. 

The companion computer will be ready when the ROS 2 topics of PX4 are available. To check for this use `ros2 topic list`. Topics starting with `/fmu/` should be present. If this does not happen in less than a minute the board didn't manage to connect to the wifi or a reboot of the XRCE-Agent is needed ([this guide](./drone_setup_docs/companion_board.md#creazione-service-per-startup-automatico-di-xrce-agent)) 

Once the topics are available the drone is completely ready to fly, just execute the offboard program.

Credits: Davide Morazzo, Ronald Cristian Dutu
