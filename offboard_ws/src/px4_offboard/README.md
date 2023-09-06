# px4-offboard
This `package` contains python examples for offboard control on ROS2 with [PX4](https://px4.io/)

The `px4_offboard` package contains the following nodes
- `offboard_uav_test1.py`: Example of basic mission reaching one waypoint for single drone
- `offboard_uav_test2.py`: Example of mission comprising of several waypoints to achieve for single drone
- `offboard_arming_attitude.py`: Test mission to arm the real drone using VehicleAttitude message
- `offboard_uav2_test.py`: Example of mission comprising of several waypoints to achieve for a set of two drones
- `offboard_uav_fake_mocap.py`: Test mission to arm the real drone using fake mocap data published on VehicleOdometry message

## Run Offboard Mission: Single drone

On the first terminal, run the following:
```
cd PX4-Autopilot
make px4_sitl gz_x500
```

On the second terminal terminal, run Micro XRCE-DDS:
```
MicroXRCEAgent udp4 -p 8888
```

In order to run the offboard position control example, open a third terminal and run the node.

```
cd swarm_uav_config/offboard_ws
source install/local_setup.bash
ros2 run px4_offboard offboard_uav_test1
```

The drone should takeoff until an attitude of 5 meters and finally it should land and disarm for the mission to finish correctly.
By running instead `ros2 run px4_offboard offboard_uav_test2` the other mission starts.

## Run Offboard Mission: Two drones
On the first terminal, run the following:
```
cd PX4-Autopilot
make px4_sitl 
```

To build the sitl. After it finishes, on the same terminal run the first drone sim:
```
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,3" PX4_GZ_MODEL=x500 PX4_MICRODDS_NS="vhcl1" ./build/px4_sitl_default/bin/px4 -i 1 
```
That launches the gazebo sim of a drone spawned at position 0,3,0 named vhcl1 and starts a microdds client connected at port 8888.

On a second terminal run the sim for the second drone:
```
cd PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,6" PX4_GZ_MODEL=x500 PX4_MICRODDS_NS="vhcl2" ./build/px4_sitl_default/bin/px4 -i 2
```
That launches the gazebo sim of a drone spawned at position 0,6,0 named vhcl2 and starts.
In order for the second drone to operate correctly, it needs to establish the communication to a different port than the default 8888 assigned to the first drone. So inside the PX4 console of the second drone, type the following:
```
uxrce_dds_client stop
uxrce_dds_client start -t udp -p 7777
```

In this way, the second drone will establish connection with the agent using 7777 port.

On a third terminal run the agent for the first drone:

```
MicroXRCEAgent udp4 -p 8888
```

On a forth terminal run the agent for the second drone:
```
MicroXRCEAgent udp4 -p 7777
```

In order to run the offboard mission example for two drones, open a fifth terminal and run the the node.

```
cd swarm_uav_config/offboard_ws
source install/local_setup.bash
ros2 launch px4_offboard offboard_uav2_test.launch.py
```

The script should run and in gazebo both drones should takeoff, reach the two waypoints, finally land and disarm for the mission to finish correctly.

