# px4-offboard
This `repository` contains a python examples for offboard control on ROS2 with [PX4](https://px4.io/)

The `px4_offboard` package contains the following nodes
- `offboard_uav.py`: Example of offboard mission of a single drones
- `offboard_uav.py`: Example of offboard mission of two drones 

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

In order to run the offboard position control example, open a third terminal and run the the node.

```
cd swarm_uav_config/offboard_ws
source install/local_setup.bash
ros2 run px4_offboard offboard_uav
```

The drone should takeoff, follow the script by walking to the two waypoints in sequence and finally it should land and disarm for the mission to finish correctly.

## Run Offboard Mission: Two drones
On the first terminal, run the following:
```
cd PX4-Autopilot
make px4_sitl 
```

To build the sitl. After it finishes, on the same terminal run the first drone sim:
```
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,3" PX4_GZ_MODEL=x500 PX4_MICRODDS_NS="vhcl1" MICRODDS_PORT="8888" ./build/px4_sitl_default/bin/px4 -i 1 
```

That launches the gazebo sim of a drone spawned at position 0,3,0 named vhcl1 and starts a microdds client connected at port 8888.
On the second terminal, run Micro XRCE-DDS Agent that will pub/sub to the first drone only:
```
MicroXRCEAgent udp4 -p 8888
```

On a third terminal run the sim for the second drone:
```
cd PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,6" PX4_GZ_MODEL=x500 PX4_MICRODDS_NS="vhcl2" MICRODDS_PORT="7777" ./build/px4_sitl_default/bin/px4 -i 2
```
That launches the gazebo sim of a drone spawned at position 0,6,0 named vhcl2 and starts a microdds client connected at port 7777.
On the forth terminal, run Micro XRCE-DDS Agent that will pub/sub to the second drone only:
```
MicroXRCEAgent udp4 -p 7777
```

In order to run the offboard mission example for two drones, open a fifth terminal and run the the node.

```
cd swarm_uav_config/offboard_ws
source install/local_setup.bash
ros2 run px4_offboard offboard_uav2
```

The script should run and in gazebo both drones should takeoff, reach the two waypoints and finally land.

