# px4-offboard
This `repository` contains a python examples for offboard control on ROS2 with [PX4](https://px4.io/)

The `px4_offboard` package contains the following nodes
- `offboard_custom.py`: Example of offboard position control using position setpoints

## Run

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
ros2 run px4_offboard offboard_custom
```

The drone should takeoff, follow the script by walking to the two waypoints in sequence and finally it should land and disarm for the mission to finish correctly.

