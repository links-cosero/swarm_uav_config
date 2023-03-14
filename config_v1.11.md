In order to install the firmware version same as the one on the Flight controller, follow the procedure below.
Clean up first the existing version.
```
cd ~/PX4-Autopilot
make clean
make distclean
```
Fetch and checkout then add the modules to match the px4 v1.11 version.
```
git fecth origin release/1.11
git checkout release/1.11
make submodulesclean
```

Install Gazebo v11 and its dependencies and start the simulation.
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
make px4_sitl gazebo
```

An error should occour like the following error: no matching function for call to ‘max(long int, int)’. To fix it go inside the file and add a cast to long int.
```
gedit /home/$user/PX4-Autopilot/platforms/common/px4_work_queue/WorkQueueManager.cpp 
```
In line 257, add static_cast<long> before PX4_STACK_ADJUSTED(wq->stacksize). Then rerun the command.

