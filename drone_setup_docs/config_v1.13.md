|Component	|Version	|
| :--- 		| :--- 		|
| **Flight Controller** | OmnibusF4SD 	|
| **PX4** 				| v1.14.0b (main)|
| **PX4-ROS2 bridge**	| microXRCE		|
| **ROS 2**				| Humble		|
| **OS**				| Ubuntu 22.04  |

# Clone repository
In order to install the firmware version same as the one on the Flight controller, follow the procedure below.
```
cd ~
git clone git clone https://github.com/links-cosero/PX4-Autopilot.git --recursive
```
<!-- Clean up first the existing version.
```
cd ~/PX4-Autopilot
make clean
make distclean
```
Fetch and checkout then add the modules to match the px4 v1.11 version.
```
git fecth origin release/1.13
git checkout release/1.13
git submodule update --recursive
make distclean
``` -->

Install Gazebo v11 and its dependencies and start the simulation.
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
make px4_sitl gazebo
```

An error should occour like the following error: no matching function for call to ‘max(long int, int)’. To fix it go inside the file and add a cast to long int.
```
gedit /home/$user/PX4-Autopilot/platforms/common/px4_work_queue/WorkQueueManager.cpp 
```
In line 257, add static_cast\<long\> before PX4_STACK_ADJUSTED(wq->stacksize). Then rerun the command.

# PX4 setup for Omnibus F4SD
## Flash the bootloader
First we need to flash the bootloader using `BetaFlight-Configurator`. After downloading Betaflight to flash the bootloader on the flight controller:
- Connect the board via USB while holding the button on the board next to the connector to enter DFU mode. 
- Open `Firmware Flasher` tab
- Click on `Load firmware [Local]` and select the file `omnibusf4sd_bl_d52b70cb39.hex` (present in this folder) 
- Click on `Flash Firmware`

## Flash the PX4 firmware
Download the correct firmware file from the [github page](https://github.com/PX4/PX4-Autopilot/releases) (name is omnibus_f4sd_default.px4). Otherwise compile locally the source code (required for main branch). To do this [install the toolchain](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html), then :
```
cd ~/PX4-Autopilot
make omnibus_f4sd_default
```

An already compiled version for OmnibusF4SD flight controller is included in this folder `omnibus_f4sd_links.px4`.

> Checkout [this doc](./compile_v1.13.md) for mor details on the PX4 build for the OmnibusF4SD target. 

To flash the firmware on the board:
- connect via USB to the computer 
- open QGroundControl
- Navigate to `Vehicle Setup -> Firmware`
- Unplug and plug back in the USB
- Select `Advanced options -> Custom file` and click OK
- Now choose the wanted firmware to flash

Now the firmware is flashed onto the board. We can verify in the `Summary` page under the  `Airframe -> Firmware Version` tab the version of the firmware.

## Setup the firmare
In QGroundControl check in the `Summary` page that there are no problems highlighted. All the problems shown here have to be resolved.

Navigate then to `Vehicle Setup -> Parameters` and set the following:

**Parameters to set**:
| Parameter name 	| Value   	| Comment	|
| :---:   			| :---: 	| :---		| 	
| COM_CPU_MAX		| -1	   	| Disable CPU load check 	|
| SYS_MC_EST_GROUP	| ekf2		| Set EKF2 as estimator 	|

**Geometry parameters**
| Parameter name 	| Value   	| Comment	|
| :---:   			| :---: 	| :---		| 	
| PWM_MAIN_TIM(0,1)	| -3 	   	| DShot 600 |
| PWM_MAIN_FUNC1	| 101		| PWM1 output to Motor1 |
| PWM_MAIN_FUNC2	| 102		| PWM2 output to Motor2 |
| PWM_MAIN_FUNC3	| 103		| PWM3 output to Motor3 |
| PWM_MAIN_FUNC4	| 104		| PWM4 output to Motor4 |
| CA_ROTOR0_PX		| 0.07		| Motor 0 X position (to be done also for other motors)	|
| CA_ROTOR0_PY		| 0.07		| Motor 0 Y position (to be done also for other motors) |

> Note: check the motor ordering is correct with the PX4 reference airframe, as wrong ordering will result in drone not able to fly.
> To swap motor ordering change the values of the parameters PWM_MAIN_FUNC

Next step is to calibrate ESC: 
- *REMOVE BLADES FROM THE MOTORS*
- open QGroundControl and connect the drone
- navigate to `Vehicle Setup -> Power`
- follow instructions in the tab `ESC PWM Minimum and Maximum Calibration`

If needed the RC remote calibration and setup is also needed. To do it navigate to `Vehicle Setup -> Radio`. Also check the `Vehicle Setup -> Flight Modes` to setup the arm channel and flight modes channel. 

# PID Tuning 
**TODO**

# SD Card and System Startup (To test)

We can customize the commands that are executed during the startup of the drone, for example we can automatically execute `microdds_client`. To do this we have to modify directly files on the SD card used in the flight controller. 

[To execute modules](https://docs.px4.io/main/en/concept/system_startup.html) during the start of the system we have to add to the SD card the file `etc/extras.txt` that contains the command:
```bash
microdds_client start -t udp -p 8888 # avvio client XRCE con UDP
```


# Flight test
To check if the setup is working and fly the drone
- Disconnect from the computer
- Turn on the RC remote
- Connect the drone battery (do it after the RC remote is fully turned on)
- Wait for the drone to connect to the RC (signaled with a second pair of beeps after the initial one)
- Arm the drone pulling the throttle all the way down to zero and move the arm switch

After the arming all the motors should start spinning slowly, then slowly releasing the throttle lever should make the blades spin faster and make the drone takeoff. 