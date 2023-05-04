# ROS2 Humble per armhf
## Setup target
**Testato su Ubuntu 20.04** 

Comandi da eseguire sulla piattaforma target (obbligatori)
```bash
sudo apt update && sudo apt upgrade
```
Installare
```bash
sudo apt install \
    bison \
    cmake \
    curl \
    libasio-dev \
    libbullet-dev \
    libcunit1-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    liblog4cxx-dev \
    libtinyxml2-dev \
    python3-dev \
    python3-netifaces \
    python3-numpy \
    python3-setuptools \
    python3-yaml \
	python3-pip
```

Installare pacchetti python

```bash
pip3 install empy lark packaging
```

## Unpack binaries
Copiare il file `ros2_humble_install_armhf_v3.tar.gz` nella cartella del target `/opt/ros2_humble`. 
Eseguire:
```bash
cd /opt/ros2_humble
tar -xzf ros2_humble_install_armhf_v3.tar.gz
```

Rinominare il file
```bash
cd install/lib/python3.8/site-packages/rclpy/
mv _rclpy_pybind11.cpython-38-x86_64-linux-gnu.so _rclpy_pybind11.cpython-38-arm-linux-gnueabihf.so
```

## Test con esecuzione demo
Aprire due terminali ed eseguire in entrambi
```bash
source /opt/ros2_humble/install/local_setup.bash
```
Avviare i due nodi talker e listener
```bash
ros2 run demo_nodes_py listener
```

```bash
ros2 run demo_nodes_cpp talker
```
