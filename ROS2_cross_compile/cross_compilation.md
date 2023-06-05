# ROS2 Cross Compilation per ARMHF
|	|	|
| :--- 		| :--- 		|
| **ROS2 version** 	| Humble 				|
| **Host OS**		| Ubuntu 20.04 (x86)	|
| **Target OS** 	| Ubuntu 20.04 (armhf)	|
| **Target Platform**		| NanoPi NEO Air|

## Introduzione
Per poter eseguire ROS2 sulla board NanoPi è necessaria la [compilazione del codice sorgente](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html), in quanto anche se è installato Ubuntu non è supportata l'installazione tramite file binari non avendo un processore a 64 bit. La board ha capacità molto limitate e non sembra riuscire a terminare la compilazione autonomamente di ROS 2. C'è la necessità quindi di compilare il codice su un altro computer e trasferire solamente il risultato della compilazione sulla NanoPi.
Questo tutorial si basa su varie guide tra cui:
- https://docs.ros.org/en/humble/How-To-Guides/Cross-compilation.html
- https://github.com/process1183/roomba-rpi/blob/master/docs/ros2_rpizw_build.md
- https://github.com/cyberbotics/epuck_ros2/tree/master/installation/cross_compile

## Ambiente per la cross compilation
Viene utilizzato un container Docker come ambiente per eseguire la cross compilation (vedere la [sezione docker](#utilizzo-di-docker) per ulteriori informazioni), in particolare avente lo stesso sistema operativo del target: è molto importante avere versioni delle librerie compatibili (in particolare GLIBC) quindi usando la stessa versione di Ubuntu si semplificano le cose. Inoltre utilizzando un container si evitano possibili conflitti con variabili e configurazioni magari già presenti sul sistema.

### Setup ambiente
Eseguire i seguenti comandi all'interno dell'ambiente di sviluppo
```bash
sudo apt update && sudo apt upgrade
```
Installare pacchetti necessari per compilare il codice ROS
```bash
sudo apt install -y build-essential git gawk texinfo bison file wget rsync wget tar python3-pip git cmake qemu-user-static python3-numpy
```
Installare librerie per Python
```bash
pip3 install rosinstall_generator colcon-common-extensions vcstool lark-parser
```

Installare la toolchain per armhf:
```bash
sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
```

Creare una cartella di lavoro e scaricare il codice sorgente di ROS2
```bash
mkdir -p /opt/rootfs /opt/ros2_humble/src
cd /opt/ros2_humble
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos
```

Creare il file `/opt/ros2_humble/build_ros.bash`:
```bash
touch ./src/ros-perception/COLCON_IGNORE \
    ./src/ros-visualization/COLCON_IGNORE \
    ./src/ros/ros_tutorials/turtlesim/COLCON_IGNORE \
    ./src/ros2/demos/image_tools/COLCON_IGNORE \
    ./src/ros2/demos/intra_process_demo/COLCON_IGNORE \
    ./src/ros2/rviz/COLCON_IGNORE \
    ./src/ros2/rosbag2/COLCON_IGNORE \
    ./src/eclipse-iceoryx/COLCON_IGNORE

export C_INCLUDE_PATH="/opt/rootfs/usr/include:/opt/rootfs/usr/include/arm-linux-gnueabihf:/opt/rootfs/usr/lib/arm-linux-gnueabihf"
export CPLUS_INCLUDE_PATH="/opt/rootfs/usr/include:/opt/rootfs/usr/include/arm-linux-gnueabihf:/opt/rootfs/usr/lib/arm-linux-gnueabihf"
export LD_LIBRARY_PATH=/opt/rootfs/lib/arm-linux-gnueabihf:/opt/rootfs/lib
export PYTHON_EXECUTABLE=/opt/rootfs/usr/bin/python3.8
if [ ! -e /lib/ld-linux-armhf.so.3 ]
then
    ln -s /opt/rootfs/lib/ld-linux-armhf.so.3 /lib/ld-linux-armhf.so.3
fi
if [ ! -e /usr/include/eigen3 ]
then ln -s /opt/rootfs/usr/include/eigen3 /usr/include/eigen3
fi


colcon build \
    $@ \
    --merge-install \
    --cmake-force-configure \
    --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=/opt/toolchain.cmake \
    -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
    -DPYTHON_EXECUTABLE=/opt/rootfs/usr/bin/python3.8 \
    -DPYTHON_SOABI=cpython-38-arm-linux-gnueabihf \
    -DBUILD_TESTING:BOOL=OFF
```


Creare il file `/opt/toolchain.cmake`:
```cmake
# https://github.com/cyberbotics/epuck_ros2/blob/master/installation/cross_compile/toolchain.cmake

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)
set(CMAKE_CROSSCOMPILING 1)

set(CMAKE_C_COMPILER /bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /bin/arm-linux-gnueabihf-g++)
set(CMAKE_SYSROOT /opt/rootfs)

# https://github.com/eProsima/Fast-DDS/issues/1262
set(CMAKE_CXX_FLAGS "-latomic")

set(CMAKE_FIND_ROOT_PATH /opt/ros2_humble/install)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(PYTHON_SOABI cpython-38-arm-linux-gnueabihf)
set(PYTHON_EXECUTABLE /opt/rootfs/usr/bin/python3.8)
set(CMAKE_THREADS_PTHREAD_ARG 0)
set(CMAKE_C_FLAGS -Wno-psabi)
set(CMAKE_CXX_FLAGS -Wno-psabi)

# https://github.com/foonathan/memory/pull/60
set(CMAKE_CROSSCOMPILING_EMULATOR /usr/bin/qemu-arm-static)
```

Infine è necessario fare una **copia dei file del target**. Questa operazione è necessaria per fare si che il compiler utilizzi le librerie del target e non quelle dell'host. I passi per svolgere questo step sono spiegati nella prossima sezione. 

## Setup target
Eseguire i seguenti comandi sulla NanoPi:
```bash
sudo apt update && sudo apt upgrade
```
```bash
sudo apt install -y \
    symlinks \
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
	python3-pip \
    python3-catkin-pkg
```
> N.B. è stata esclusa la libreria `libopencv-dev` siccome è molto grande e necessaria solamente per il pacchetto `ros-perception` che è stato disabilitato nella compilazione. Per abilitarlo rimuovere il relativo COLCON_IGNORE, installare questa libreria e ripetere tutta la compilazione. 

```bash
pip3 install empy lark packaging
```

Dopo aver installato tutte le librerie è necessario fare una copia delle cartelle `/lib` `/usr` `/etc` all'interno dell'host nella cartella `/opt/rootfs`. Questo può essere fatto utilizzando `rsync` dal sistema host:
```bash
rsync -rlR --safe-links <NanoPi_IP>:/{etc,lib,usr} /opt/rootfs/
```

## Compilazione
Dopo aver seguito tutti gli step precendenti per avviare la compilazione nell'ambiente di sviluppo:
```bash
cd /opt/ros2_humble
. build_ros.bash
```

Una volta terminata sarà presente la cartella `install` che conterrà tutti i file necessari per ROS 2 sulla NanoPi, quindi basterà copiare la cartella sulla nanopi e avviare ROS. Per fare la copia creare un archivio e trasferirlo con scp:
```bash
tar -cf - install/ | gzip > ros2_humble_install_armhf.tar.gz
scp ros2_humble_install_armhf.tar.gz <NanoPi_IP>:~/opt/ros2_humble
```
Seguire le [istruzioni per l'installazione](./install_tutorial.md) per gli step finali.

# Utilizzo di Docker
Per utilizzare il container docker relativo all'ambiente di sviluppo prima è stato creato il container dall'immagine (solo una volta) con il comando `docker create -it --name humble_cc_container humble_cc:latest` e poi tutte le volte viene avviato con `docker start -i humble_cc_container`, così facendo le modifiche fatte al container non sono perse tra i riavvii.
> In questa repository nella cartella `docker/humble_cc_env` viene fornito un Dockerfile che esegue già tutti gli step per la configurazione dell'ambiente e anche la copia del filesystem armhf. Può essere fatta partire la build con eseguendo il file `docker/humble_cc_env/build.bash` e avviare poi il container con `docker start -i humble_cc_container`. Una volta avviato basterà avviare la [compilazione di ROS](#compilazione). 

## Docker come emulatore armhf
Una possibilità è quella di usare un altro container docker che emula l'architettura armhf per generare i file necessari alla compilazione di ROS: questo approccio è molto comodo se non si ha accesso al target fisico e si è rivelato funzionante: ROS2 è stato compilato utilizzando questo metodo e il risultato ha eseguito senza problemi sulla NanoPi. 

Per utilizzare questo metodo semplicemente seguire allo stesso modo le istruzioni spiegati nella guida per il [setup del target](#setup-target) in un container armhf. Per copiare i file dal container armhf a quello per la compilazione è stato fatto un archivio delle cartelle desiderate e copiato sull'altro container:
```bash
# Da eseguire sul container armhf
symlinks -cr  /lib /usr /etc
tar -cvf - /lib /usr /etc | gzip > rootfs.tar.gz

# Da eseguire sul container per la compilazione dopo aver copiato l'archivio
cd /opt/rootfs
tar -xzf rootfs.tar.gz
```
> **ATTENZIONE!** \
> Copiare i file facendone un archivio ha il lato negativo di poter rompere alcuni symbolic links che sono utilizzati per trovare le librerie durante la compilazione e che devono essere aggiustati manualmente in seguito (vedere [troubleshooting](#troubleshooting)).

# Cross compilation pacchetti
Vedere la [seguente guida](./packages_cross_compilation.md) per più dettagli.

# Troubleshooting

- **Versione GLIBC non corretta**: usare lo stesso sistema operativo sia per la compilazione che per il target e aggiornare tutti i pacchetti. Verificare le versioni con i comandi `ld --version` `ldd --version`
- **Libreria non trovata durante la compilazione**: verificare di averla installata sul dispositivo target e di aver copiato il file system sul'host come descritto nella guida. Se l'errore persiste verificare che il file specificato non sia un link che indica un path inesistente e nel caso correggerlo. 
- **Libreria viene cercata sul dispositivo host**: creare un link simbolico che punta alla file corretto all'interno della cartella `/opt/rootfs`
- **Errore durante la compilazione di iceoryx**: il pacchetto è stato disabilitato in quanto genera un errore di compilazione. ROS funziona comunque anche senza. 
- **All'avvio di ROS2 errore riguardo a '_rclpy_pybind11.cpython-38-x86_64-linux-gnu.so'**: seguire le [istruzioni di installazione](./install_tutorial.md#unpack-binaries)
- **pybind11_vendor: Python 64-bit but compiler 32-bit**: verificare le variabili di ambiente relative al `PYTHON_EXECUTABLE`, `LD_LIBRARY` siano corrette sia nel file `toolchain.cmake` e `build_ros.bash`. 