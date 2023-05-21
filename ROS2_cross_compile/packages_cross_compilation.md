# Packages Cross Compilation
|	|	|
| :--- 		| :--- 		|
| **ROS2 version** 	| Humble 				|
| **Host OS**		| Ubuntu 20.04 (x86)	|
| **Target OS** 	| Ubuntu 20.04 (armhf)	|
| **Target Platform**		| NanoPi NEO Air|

# Introduzione
Questa guida è basata sull'ambiente preparato in precedenza per compilare ROS 2 ([questa guida](./cross_compilation.md)). Si utilizza lo stesso approccio visto in precedenza: sul container docker si prepara l'ambiente necessario e si effettua la cross compilation e poi si copia la cartella `install` generata sulla board target. 

- https://docs.ros.org/en/humble/How-To-Guides/Cross-compilation.html#cross-compiling-against-a-pre-built-ros-2

## Impostare ROS 2
La compilazione di un pacchetto ROS 2 ha bisogno che sia presente l'[underlay](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#background) di ROS 2 **cross-compiled per il target desiderato** e NON di ROS 2 della macchina host. Per fare questo eseguire i seguenti step:
```bash
# Place the install folder in the rootfs folder
mkdir -p /opt/rootfs/opt/ros2_humble
cp -r /opt/ros2_humble/install /opt/rootfs/opt/ros2_humble
cd /opt/rootfs/opt/ros2_humble
``` 
Impostare l'underlay di ROS2 c.c. (da fare ogni volta che si apre un nuovo terminale)
```bash
source /opt/rootfs/opt/ros2_humble/install/local_setup.bash
```
> **N.B.** si sta facendo il source di una installazione a 32-bit di ROS anche se la macchina host è 64-bit. Il comando source non dovrebbe dare nessun errore ma chiaramente provare a richiamare comandi ROS potrebbe non funzionare, non sono comunque necessari.

## Impostazione workspace
Copiare la cartella del workspace desiderato sul container, non è importante la posizione, per esempio inserirla nella cartella `/opt/rootfs`. 

Per avviare la compilazione viene utilizzato `colcon` e siccome vanno impostate molte variabili sono raccolte nello script `cross_compile_ws.bash`. E' simile allo script usato per compilare ROS 2, in particolare viene impostata la variabile `ROS2_INSTALL_PATH` che definisce la posizione di ROS2.  

Eseguire la compilazione
```bash
. workspace_cc/cross_compile_ws.bash
```

Comprimere la cartella install:
```bash
tar -cvf - install/ | gzip > my_pkg_install.tar.gz
```

Infine copiarla sul target.