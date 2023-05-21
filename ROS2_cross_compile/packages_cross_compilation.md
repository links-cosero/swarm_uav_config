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

## Impostazione ambiente Docker
La compilazione di un pacchetto ROS 2 ha bisogno che sia presente l'[underlay](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#background) di ROS 2 **cross-compiled per il target desiderato** e NON di ROS 2 della macchina host. Per fare questo eseguire i seguenti step nel container utilizzato come ambiente:
```bash
# Inserire la cartella 'install' di ROS 2 nella cartella rootfs
mkdir -p /opt/rootfs/opt/ros2_humble
cp -r /opt/ros2_humble/install /opt/rootfs/opt/ros2_humble
cd /opt/rootfs/opt/ros2_humble
``` 
Impostare l'underlay di ROS2 c.c. (da fare ogni volta che si apre un nuovo terminale)
```bash
source /opt/rootfs/opt/ros2_humble/install/local_setup.bash
```
> **N.B.** si sta facendo il source di una installazione a 32-bit di ROS anche se la macchina host è 64-bit. Il comando source non dovrebbe dare nessun errore ma chiaramente provare a richiamare comandi ROS potrebbe non funzionare, non sono comunque necessari.

## Cross-compilation di un pacchetto
Copiare la cartella `workspace_cc` all'interno del workspace desiderato ed eseguire il seguente comando:
```bash
. workspace_cc/docker_cc.bash
```
Lo script in automatico eseguirà i seguenti passi:
- copia del workspace sul container
- copia dei file toolchain
- cross-compilation
- copia sul sistema host (nella cartella `./install_armhf`) della cartella  `install` compressa. 

Infine copiare la cartella compressa sul target e estrarla per effettuare l'installazione. 