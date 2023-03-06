# Groundtruth from Gazebo

# ROS2 Integration

[https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html)

## Installazione

Installazione di ignition-tools per poter usare il comando `ign topic -l` ed avere la lista di tutti i topic pubblicati di gazebo garden. Vogliamo trasformare i topic propri di Gazebo in topic leggibili e scrivibili anche da ROS2. Per fare questo ci serviremo del pacchetto `ros_gz_bridge`. Da notare che questo pacchetto installato con apt non funziona, nella pagina [github](https://github.com/gazebosim/ros_gz) viene infatti specificato che per la combinazione ROS2 Humble e Gazebo Garden esso va compilato con colcon. 

Per installare ignition-tools (se non già presente)

```bash
sudo apt install ignition-tools
```

Per verificare l’installazione avviare Gazebo e in un terminale eseguire il seguente comando che restitiusce i topic Gazebo disponibili. 

```bash
ign topic -l
```

Per l’installazione del bridge tra ros e gazebo:

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src
export GZ_VERSION=garden
git clone https://github.com/gazebosim/ros_gz.git -b ros2
```

Installare le dependencies (installare rosdep se non presente)

```bash
cd ~/ws
rosdep install -r --from-paths src -i -y --rosdistro humble
```

Compilare il workspace with colcon:

```bash
colcon build
source install/setup.bash # Notare che usando setup.bash invece che 
								# local_setup.bash il pacchetto rimane sempre disponibile
								# anche in futuro 
```

## Esecuzione

Per avviare il bridge tra ros e gazebo usare il seguente comando, dopo aver avviato la simulazione gazebo: 

```bash
ros2 run ros_gz_bridge parameter_bridge  /world/default/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V
```

Per collegare il tipo corretto di ROS al messaggio di Gazebo usare la tabella contenuta [in questo link](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge). Il comando precendente funziona avviando la simulazione `make px4_sitl gz_x500`. Si può verificare con `ros2 topic list` la presenza del nuovo topic nell’ambiente ros. 

Per leggere il topic ho scritto un piccolo listener che stampa sulla console il contenuto del topic `/world/default/pose/info` contentente le posizioni degli oggetti nell’ambiente Gazebo. Il secondo elemente rappresenta il drone x500. 

Per avviare il nodo dopo aver scaricato il codice e compilato il package:

```bash
ros2 run motion_capture mocap_node
```

# Workspace gz_groundtruth

Usando il pacchetto `px4_offboard` è stato inserito nella visualizzazione anche la posizione data da Gazebo. Nella cartella /src sono presenti tutti i pacchetti in modo da essere già pronti per essere compilati con colcon. E’ stato inserito anche il pacchetto `ros_gz` che è stato introdotto nella sezione precedente. 

### px4_offboard

E’ stato modificato il file `offboard_position_control.launch.py` aggiungendo anche l’avvio del nodo motion_capture in automatico. 

```python
# ...
Node(
    package='motion_capture',
    namespace='motion_capture',
    executable='mocap_node',
    name='mocap'
)
# ...
```

### motion_capture

Questo pacchetto è stato scritto interamente da capo, i compiti svolti al suo interno sono i seguenti:

- Avvio di ros_gz_bridge per ottenere la posizione del drone
- Trasformazione del messaggio di Gazebo in un reference frame corretto per ROS2
- Pubblicazione dei seguenti topic necessari per Rviz:
    - `/x500/pose` : orientamento e posizione attuale del drone
    - `/x500/path` : cronologia posizioni del drone per visualizzazione

Per eseguire questo pacchetto, dopo aver avviato la simulazione `make px4_sitl gz_x500` si esegue il comando `ros2 launch px4_offboard offboard_position_control.launch.p`, poi come nel tutorial iniziale bisogna ancora impostare manualmente la modalità offboard e armare il drone. Automaticamente si aprirà anche Rviz dove potremmo vedere la posizione stimata dall’autopilota (**linea verde**) e la posizione ricavata direttamente da Gazebo (**linea rossa**).  

Per la configurazione di Rviz ho sovrascritto il file `gz_groundtruth/src/px4-offboard/resource/visualize.rviz` dal menù di rviz facendo “salva con nome”, dopo aver aggiunto le nuove tracce.