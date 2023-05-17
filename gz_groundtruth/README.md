# PX4 External Odometry with Gazebo-Classic

### WORK IN PROGRESS: da rivedere ancora XRCE bridge
**Software utilizzati:** 

| Software | Version |
| --- | --- |
| PX4 | v1.11 |
| Gazebo | Classic v11 |
| ROS2 | Humble |
| Ubuntu | 22.04 LTS |
| gazebo_ros_pkgs | Version from apt |

# Setup gazebo_ros_pkgs

Per rendere disponibili informazioni sulla posizione da gazebo sono necessari dei plugins da eseguire durante la simulazione del drone. Come primo passo dopo aver installato Gazebo-Classic v11 bisogna installare il pacchetto che rende disponibili i plugins e altre funzionalità a ROS2 (tutorial completo a [questo link](https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)):

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

Il plugin necessario è chiamato “libgazebo_ros_p3d.so” e pubblica su un topic le informazioni sulla posizione di un certo oggetto nella simulazione. Per aggiungerlo bisonga modificare il file dentro usato per la simulazione `PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf` (valido quindi solo per il modello di drone Iris) aggiungendo il seguente codice: 

```xml
<plugin name="gazebo_ros_p3d" filename="libgazebo_ros_p3d.so">
	<ros>
	  <namespace>/iris</namespace>
	  <remapping>odom:=odom</remapping>
	</ros>
	<body_name>base_link</body_name>
	<frame_name>map</frame_name>
	<update_rate>50</update_rate>
	<xyz_offset>0 0 0</xyz_offset>
	<rpy_offset>0 0 0</rpy_offset>
	<gaussian_noise>0.01</gaussian_noise>
</plugin>
```

Io l’ho inserito alla riga 409.

Una volta fatto questo si può avviare la simulazione e si potrà vedere tra i topics di ROS2 la presenza del nuovo topic `/iris/odom` . Il plugin funziona avviando un nodo ROS che pubblica questo messaggio, il la repository con il codice sorgente è visualizzabile [a questo link](https://github.com/ros-simulation/gazebo_ros_pkgs). 

# Introduzione

Per simulare in ambiente Gazebo un sistema di Motion Capture come il Vicon, vengono presi direttamente da gazebo i dati sulla posizione attuale del drone e resi disponibili in ambiente ROS2 facendo finta siano output del sistema Vicon (spiegato nella sezione precedente). L’obiettivo è una volta che sono disponibili come topic su ROS2 possono essere inviati a PX4 sotto forma di `vehicle_visual_odometry` e integrati dall’autopilota nella stima della posizione corrente. Chiaramente data la loro precisione hanno un grande impatto sulla stima finale della posizione data dall’autopilota. Bisogna *impostare su PX4 anche alcuni parametri* come spiegato nell’ultima sezione di questo documento per far si che questi dati siano considerati nella stima della posizione. 

Per eseguire il presente workspace usare 3 terminali:

```python
MicroXRCEAgent udp4 -p 8888 # TODO: da cambiare con RTPS
```

```python
cd ~/path_to_PX4/PX4-Autopilot
make px4_sitl gazebo
```

```python
cd ~/path_to_repo/uav_config/gz_groundtruth
source install/local_setup.bash
ros2 launch px4_offboard offboard_position_control.launch.py
```

Sarà infine necessario impostare il drone in modalità offboard e armarlo (e.g. da QGroundControl)

Automaticamente si aprirà anche Rviz dove potremmo vedere la posizione stimata dall’autopilota (**linea verde**) e la posizione ricavata direttamente da Gazebo (**linea rossa**).

Per provare la differenza con e senza invio dei messaggi di visual_odometry commentare la seguente linea nel codice che si occupa dell’invio dei dati (linea 72 file mocap_node.py) e ricambiare i parametry di PX4 riguardo alla visual odometry di nuovo a default. 

**Attenzione: Le istruzioni riportate in seguito sono solamente una spiegazione del funzionamento del presente workspace, non servono ad eseguire il pacchetto.** 

# Pacchetto motion_capture (Work in progress)

Questo pacchetto è stato scritto interamente da capo, i compiti svolti al suo interno sono i seguenti:

- Trasformazione del messaggio di Gazebo in un reference frame corretto per ROS2
- Pubblicazione dei seguenti topic necessari per Rviz:
    - `/x500/pose` : orientamento e posizione attuale del drone
    - `/x500/path` : cronologia posizioni del drone per visualizzazione
- Task (ancora funzionante solo parzialmente) che si occupa di formattare correttamente i dati per PX4 e inviare il messaggio `vehicle_visual_odometry`.

Per la configurazione di Rviz ho sovrascritto il file `gz_groundtruth/src/px4-offboard/resource/visualize.rviz` dal menù di rviz facendo “salva con nome”, dopo aver aggiunto le nuove tracce.

# PX4 Visual Odometry (Work in progress)

I contenuti principali sono stati presi da [questa pagina di documentazione](https://dev.px4.io/v1.11_noredirect/en/ros/external_position_estimation.html). 

I dati di odometria visuale sono inviati a PX4 tramite il messaggio `vehicle_visual_odometry` in quanto è l'unico supportato dal filtro EKF2 (altrimenti va usato LPE). Vengono per ora inviati nel messaggio solo informazioni sulla posizione e orientamento e non sulle velocità. Attualmente ci sono ancora problemi sull’effettuare le trasformazioni corrette per l’orientazione, quindi per ora funziona solo l’invio della posizione. 

Il tipo del messaggio da inviare è **VehicleOdometry** e al suo interno sono impostate le seguenti informazioni.

- Posizione e relativa varianza
- Orientamento (quaternione) e relativa varianza
- Velocità e relativa varianza
- Velocità angolare e relativa varianza
- Timestamp del momento di campionamento

C’è la possibilità di inviare solo parzialmente i parametri, per esempio solo la posizione. Per fare questo gli altri sono impostati a NaN con il comando  

```python
velocity = float('NaN')
```

Importante impostare correttamente il parametro `timestamp` e `timestamp_sample`: il primo rappresenta il momento di invio del messaggio e il secondo il momento di campionamento. Se impostati non correttamente lo stimatore EFK2 ignorerà i campioni inviati. Per impostarli correttamente bisogna fare riferimento al tempo utilizzato a bordo di PX4 e non semplicemente il timestamp preso dal PC: come soluzione temporanea il nodo motion_capture copia il timestamp pubblicato sul topic `/fmu/out/vehicle_attitude` e lo inserisce nel messaggio `vehicle_visual_odometry`. 

## Parametri da impostare su PX4

Oltre a inviare i dati di odometria bisogna anche impostare alcuni parametri da QGroundControl per far si che la odometria visuale sia utilizzata come fonte primaria:

|**Parameter**| **Value**|
|:---|:---|
|EKF2_MULTI_IMU | 1|
|EKF2_IMU_CTRL | 1|
|EKF2_BARO_CTRL | Disabled|
|EKF2_EV_CTRL | 11 (Horizontal+Vertical+Yaw Positioning)|
|EKF2_EV_DELAY | 10.0ms (L’ho deciso io può essere anche diverso)|
|EKF2_HGT_REF | Vision|
|EKF2_EV_NOISE_MD | EV noise parameters|
|SYS_HAS_GPS | 0|
|GPS_CHECK | 240|
|GPS_CTRL | 4|