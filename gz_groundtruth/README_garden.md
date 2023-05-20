# PX4 External Odometry with Gazebo

**Software utilizzati:** 

| Software | Version |
| --- | --- |
| PX4 | v1.14-beta2 |
| Gazebo | Garden |
| ROS2 | Humble |
| Ubuntu | 22.04 LTS |
| ros_gz_bridge | Compiled from main branch |

# Introduzione

Per simulare in ambiente Gazebo un sistema di Motion Capture come il Vicon, vengono presi direttamente da gazebo i dati sulla posizione attuale del drone e resi disponibili in ambiente ROS2 facendo finta siano output del sistema Vicon. L’obiettivo è una volta che sono disponibili come topic su ROS2 possono essere inviati a PX4 sotto forma di `vehicle_visual_odometry` e integrati dall’autopilota nella stima della posizione corrente. Chiaramente data la loro precisione hanno un grande impatto sulla stima finale della posizione data dall’autopilota. Bisogna *impostare su PX4 anche alcuni parametri* come spiegato nell’ultima sezione di questo documento per far si che questi dati siano considerati nella stima della posizione. 

I passi da compiere saranno quindi 2: estrarre da gazebo i dati, formattarli correttamente e inviarli al drone. 

Per eseguire il presente workspace usare 3 terminali:

```python
MicroXRCEAgent udp4 -p 8888
```

```python
cd ~/path_to_PX4/PX4-Autopilot
make px4_sitl gz_x500
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

# Scaricamento dati da Gazebo

All’interno dell’ambiente gazebo sono presenti dei topic (non visualizzabili da ROS) e vogliamo copiare il contenuto del topic  `/world/default/pose/info`  su un topic visualizzabile da ROS. Questo topic contiene le informazioni sulla posizione di tutti gli oggetti nella simulazione Gazebo: in particolare noi saremo interessati al **secondo elemento del vettore restituito** in quanto rappresenta la posizione del drone. N.B. ho trovato che è il secondo elemento in modo sperimentale guardando i dati che dava in output. 

Per visualizzare i topic di Gazebo avremmo bisogno di `ignition-tools`:

```bash
sudo apt install ignition-tools
# Per visualizzare tutti i topic disponibili
ign topic -l
```

Il collegamento con ROS sarà gestito da `ros_gz_bridge` :  questo pacchetto installato con apt non funziona, nella pagina [github](https://github.com/gazebosim/ros_gz) viene infatti specificato che per la combinazione ROS2 Humble e Gazebo Garden esso va compilato con colcon. 

Esso va inserito come package nel workspace dove viene utilizzato, altrimenti non funziona sempre (provato sperimentalmente). **N.B. Nella presente repository il workspace ha già all’interno tutti i package necessari, quindi i passi seguenti sono un tutorial solo per costruire un eventuale nuovo workspace.** 

Per scaricare la repository nel workspace:

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src
export GZ_VERSION=garden
git clone https://github.com/gazebosim/ros_gz.git -b ros2
```

Installare le dependencies (installare rosdep se non presente)

```bash
cd ~/workspace
rosdep install -r --from-paths src -i -y --rosdistro humble
```

Compilare il workspace with colcon:

```bash
colcon build
source install/local_setup.bash
```

## Utilizzo ros_gz_bridge

Per avviare il bridge tra ROS e Gazebo usare il seguente comando, dopo aver avviato la simulazione gazebo:

```bash
ros2 run ros_gz_bridge parameter_bridge  /world/default/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V
```

Per collegare il tipo corretto di ROS al messaggio di Gazebo usare la tabella contenuta [in questo link](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge). Il comando precendente funziona avviando la simulazione `make px4_sitl gz_x500`. Si può verificare con `ros2 topic list` la presenza del nuovo topic nell’ambiente ROS.

Nel codice questo comando viene eseguito automaticamente all’avvio del nodo `mocap_node` eseguendo le seguenti istruzioni python:

```python
import subprocess

def launch_ros_gz_bridge(topic:str, ros_type:str, gz_type:str):
        """ Launch subprocess to bridge Gazebo and ROS2 """
        argument = f"{topic}@{ros_type}[{gz_type}"
        command = "ros2 run ros_gz_bridge parameter_bridge "+argument
        subprocess.Popen(command.split(' '))

# Richiamo funzione
launch_ros_gz_bridge(
	"/world/default/pose/info",
	"geometry_msgs/msg/PoseArray",
	"gz.msgs.Pose_V")
```

Si può notare che ora è disponibile con il comando `ros2 topic list` il nuovo topic creato da ros_gz_bridge. 

# Pacchetto motion_capture

Questo pacchetto è stato scritto interamente da capo, i compiti svolti al suo interno sono i seguenti:

- Avvio di ros_gz_bridge per ottenere la posizione del drone
- Trasformazione del messaggio di Gazebo in un reference frame corretto per ROS2
- Pubblicazione dei seguenti topic necessari per Rviz:
    - `/x500/pose` : orientamento e posizione attuale del drone
    - `/x500/path` : cronologia posizioni del drone per visualizzazione
- Task (ancora funzionante solo parzialmente) che si occupa di formattare correttamente i dati per PX4 e inviare il messaggio `vehicle_visual_odometry`.

Per la configurazione di Rviz ho sovrascritto il file `gz_groundtruth/src/px4-offboard/resource/visualize.rviz` dal menù di rviz facendo “salva con nome”, dopo aver aggiunto le nuove tracce.

# PX4 Visual Odometry

I contenuti principali sono stati presi da [questa pagina di documentazione](https://docs.px4.io/main/en/ros/external_position_estimation.html) 

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

Se non si può accedere a QGC questi valori si possono impostare da dentro la SD_CARD creando mettendoli dentro il file extra.txt dentro la cartella etc scrivendo ciascun parametro con il suffisso param set.

|**Parameter**| **Changed value**|**Default value**|
|:---|:---|:---|

|EKF2_BARO_CTRL | 0 (Disabled)| | 1 (Enabled)| 
|EKF2_EV_CTRL | 3 (Horizontal+Vertical)| 15 (All flags selected)|
|EKF2_EV_DELAY | 10.0 ms| 0.0 ms|
|EKF2_HGT_REF | 3 (Vision)| 1 (GPS)|

# Analisi dei Log di sistema
Uno strumento importante per capire nel dettaglio il comportamento dell'autopilota è l'analisi del file di log creato alla fine di ogni simulazione in automatico. Al suo interno si trovano registrate molte informazioni, in particolare anche quelle riguardo allo stimatore EKF2. 
Il programma che ho utilizzato per visualizzare i dati è [PlotJuggler](https://github.com/facontidavide/PlotJuggler). Per installarlo si può usare semplicemente snap:
```bash
sudo snap install plotjuggler
```
Per ottenere il file di log (.ulg) si deve utilizzare QGroundControl. Avviare QGroundControl e l'autopilota, poi andare nella sezione Analyze Tools -> Log Download . Qui selezionare il file desiderato e scaricarlo con il tasto Download (bottone refresh forse necessario per visualizzare la lista dei log). Una volta scaricato è possibile caricarlo su PlotJuggler (da interfaccia grafica dopo averlo avviato)

Una volta caricati i dati sarà possibile visualizzare dei plot temporali di tutte le informazioni registrate (drag and drop della variabile sul plot per visualizzarla), un esempio di dati relativi alla visual odometry:
- `vehicle_visual_odometry` : contiene le informazioni sui dati inviati a PX4
- `estimator_aid_src_ev_pos.00/fused` : indica che le informazioni sulla visual odometry sono utilizzate nella sensor fusion. 
- `estimator_aid_src_ev_hgt.00/observation`
- `estimator_aid_src_ev_pos.00/observation0`
