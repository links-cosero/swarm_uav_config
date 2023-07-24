# VICON Motion Capture

Tramite l'SDK fornito da Vicon si è costruito un nodo ROS 2 the legge le informazioni sulla posizione degli oggetti e li pubblica su un topic. L'SDK è stato scompattato nella cartella `./src/vicon_ros2_cpp/include`

Per fare correttamente la compilazione dell'eseguibile ROS 2 specificare in CMakeLists di fare il link con la libreria ViconSDK
```
add_executable(vicon_v2
    src/viconv2.cpp)

target_link_libraries(vicon_v2 libViconDataStreamSDK_CPP.so)
ament_target_dependencies(vicon_v2 rclcpp geometry_msgs px4_msgs tf2 tf2_ros)
target_include_directories(vicon_v2 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $<INSTALL_INTERFACE:include>)
target_compile_features(vicon_v2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

```
Per eseguire il nodo utilizzare il comando:
```bash
source install/local_setup.bash
ros2 run vicon_ros2_cpp vicon_v2
```
il nodo `vicon_v2` utilizza la classe `ViconDataStreamSDK::CPP::RetimingClient` che ha performance nettamente superiori in termini di frequenza costante dei  campioni rispetto al client di base `ViconDataStreamSDK::CPP::Client` utilizzato nel nodo `vicon`. Il nodo `vicon_v2` ha una frequenza più precisa ma presenta alcuni outliers nelle misure fornite: vengono filtrare dal filtro EKF2 di PX4 e non presentano alcun problema. 

# Pyvicon_datastream (non utilizzato)

Il systema vicon viene utilizzato per ricevere informazioni sulla posizione e orientamento dei vari oggetti all'interno della gabbia. I dati vengono estratti utilizzando un [wrapper python](https://pypi.org/project/pyvicon-datastream/) dell'SDK originale in CPP. 
In questa libreria non tutte le funzioni sono state tradotte in python, solamente le principali. La reference delle funzioni è la stessa della classe in CPP. 

## Esempio di utilzzo
L'indirizzo IP del VICON sulla rete `ROBOTICS_LAB` è 192.168.50.56  .

```python
from pyvicon_datastream import tools

VICON_TRACKER_IP = "192.168.50.56"
OBJECT_NAME = "drone1"

mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
while(True):
    position = mytracker.get_position(OBJECT_NAME)
    print(f"Position: {position}")
    time.sleep(0.5)
```

L'object name si riferisce agli oggetti creati dal software VICON tracker. Gli oggetti dati dalla funzione `get_position()` hanno la seguente struttura. Le convenzioni utilizzate degli angoli e delle posizioni sono presenti sul manuale. 
```python
(
	latency (seconds), 
	frame_number, 
	[
		subject_name,
		segment_name,
		pos_x   (millimiters),
		pos_y   (millimiters),
		pos_z   (millimiters),
		euler_x (rad),
		euler_y (rad),
		euler_z (rad)
	]
)

```

## Frame di riferimento
- **Global reference frame**: viene impostato con la bacchetta per la calibrazione dal software VICON tracker. N.B. i dati di traslazione sono espressi in millimetri. Possono venire impostate le direzioni degli assi in modo arbitrario (purchè destrorso) utilizzando il seguente comando: 
```python
from pyvicon_datastream import tools
import pyvicon_datastream as pv

self.vicon_tracker = tools.ObjectTracker(self.vicon_tracker_ip)
self.vicon_tracker.vicon_client.set_axis_mapping(
		pv.Direction.Forward,
		pv.Direction.Right,
		pv.Direction.Down)
```
In questo caso è stato impostato una convenzione di tipo FRD.

- **Local reference frame**: Il frame locale viene assegnato al momento della creazione dell'oggetto. E' importante allineare inizialmente l'oggetto agli assi nella direzione desiderata, in quanto il frame locale sarà assegnato inizialmente "parallelo" al frame globale. 
