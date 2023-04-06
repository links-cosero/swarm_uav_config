# Compilare per OmnibusF4SD 

|Component	|Version	|
| :--- 		| :--- 		|
| **Flight Controller** | OmnibusF4SD 	|
| **PX4** 				| v1.14.0b (main)|
| **OS**				| Ubuntu 22.04  |

Per scaricare la repository e fare correttamente il checkout alla versione main di PX4 eseguire i seguenti comandi:

```bash
git clone https://github.com/links-cosero/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout main
git submodule update --recursive
make distclean
```

E’ necessario inoltre installare tools aggiuntivi per la compilazione del codice. Per installare la toolchain su Ubuntu seguire [questa guida](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html). Fare un reboot oppure logout prima di compilare il codice dopo aver installato tutti i tools. 

Per compilare il codice per OmnibusF4SD eseguire il seguente comando

```bash
cd PX4-Autopilot
make omnibus_f4sd_default
```

Dopo la compilazione nella cartella `build/omnibus_f4sd_default` sarà contenuto il file `omnibus_f4sd_default.px4`che potrà essere caricato sul flight controller tramite QGroundControl. 

# Aggiungere moduli al firmware

Il codice sorgente è diviso in moduli che possono essere selezionati oppure no per essere compilati. Nella versione default per il target OmnibusF4SD la lista dei moduli è contenuta nella cartella `build/omnibus_f4sd_default/src/modules`: si può vedere che manca il modulo RTPS necessario per la comunicazione. 

Per creare una nuova configurazione dell'autopilota creare il file `boards/omnibus/f4sd/links.px4board`. Aprire quindi l'utility per la configurazione con il seguente comando:
```bash
make omnibus_f4sd_links boardconfig
```

Attraverso questo menu si potranno selezionare i moduli, driver e comandi da includere con l'autopilota. E' da tenere in considerazione che l'OmnibusF4SD ha una memoria a disposizione molto limitata quindi sono pochi i moduli che si possono includere. 
Per utilizzare la configurazione attualmente utilizzata creare il seguente file `boards/omnibus/f4sd/links.px4board`:
```
CONFIG_DRIVERS_GPS=n
CONFIG_DRIVERS_TELEMETRY_FRSKY_TELEMETRY=n
# CONFIG_EKF2_DRAG_FUSION is not set
# CONFIG_EKF2_GNSS_YAW is not set
# CONFIG_EKF2_RANGE_FINDER is not set
# CONFIG_EKF2_SIDESLIP is not set
CONFIG_MODULES_EKF2=y
```
Ed eseguire `make omnibus_f4sd_links`.

Nell'esempio sopra riportato nella configurazione utilizzata in laboratorio sono state fatte le seguenti modifiche rispetto alla versione `_default`:
- Rimozione driver GPS e Telemetria FRSKY
- Aggiunta modulo filtro Kalman EKF2

<!-- ```bash
CONFIG_MODULES_MICRORTPS_BRIDGE=y
CONFIG_MODULES_MICRODDS_CLIENT=y
```

Per fare questo si possono eseguire le seguenti istruzioni

```bash
cp boards/omnibus/f4sd/default.px4board boards/omnibus/f4sd/rtps.px4board
echo "CONFIG_MODULES_MICRORTPS_BRIDGE=y" >> boards/omnibus/f4sd/rtps.px4board
echo "CONFIG_MODULES_MICRODDS_CLIENT=y" >> boards/omnibus/f4sd/rtps.px4board
```

Ora sarà disponibile un nuovo target per la compilazione

```bash
make omnibus_f4sd_rtps
```

Provando ora a eseguire la compilazione potrebbe fallire per la mancanza di alcuni file. Alcune definizioni di messaggi di debug provocano l’errore e per rimuoverli si deve modificare il file `msg/tools/urtps_bridge_topics.yaml` commentando i topic di debug in questo modo: 

```yaml
...
rtps:
  # topic ID 1
  #- msg:     debug_array
  #  receive: true
  # topic ID 2
  #- msg:     debug_key_value
  #  receive: true
  # topic ID 3
  #- msg:     debug_value
  #  receive: true
  # ...
  #- msg:     debug_vect
  #  receive: true
  - msg:     offboard_control_mode
    receive: true
  - msg:     optical_flow
    receive: true
...
``` -->


# System startup in SITL

Per eseguire comandi all’avvio della simulazione con gazebo, modificare il file `ROMFS/px4fmu_common/init.d-posix/rcS`. Per esempio per avviare in automatico il client XRCE inserire alla riga 239 il comando, in questo modo: 

```bash
# ...
# Try to start the micrortps_client with UDP transport if module exists
if px4-micrortps_client status > /dev/null 2>&1
then
	. px4-rc.rtps
	microdds_client start -t udp -p 8888 # XRCE client startup
fi
# ...
```

<!-- ## timer_config.h (ancora da testare)
Un'altro modo per scambiare gli output della scheda è cambiare la definizione di dove sono mappati gli output PWM. E' possibile fare questo dal file `boards/omnibus/f4sd/src/timer_config.cpp`, in particolare la seguente sezione di codice riporta la posizione dove è possibile cambiare il collegamento tra timer e pin GPIO. 
```c++
/* ... */
constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel3}, {GPIO::PortB, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel4}, {GPIO::PortA, GPIO::Pin3}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel3}, {GPIO::PortA, GPIO::Pin2}),
};
/* ... */
```

## Geometry files (ancora da testare)
[Official documentation](https://docs.px4.io/v1.13/en/concept/geometry_files.html) 

Per creare un geometry file che specifica la configurazione dei motori necessaria per i droni nel Robotics Lab Links, va creato il seguente file  `src/lib/mixer/MultirotorMixer/geometries/quad_x_links.toml`. Il contenuto del file specifica la disposizione dei motori e degli attuatori
```toml
# Generic Quadcopter in X configuration

[info]
key = "4x_links"
description = "Quadcopter X configuration. Motor ordering changed to match Links drone"

[rotor_default]
direction = "CW"
axis      = [0.0, 0.0, -1.0]
Ct        = 1.0
Cm        = 0.05

[[rotors]]
name      = "front_right"
position  = [0.707107, 0.707107, 0.0]
direction = "CCW"

[[rotors]]
name     = "rear_right"
position = [-0.707107, 0.707107, 0.0]

[[rotors]]
name      = "rear_left"
position  = [-0.707107, -0.707107, 0.0]
direction = "CCW"

[[rotors]]
name     = "front_left"
position = [0.707107, -0.707107, 0.0]

```
Da notare che l'ordine con cui vengono dichiarati è importante per PX4. Semplicemte scambiando di ordine da un altro geometry file, questo file rispecchia la configurazione dei droni al Robotics Lab. 

Successivamente è necessario aggiungere al file `src/lib/mixer/MultirotorMixer/geometries/CMakeLists.txt`
```cmake
# ...
	quad_y.toml
	tri_y.toml
	twin_engine.toml
	quad_x_links.toml # file aggiunto
)
# ...

```
Una volta fatto questo si può creare un nuovo file mixer che utilizza la geometria precedentemente specificata. 

Per fare questo creare il file `ROMFS/px4fmu_common/mixers/quad_x_links.main.mix` che contiene: 
```
R: 4x_links

AUX1 Passthrough
M: 1
S: 3 5  10000  10000      0 -10000  10000

AUX2 Passthrough
M: 1
S: 3 6  10000  10000      0 -10000  10000

Failsafe outputs
The following outputs are set to their disarmed value
during normal operation and to their failsafe falue in case
of flight termination.
Z:
Z:
```

Importante è la prima riga dove si specifica la geometria da utilizzare. Una spiegazione più dettagliata sulla struttura del mixer file viene fornita nella [documentazione ufficiale](https://docs.px4.io/v1.13/en/concept/mixing.html). 

A questo punto è necessario solamente caricare durante l'esecuzione dell'autopilota il nuovo mixer al posto di quello caricato automaticamente. Per fare questo sulla OmnibusF4SD si dovrà creare sulla microSD il file `/etc/config.txt` contenente: 
```bash
set MIXER quad_x_links
``` 
Per caricare il mixer sulla simulazione su Gazebo, eseguire il seguente comando sulla shell di PX4 (da notare che questo causerà instabilità nel drone quando si tenta il decollo in quando il mixer file è sbagliato per il drone simulato):
```bash
mixer load /dev/pwm_output0 /absolute/path/to/quad_x_links.main.mix
``` -->