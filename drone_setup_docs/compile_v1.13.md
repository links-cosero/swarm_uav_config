# Compilare per OmnibusF4SD

Per scaricare la repository e fare correttamente il checkout alla versione 1.13 di PX4 eseguire i seguenti comandi:

```bash
git clone https://github.com/links-cosero/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout release/1.13
git submodule update --recursive
make distclean
```

E’ necessario inoltre installare tools aggiuntivi per la compilazione del codice. Per installare la toolchain su Ubuntu seguire [questa guida](https://docs.px4.io/v1.13/en/dev_setup/dev_env_linux_ubuntu.html). Fare un reboot oppure logout prima di compilare il codice dopo aver installato tutti i tools. 

Per compilare il codice per OmnibusF4SD eseguire il seguente comando

```bash
cd PX4-Autopilot
make omnibus_f4sd_default
```

Nella cartella `build/omnibus_f4sd_default` sarà contenuto il file `omnibus_f4sd_default.px4`che potrà essere caricato sul flight controller tramite QGroundControl. 

# Aggiungere moduli al firmware

Il codice sorgente è diviso in moduli che possono essere selezionati oppure no per essere compilati. Nella versione default per il target OmnibusF4SD la lista dei moduli è contenuta nella cartella `build/omnibus_f4sd_default/src/modules`: si può vedere che manca il modulo RTPS necessario per la comunicazione. 

Si può creare una nuova configurazione per il target omnibusf4sd creando un nuovo file all’interno della cartella `boards/omnibus/f4sd` e chiamarlo ad esempio `rtps.px4board` . All’interno di questo file viene specificata la configurazione del firmware e anche i moduli da includere: in questo file va copiato tutto il contenuto del file `default.px4board` e aggiunto: 

```bash
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
```

# System Startup

Si può personalizzare i comandi che vengono eseguiti all’avvio del drone come per esempio eseguire in automatico il `microdds_client`. I file necessari necessari possono essere modificati direttamente nel codice sorgente oppure aggiunti sulla scheda SD. 

[Per avviare ulteriori moduli](https://docs.px4.io/v1.13/en/concept/system_startup.html) all’avvio all’interno della scheda SD si deve aggiungere il file `etc/extras.txt`  contenente i comandi da eseguire:

```yaml
microdds_client start -t udp -p 8888 # avvio client XRCE
```

Si possono anche definire nuovi mixer file all’interno della cartella `etc/mixers/NAME_OF_MIXER` nella scheda SD. 

## System startup in SITL

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
# Ordine Motori
Il drone usato nel laboratorio links utilizza un ESC che non è compatibile con l'ordine dei motori indicato nell'airframe standard di PX4 per i quadrirotore. Per risolvere questo problema un primo tentativo  è stato fatto scambiando l'ordine dei fili di output dal Flight Controller all'ESC. L'ordine corretto è rappresentato nella seguente tabella
|       | Motore ESC attuale  | Motore ESC corretto | 
|:---   | :---:               | :---:               |
|PX4_M1 | M1                  | M1                  |
|PX4_M2 | M2                  | M4                  | 
|PX4_M3 | M3                  | M2                  | 
|PX4_M4 | M4                  | M3                  | 

La direzione di rotazione è corretta per tutti e 4 i motori sia prima che dopo lo scambio. 

## timer_config.h (ancora da testare)
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
```