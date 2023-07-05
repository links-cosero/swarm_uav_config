# Companion Board Setup
|Componente	|Versione	|
| :--- 		| :--- 		|
| **Flight Controller** | OmnibusF4SD 	|
| **PX4** 				| v1.14.0b		|
| **PX4-ROS2 bridge**	| microXRCE		|
| **ROS 2**				| Humble		|
| **System OS**			| Ubuntu 22.04	|
| **Companion Board**	| NanoPi Neo Air|
| **Comp. board SW**	| Ubuntu 20.04	|


# [Configurazione Porta Seriale](https://docs.px4.io/main/en/peripherals/serial_configuration.html)
Bisogna configurare che il server microdds usi come protocollo UART e porta sia quella assegnata a TELEM2, che di default dovrebbe essere la UART3 (/dev/ttyS2), come spiegato in [questa guida](https://docs.px4.io/main/en/hardware/serial_port_mapping.html)
Con il comando `make omnibus_f4sd_links boardconfig` si può utilizzare l'utility per configurare anche le porte seriali (vedere [questa guida](./compile_v1.13.md) per maggiori informazioni). La configurazione attuale è la seguente:
```
(/dev/ttyS2) URT6 tty port
()  GPS1 tty port
()  GPS2 tty port
()  GPS3 tty port
()  GPS4 tty port
()  GPS5 tty port
()  TEL1 tty port
(/dev/ttyS1) TEL2 tty port
()  TEL3 tty port
()  TEL4 tty port
()  TEL5 tty port
()  RC tty port
()  WIFI tty port
()  PPB (Pixhawk Payload Bus) tty port
```
From `boards/omnibus/f4sd/nuttx-config/include/board.h` list of available UARTs:
|UART	| Device 	|TX pin	|RX pin |Connector  |BaudRate| Note	|
| :--- 	|:---		|:---:	|:---:	|:--- 		|:---	|:---	|
|UART1	|/dev/ttyS0 |PA9	|PA10	|J5			|		|Probably radio		|
|USART3	|-			|PC10	|PC11	|J4			|		|Shared with SPI3, not usable?|
|UART4	|/dev/ttyS1 |PA0	|PA1	|			|57600	|MAVLINK or TEL2		|
|UART6	|/dev/ttyS2 |PC6	|PC7	|J10		|		|		|

L'unica porta seriale disponibile per XRCE è UART6. 

[Share the same port with mavlink and rtps](https://discuss.px4.io/t/sharing-one-port-between-mavlink-and-fastrtps-bridge/10247)

# Companion Computer Setup tramite adattatore USB-UART
Seguire i passi spiegati sulla [wiki](https://wiki.friendlyelec.com/wiki/index.php/NanoPi_NEO_Air) per installare sulla scheda SD il sistema operativo desiderato. Sono necessarie ulteriori configurazioni (testate solamente su Ubuntu 20.04): l'immagine scaricata di default crea delle partizioni con un numero troppo scarso di INodes, è necessario quindi riformattare la partizione `userdata` con un numero più alto di inodes, altrimenti essi verranno esauriti prima di esaurire lo spazio (8GB microSD). Per fare questo eseguire i seguenti comandi <u>prima di inserire la microSD nella NanoPi per la prima volta</u>:
```bash
# Identificare la partizione "userdata" con il comando
lsblk
# smontare la partizione
sudo umount /dev/my_userdata_partition
# formattare la partizione
sudo mke2fs -i 4096 -L userdata /dev/my_userdata_parition
```

## Serial USB terminal
Una volta compiuto questo step per avere una console utilizzare la porta di debug UART0 (spiegato nella guida). Nel nostro caso abbiamo utilizzato un adattatore USB `TTL-232R-3V3` con il seguente [datasheet](https://docs.rs-online.com/9110/0900766b8139de64.pdf). Collegare i fili nel seguente modo:
|TTL-232R-3V3	| NanoPi debug port	|
| :---: 		| :---: 		|
|VCC|VCC_5V|
|GND|GND|
|RXD|UART_TXD0|
|TXD|UART_RXD0|

Per avviare il collegamento seriale utilizzare il programma Putty:
- Navigare su `Session`
- Selezionare modalità `Serial`
- Impostare *Serial line* a `/dev/ttyUSB0` e *Speed* `115200`
- Cliccare su Open
- Premere invio può essere necessario per far partire la seriale se non si vede dell'output subito nella console

La password per l'utente `pi` è `pi`, per l'utente `root` è `fa` di default. 

## Connessione WiFi e SSH
Una volta aperta la connessione seriale (N.B. può essere necessario oltre alla porta seriale collegata al PC anche la porta microUSB) per connettere la scheda ad un access point (è solo supportato il WiFi 2.4 GHz) eseguire i seguenti comandi:
```bash
su root
nmcli dev
nmcli r wifi on
# Scan available networks
nmcli dev wifi
# Connect to a network (user will be prompted to insert the password)
nmcli dev wifi connect "SSID" --ask
```
Dopo la connessione la scheda si connetterà in automatico al WiFi configurato ad ogni avvio. 

Per stabilire una connessione SSH, ottenere l'indirizzo IP locale della scheda (dalla pagina di amministrazione del router per esempio) ed eseguire il comando dal proprio computer:
```bash
# Insert the actual IP address
ssh root@XXX.XXX.X.X
# Or
ssh pi@XXX.XXX.X.X
```

## Antenna
Sulla scheda è presente un connettore per una antenna esterna. Sperimentalmente senza antenna il segnale ricevuto anche da molto vicino è molto debole e anche la velocità di scaricamento dati è esageratamente lenta. Anche dalla pagina del router viene mostrato che il collegamento è molto debole: per esempio a 3 metri dal router il laptop viene ricevuto con una potenza di -60dBm, mentre la nanoPi con -90dBm di potenza. Provando a collegare una antenna la situazione migliora moltissimo e la scheda ha un comportamento "normale". 
Il connettore della antenna viene specificato sul sito di acquisto essere di tipo IPX.

# Companion Computer Setup tramite antenna APK
Scaricare da [qui](https://onedrive.live.com/?authkey=%21ACFNomemEVW6hxM&cid=1F5B36BBA3D56743&id=1F5B36BBA3D56743%2118033&parId=1F5B36BBA3D56743%219624&o=OneUp) il file immagine che installerà sulla companion Ubuntu 20.04 LTS 4.14.111.
Una volta scaricato usare Balena Etcher per flashare sulla scheda SD collegata al PC l'immagine.
Dopodiché andare in ```media/$USER/rootfs/etc/wpa_supplicant``` e creare un file ```wpa_supplicant.conf``` da terminale nel seguente modo:
```
sudo gedit wpa_supplicant.conf 
```
All'interno del file inserire la seguente:
```
network={
ssid="Your wifi network"
psk="Password"
proto=RSN
key_mgmt=WPA-PSK
pairwise=CCMP
auth_alg=OPEN
}
```
Cambiando opportunamente ssid e psk per collegarsi alla rete wifi. 
Dopodiché, andare in ```media/$USER/rootfs/etc/networks/interfaces``` e aggiungere le seguenti righe di codice:
```
auto wlan0
allow-hotplug wlan0
iface wlan0 inet dhcp
wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
iface default inet dhcp
```

Dodichè una volta che la procedura è andata a buon fine, ricollegare la scheda SD alla companion. Dopo aver collegato l'antenna alla companion, digitare il seguente comando da terminale per collegarsi board tramite ssh
```
ssh root@192.168.50.124
```
La password da digitare è ```fa```. Dopodiché la connessione dovrebbe avvenire con successo. Se ciò non si dovesse verificare, controllare nella pagina web del router se la connessione con la companion è avvenuta verificando se l'ip della companion è presente nella lista dei dispositivi collegati.

# Installazione ROS 2 Humble
## Compilazione source
Seguita [questa guida](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html). 
Fallisce perchè probabilmente manca abbastanza RAM (solo 500 MB disponibili).

## Cross compilation
Sequire [questi passaggi](../ROS2_cross_compile/install_tutorial.md) per utilizzare ROS2 sulla NanoPi NEO Air. 

<!-- ## Docker
La [repository ufficiale docker](https://hub.docker.com/_/ros/) di ROS ha disponibili le immagini di tutte le versioni, però solamente alcune sono disponibili per l'architettura `arm32v7` che è quella della NanoPi. 

Per installare la Docker Engine sulla NanoPi è stata seguita [questa guida](https://docs.docker.com/engine/install/ubuntu/). Dopo aver compilato una immagine di prova per arm32v7 sul PC, per caricarla ed eseguirla sulla NanoPi:
```bash
# Salvare immagine su un file
docker save -o ./my_image_name.tar
```

Dopo aver trasferito l'immagine dal PC alla NanoPi (con `scp` per esempio):
```bash
docker load -i my_image_name.tar
docker run my_image_name
```

Per creare un container da avviare più volte (diminuisce molto il tempo di avvio in particolare su NanoPi):
```bash
docker create --name my_container my_image_name
# Per avviarlo
docker start my_container 
``` -->


## Comunicazione MicroXRCEAgent
La companion computer è connessa al flight controller tramite un collegamento UART. Sulla companion computer è stato installato il software MicroXRCEAgent attraverso snap:
```bash
sudo apt update
sudo apt install snapd
sudo snap install micro-xrce-dds-agent --edge
```
Per avviare l'agent (fare l'accesso con utente root):
```bash
micro-xrce-dds-agent serial -D /dev/ttyS0 -b 1500000
```
Ora eseguendo ROS 2 sulla ground station sarà in grado di ricevere i topic pubblicati dall'agent(che viene eseguito sulla companion computer), verificabile con il comando `ros2 topic list`. 

Usando questa configurazione si può stabilire un collegamento tra ROS2 (eseguito sulla ground station) e PX4 sul flight controller, però non viene eseguita una istanza di ROS 2 sulla companion computer. 

### Creazione service per startup automatico di XRCE-Agent
Creare il file `/etc/systemd/system/xrce-agent.service`:
```
[Unit]
Description=XRCE agent for PX4
Wants=network-online.target
After=network-online.target
StartLimitIntervalSec=0
[Service]
Type=simple
Restart=always
RestartSec=1
User=root
ExecStart=/snap/bin/micro-xrce-dds-agent serial -D /dev/ttyS1 -b 1500000

[Install]
WantedBy=multi-user.target
```
Per attivarlo (solo la prima volta):
```bash
systemctl enable xrce-agent.service
```

A volte il l'agent parte in modo errato, non pubblicando i topic sulla rete, per risolvere riavviare il service con il comando:
```bash
systemctl restart xrce-agent.service
```