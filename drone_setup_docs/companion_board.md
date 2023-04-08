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

# Companion Computer Setup
Seguire i passi spiegati sulla [wiki](https://wiki.friendlyelec.com/wiki/index.php/NanoPi_NEO_Air) per installare sulla scheda SD il sistema operativo desiderato. Una volta compiuto questo step per avere una console utilizzare la porta di debug UART0 (spiegato nella guida). Nel nostro caso abbiamo utilizzato un adattatore USB `TTL-232R-3V3` con il seguente [datasheet](https://docs.rs-online.com/9110/0900766b8139de64.pdf). Collegare i fili nel seguente modo:
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

La password per l'utente `pi` è `pi`, per l'utente `root` è `fa` di default. 

## Connessione WiFi e SSH
Una volta aperta la connessione seriale per connettere la scheda ad un access point (è solo supportato il WiFi 2.4 GHz) eseguire i seguenti comandi:
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