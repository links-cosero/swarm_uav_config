# Companion Board Setup
|Componente	|Versione	|
| :--- 		| :--- 		|
| **Flight Controller** | OmnibusF4SD 	|
| **PX4** 				| v1.13.3		|
| **PX4-ROS2 bridge**	| microXRCE		|
| **ROS 2**				| Humble		|
| **Companion Board**	| NanoPi Neo Air (?)|
| **Comp. board SW**	| **?**				|


# [Configurazione Porta Seriale](https://docs.px4.io/v1.13/en/peripherals/serial_configuration.html)
Bisogna configurare che il server microdds usi come protocollo UART e porta sia quella assegnata a TELEM2, che di default dovrebbe essere la UART3 (/dev/ttyS2), come spiegato in [questa guida](https://docs.px4.io/v1.13/en/hardware/serial_port_mapping.html)
Con il comando `make omnibus_f4sd_rtps boardconfig` si può utilizzare l'utility per configurare anche le porte seriali. La configurazione attuale è la seguente:
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


> Mavlink su che porta comunica? \
> FRsky su che porta comunica? Probabilmente UART8 (/dev/ttyS3) penso si possa togliere
> 
# Companion Computer Setup
Come impostazione predefinita TEL2 è la porta per il companion computer, quindi nel nostro caso sarebbe UART4 (/dev/ttyS1). 

[Share the same port with mavlink and rtps](https://discuss.px4.io/t/sharing-one-port-between-mavlink-and-fastrtps-bridge/10247)