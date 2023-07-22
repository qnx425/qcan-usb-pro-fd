# !ACHTUNG! READ FIRST [COPYRIGHT ISSUE](https://github.com/moonglow/pcan_pro_x/issues/16)

## QPCAN USB PRO FD firmware for NUCLEO-H743ZI2 board

Target hardware: NUCLEO-H743ZI2 board

Pinout:  
|PIN/PINS|DESCRIPTION|  
| ------ | ------ |  
|PD0/PD1 |CAN1 RX/TX|  
|PB12/PB6|CAN2 RX/TX|  
|PA11/PA12|USB FS DM/DP|  

Works with [PEAK PCAN-View][pvw] in Windows

Before connecting the device to the USB port, the following must be done in the PCAN-View window:  
1. All transmissions must be disabled.  
2. PCAN-View program must be disconnected from PCAN hardware.  

![](/images/1.png)  
  
![](/images/2.png)  

Otherwise for unknown reasons there is a failure in USB connection.  
Inside pcan_protocol_process_data() function fields pmsg->size and pmsg->type are zero.

Toolchain: GNU Arm Embedded Toolchain

It is possible to build firmware with STM32CubeIDE project (I used version 1.12.1) or make.  

---

Limitations.  
1. Clock Frequency must be 80 MHz.  
2. Error Generator does not work.  

License
----

WTFPL

[pvw]: <https://www.peak-system.com/PCAN-View.242.0.html?&L=1>
