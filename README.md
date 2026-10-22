# MSP430FR5969 Rocket Sensor Project
This is an embedded project for Rocket Taiwan Cup, RTC 2026.
Using TI's board MSP430FR5969 to implement Relay Controlling, Altimeter Module MS5607 Data Collecting, and SD Card Storing.
## MSP430FR5969
- Document
  https://www.ti.com/lit/ds/symlink/msp430fr5969.pdf
- Goal
  This MSP430 board focus on the sensor data collecting and motor engine triggering, which using the relay to acheive. 
  Cause MSP430 has less performance, the rest feature of the rocket functionality, we put it on another FPGA board. Of course, we implement redundancy between the MSP430 and the FPGA, but we don't discuss here.
You can get the basic information and regulation of the MSP430 in above link.
## Environment Setting
- **Code Composer Studio**。
  https://www.ti.com/tool/CCSTUDIO
## Relay Controlling and communication between FPGA(main computing board)
  We receive rocket launch signal from FPGA through `rx`.
  - 0: launch signal
  We using GPIO to activate the relay
  - P3.0: trigger motor 
  - P1.5: motor status
## Altimeter Module MS5607
- Document
  https://docs.rs-online.com/9a1d/0900766b8124211e.pdf

## 
