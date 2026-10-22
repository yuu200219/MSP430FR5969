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
## System Architecture Overview
### System Initialization
- Disables the Watchdog Timer and configures system clock SMCLK at 8MHz.
- Configures eUSCI_A0 (UART at 115200 Baud), eUSCI_B0 (SPI_1 for MS5607 sensor at 1MHz), and eUSCI_A1 (SPI_2 for SD Card at initial 400kHz).
- Configures Timer_A0 to generate an interrupt every 20ms.
- FRAM Integrity & Sensor Calibration
- Storage Setup using `f_mount()`
### Phase 1: Pre-Launch Standby
- Trigger Rate: Driven by the 20ms ISR accumulator to wake the CPU every 100ms (10Hz).
- Operations:
  - Reads raw temperature ($D_2$) and pressure ($D_1$) 10 times in sequence to calculate Mean Sea Level (MSL) altitude.
  - Averages these 10 samples to lock in the ground altitude reference (ground_alt).
  - No FRAM/SD logging occurs during this phase to prevent buffer corruption or index drift prior to flight.
  - Continuously monitors UART RX: when the ASCII character '0' (ignition command) is received, rocket_launched drops to 0, setting global_tick = 0 and launching Flight Mode.
### Phase 2: Active Flight Operations (rocket_launched == 0)
- Step 1: Raw Sensor Sampling
  - Sends `CMD_CONV_D1` (OSR 4096) with an 80,000-cycle delay (~10ms) to read the 24-bit raw pressure value (`D_1`).
  - Sends `CMD_CONV_D2` (OSR 4096) with an 80,000-cycle delay (~10ms) to read the 24-bit raw temperature value (`D_2`).
- Step 2: Digital Signal Processing (some processing algo. according to MS5607 datasheet)
- Step 3:Telemetry & FRAM Logging
  - FRAM Buffer: Encapsulates `global_tick`, `filtered_pressure`, `altitude`, and `temperature` into a `LogEntry` structure and stores it in `altimeter_log[write_index]`. The buffer wraps around automatically every 100 entries.
    ```=c
    typedef struct {
    
        uint32_t timestamp;
    
        float pressure;
    
        float altitude;
    
        uint16_t temperature;
    
    } LogEntry;
    ```
  - UART Transmission: Formats and transmits live CSV telemetry (AAA, timestamp, speed, pressure, altitude, rel_alt, temp) at 115200 baud to the ground station.
- Step 4: Parachute Deployment Decisions
  - Drogue Parachute Trigger:
    - Condition: Flight time is between 20s and 27s (200 <= global_tick < 270), filtered_speed $\le 20\text{ m/s}$, and Relay 1 feedback (P1.5) reports LOW.
    - Action: Drives P3.0 HIGH to trigger the drogue parachute deployment channel.
    - Fallback: If time exceeds 25s (global_tick >= 250) and Relay 1 has not actuated, re-asserts P3.0 HIGH.
  - Main Parachute Trigger:
    - Condition: Flight time exceeds 27s (global_tick > 270), rel_alt $\le 450\text{ m}$, and Relay 2 feedback (P4.3) reports LOW.
    - Action: Drives P3.4 HIGH to activate the main parachute deployment line.

