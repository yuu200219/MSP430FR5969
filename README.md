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
- **Clock & Hardware**: Disables Watchdog Timer; configures SMCLK at 8MHz.
- **Peripherals**:
  - `eUSCI_A0`: UART at 115200 Baud (Telemetry broadcast).
  - `eUSCI_B0`: SPI_1 for MS5607 Altimeter at 1MHz.
  - `eUSCI_A1`: SPI_2 for SD Card (Initial 400kHz for initialization, then boosted to 4MHz/8MHz).
  - `Timer_A0`: Configured for a **10ms (100Hz)** base hardware interrupt rate.
- **FRAM Integrity & Calibration**: Verifies persistent magic number (`0xABCD`). If uninitialized, reads MS5607 PROM coefficients ($C_1 \dots C_6$) and saves them to FRAM.
- **Storage Initialization**: Mounts FatFs filesystem using `f_mount()` and dynamically creates/opens the next sequential log file (e.g., `FL_00001.CSV`).

---

### Phase 1: Pre-Launch Standby (`rocket_launched == 1`)
- **Execution**: Runs in the foreground upon boot.
- **Ground Calibration**:
  - Takes 10 sequential MS5607 samples (with 9ms conversion delays).
  - Averages calculated MSL altitudes to set the reference ground altitude (`ground_alt`).
- **Ignition Sensing**: Monitors UART RX for ASCII `'0'` (Ignition command from FPGA/Ground Station). Upon receipt, sets `rocket_launched = 0`, resets `global_tick = 0`, and engages Flight Mode.

---

### Phase 2: Active Flight Operations (`rocket_launched == 0`)
Driven by a non-blocking state machine (`cycle_tick`) inside the **10ms Timer ISR**, executing a full sampling and decision cycle every **100ms (10Hz)**:

- **Step 1: Non-blocking Sensor Sampling (MS5607)**
  - **Cycle 0 (0ms)**: Sends pressure conversion command (`CMD_CONV_D1`, OSR 4096).
  - **Cycle 1 (10ms)**: Reads 24-bit raw pressure ($D_1$) via SPI; sends temperature conversion command (`CMD_CONV_D2`, OSR 4096).
  - **Cycle 2 (20ms)**: Reads 24-bit raw temperature ($D_2$) via SPI.

- **Step 2: Signal Processing & Calculations**
  - **MS5607 Compensation**: Computes 2nd-order temperature-compensated pressure ($P$) and altitude ($H$).
  - **Relative Altitude**: Evaluates $\text{rel\_alt} = H - \text{ground\_alt}$.
  - **Velocity Estimation**: Calculates vertical velocity ($\Delta H / \Delta t$) smoothed through a 5-sample moving average window (`filtered_speed`).
  - **Altitude Voting Filter**: Maintains a 10-sample window for $\text{rel\_alt} \le 450\text{ m}$ validation.

- **Step 3: Double-Buffered Storage & Telemetry Broadcast**
  - **FRAM Double Buffering**: Writes timestamp, pressure, altitude, and temperature into `altimeter_log[active_buf][write_index]`.
  - **SD Flush**: When a buffer fills 50 records (~5s), it toggles `active_buf` and flags the Main Loop to write the filled buffer to the SD card asynchronously.
  - **UART Output**: Passes updated data to the Main Loop to transmit live CSV telemetry (`AAA, tick, speed, pressure, alt, rel_alt, temp`).

- **Step 4: Parachute Deployment & Pulse Auto-Cutoff**
  - **Drogue Parachute Trigger**:
    - **Condition**: Flight time between 20s and 25s (`200 <= global_tick < 250`), `filtered_speed` $\le 40\text{ m/s}$, and not previously triggered.
    - **Action**: Drives `P3.0` HIGH and sets `parachute_pulse_timer = 350` (**3.5-second pulse**).
  - **Main Parachute Trigger**:
    - **Condition**: Flight time exceeds 80s (`global_tick > 800`), at least 8 of the last 10 samples report $\text{rel\_alt} \le 450\text{ m}$, and not previously triggered.
    - **Action**: Drives `P3.4` HIGH and sets `parachute_pulse_timer = 1000` (**10-second pulse**).
  - **Safety Auto-Cutoff**: Decrements `parachute_pulse_timer` every 10ms. When it reaches zero, automatically pulls `P3.0` (and `P3.4`) LOW to prevent coil burn/battery drain.
