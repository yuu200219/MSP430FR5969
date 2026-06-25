# Plan: Implement Smart Sensitivity for Space Bar Control

## Background & Motivation
The user requested higher sensitivity for the space bar control (turning the relay off faster when released). However, reducing the static timeout counter caused flickering (the relay rapidly opening and closing). This happens because of the keyboard's "Delay Until Repeat" mechanism (usually ~500ms). If the timeout is set smaller than the OS delay, the timer will expire before the second character arrives.

## Scope & Impact
- Modify `main.c` to increase the timer resolution to 20ms.
- Implement an adaptive timeout state machine in the UART and Timer ISRs.
- Ensure the main program's 200ms sampling cycle is unaffected.
- No changes to `diskio.c` are needed, as `global_tick` will still increment every 200ms.

## Proposed Solution
1. **Increase Timer Resolution**: Change the Timer_A period from 200ms to 20ms.
2. **Sub-tick Counter**: Introduce a `sub_tick` counter in `Timer_A0_ISR`. The main loop logic (`timer_triggered`, `global_tick++`, and waking the CPU) will only execute every 10 ticks (200ms).
3. **Adaptive Timeout (`in_repeat_mode`)**:
   - Add a global variable `in_repeat_mode = 0`.
   - **First Press**: When a space is received, if `space_bar_timeout == 0`, set it to 35 (700ms) to bridge the OS repeat delay.
   - **Repeat Mode**: If a space is received and `space_bar_timeout > 0`, set `in_repeat_mode = 1` and reset the timeout to 6 (120ms) for high sensitivity.
   - **Release**: When `space_bar_timeout` hits 0 in the Timer ISR, turn the relay off and reset `in_repeat_mode = 0`.

## Implementation Plan

### Step 1: Global Variables
Update globals in `main.c`:
```c
volatile uint8_t timer_triggered = 0;
volatile uint32_t global_tick = 0;
volatile uint8_t space_bar_timeout = 0; // 0 = off, >0 = relayer on countdown
volatile uint8_t in_repeat_mode = 0;    // Tracks if we are in auto-repeat
float ground_alt = 0;
```

### Step 2: Timer Initialization
Update `system_init()` or `main()` where the timer is initialized:
```c
    // 修正：使用除以 8 分頻器，讓計數基準變成 1MHz
    // timerParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64; // 分頻器 = 64
    // timerParam.timerPeriod = 25000; // 200ms 存取一次資料
    
    // Change to 20ms resolution for higher sensitivity
    timerParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64; 
    timerParam.timerPeriod = 2500; // 125kHz * 0.02s = 2500 counts
```

### Step 3: ISR Updates
Replace the existing ISRs at the bottom of `main.c`:
```c
// --- 中斷服務 ---
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void) {
    static uint8_t sub_tick = 0;
    
    // Space bar timeout logic (runs every 20ms)
    if (space_bar_timeout > 0) {
        space_bar_timeout--;
        if (space_bar_timeout == 0) {
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN2); // Turn off relayer
            in_repeat_mode = 0;
        }
    }

    // 200ms task logic (runs every 10 ticks)
    if (++sub_tick >= 10) {
        sub_tick = 0;
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        timer_triggered = 1;
        global_tick++;
        __bic_SR_register_on_exit(LPM0_bits); // 喚醒 CPU
    }
}

#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
    switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG: {
            uint8_t rx_val = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
            if (rx_val == ' ') { // Space bar pressed
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN2); // Turn on relayer
                
                if (space_bar_timeout > 0) {
                    in_repeat_mode = 1; // Second char arrived, we are holding the key
                }

                if (in_repeat_mode) {
                    space_bar_timeout = 15; // 300ms (fast release response)
                } else {
                    space_bar_timeout = 35; // 700ms (prevents flicker during initial 500ms delay)
                }
            }
            break;
        }
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
    }
}
```