#include <msp430.h>

// ===== 參數：用 1MHz SMCLK 做 50Hz 伺服 PWM =====
// 週期 20ms => 20000 ticks (at 1MHz)
#define SERVO_PERIOD_TICKS   20000

// SG90 常用脈寬範圍（微秒 = ticks，因為 1MHz => 1 tick = 1us）
#define SERVO_MIN_US         1000   // ~0 deg
#define SERVO_MID_US         1500   // ~90 deg
#define SERVO_MAX_US         2000   // ~180 deg

static void delay_ms(unsigned int ms)
{
    // 這是簡單延遲：假設 MCLK 約 1MHz
    while (ms--) {
        __delay_cycles(1000);
    }
}

static void clock_init_1mhz_smclk(void)
{
    // MSP430FR 系列預設 DCO 約 1MHz，這裡保守做一次設定
    CSCTL0_H = CSKEY_H;
    CSCTL1 = DCOFSEL_0;                 // DCO ~ 1MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK; // ACLK=VLO, SMCLK/MCLK=DCO
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1; // 不分頻
    CSCTL0_H = 0;
}

static void servo_pwm_init_p12_ta11(void)
{
    // P1.2 設為 TA1.1 輸出
    P1DIR  |= BIT2;
    P1SEL0 |= BIT2;
    P1SEL1 &= ~BIT2;

    // Timer_A1: Up mode, SMCLK, PWM on CCR1
    TA1CCR0  = SERVO_PERIOD_TICKS - 1;          // 20ms 週期
    TA1CCTL1 = OUTMOD_7;                        // Reset/Set PWM
    TA1CCR1  = SERVO_MID_US;                    // 初始 1.5ms
    TA1CTL   = TASSEL__SMCLK | MC__UP | TACLR;  // SMCLK, Up mode
}

static void servo_set_us(unsigned int us)
{
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    TA1CCR1 = us;
}

static void servo_set_deg(unsigned int deg)
{
    // 線性映射：0..180 -> 1000..2000 us
    if (deg > 180) deg = 180;
    unsigned int us = SERVO_MIN_US + (unsigned long)(SERVO_MAX_US - SERVO_MIN_US) * deg / 180;
    servo_set_us(us);
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog

    clock_init_1mhz_smclk();
    servo_pwm_init_p12_ta11();

    PM5CTL0 &= ~LOCKLPM5;    // 解鎖 GPIO 設定
    
    // 伺服啟動後給它一點時間穩定
    delay_ms(500);

    while (1) {
        servo_set_deg(0);
        delay_ms(800);

        servo_set_deg(90);
        delay_ms(800);

        servo_set_deg(180);
        delay_ms(800);
    }
}
