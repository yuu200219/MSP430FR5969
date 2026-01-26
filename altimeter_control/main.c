#include "driverlib.h"
#include <msp430.h>
#include <math.h>
#include <stdio.h>

// --- MS5607 指令集 ---
#define CMD_RESET       0x1E
#define CMD_ADC_READ    0x00
#define CMD_PROM_RD     0xA0
#define CMD_CONV_D1     0x44 // OSR 1024 (Pressure)
#define CMD_CONV_D2     0x54 // OSR 1024 (Temperature)

// --- 資料結構 ---
typedef struct {
    uint32_t timestamp;
    float pressure;
    float altitude;
    uint16_t temperature;
} LogEntry;

// --- FRAM 持久化資料 ---
#pragma PERSISTENT(altimeter_log)
LogEntry altimeter_log[100] = {0};

#pragma PERSISTENT(write_index)
unsigned int write_index = 0;

#pragma PERSISTENT(ms5607_coeffs)
uint16_t ms5607_coeffs[7] = {0}; // 校準係數

#pragma PERSISTENT(fram_initialized)
uint16_t fram_initialized = 0; // 用來存 Magic Number，例如 0xABCD

// --- 全域變數 ---
volatile uint8_t timer_triggered = 0;
volatile uint16_t global_tick = 0;

// --- 函式宣告 ---
void check_fram_integrity(void);
void system_init(void);
void ms5607_reset(void);
void ms5607_read_prom(void);
uint32_t ms5607_get_raw(uint8_t conv_cmd);
void calculate_and_log(uint32_t D1, uint32_t D2);
void export_data(void);
void UART_print(char* s);

// --- 主程式 ---
void main(void) {
    WDT_A_hold(WDT_A_BASE);
    
    // 1. 解鎖關鍵硬體保護 (FR5969 專用寫法)
    *(volatile unsigned int *)(0x0160) = 0xA500;
    PMM_unlockLPM5();    // 解鎖 GPIO
    
    system_init();
    check_fram_integrity();
    write_index = 0;
    // 2. 啟動 Timer_A (10ms 中斷, SMCLK=8MHz)
    Timer_A_initUpModeParam timerParam = {0};
    timerParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    // 修正：使用除以 8 分頻器，讓計數基準變成 1MHz
    timerParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    timerParam.timerPeriod = 12500; // 1,000,000 / 100Hz = 10,000
    timerParam.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    timerParam.timerClear = TIMER_A_DO_CLEAR;
    timerParam.startTimer = true;
    Timer_A_initUpMode(TIMER_A0_BASE, &timerParam);

    __enable_interrupt();
    
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0); // LED2 green
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN6); // LED1 red
    // UART_print("MSP430 Wake Up!\r\n"); // 測試 UART
    
    while (1) {
        // UART_print("Alive\r\n");
        // 進入 LPM0 睡眠，等待計時器喚醒
        __bis_SR_register(LPM0_bits | GIE); 

        if (timer_triggered) {
            timer_triggered = 0;

            // 流程：讀取 -> 計算 -> 存檔
            uint32_t D1 = ms5607_get_raw(CMD_CONV_D1);
            uint32_t D2 = ms5607_get_raw(CMD_CONV_D2);
            
            calculate_and_log(D1, D2);
            GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN6);
            if(write_index >= 100)
                write_index = 0;
            // 檢查是否滿了
            // if (write_index >= 100) {
            //     // Timer_A_stop(TIMER_A0_BASE);
                
            //     // GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
            //     // export_data();
            //     uint16_t total_len = buf_ptr - &export_buf[active_buf_idx][0];
            //     if (total_len > 0 && total_len < (LOG_SIZE * ENTRY_STR_MAX)) {
            //         DMA0CTL &= ~DMAEN; // 先關閉，確保重置狀態機
                    
            //         __data20_write_long((unsigned long)&DMA0SA, (unsigned long)&export_buf[active_buf_idx][0]);
            //         DMA0SZ = total_len; 
                    
            //         DMA0CTL |= DMAEN;  // 重新開啟
            //         DMA0CTL |= DMAREQ; // 【關鍵】軟體手動觸發第一次搬移，啟動連鎖反應
            //     }

            //     active_buf_idx = !active_buf_idx;
            //     buf_ptr = &export_buf[active_buf_idx][0];
            //     write_index = 0;
            //     GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN6);
                
            //     // GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
                
            //     // Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
            // }
        }
    }
}

void check_fram_integrity(void) {
    ms5607_reset();
    // 1. 檢查寫入索引是否合法
    if (write_index >= 100) {
        write_index = 0; // 索引異常，強制歸零防止溢位
    }

    // 2. 檢查是否需要重新讀取感測器係數
    // 如果 Magic Number 不對，或者第一筆係數為 0，就重新讀取
    if (fram_initialized != 0xABCD || ms5607_coeffs[1] == 0) {
        ms5607_read_prom();
        
        // 讀完後做一個簡單判斷，如果讀到全 0 代表感測器沒接好
        if (ms5607_coeffs[1] != 0 && ms5607_coeffs[1] != 0xFFFF) {
            fram_initialized = 0xABCD; // 標記為初始化成功
        }
    }
}
// --- DMA init ---
void dma_init() {
    // 設置 DMA 觸發源為 EUSCI_A0 的 TX 標記 (UCA0TXIFG)
    // DMA_initParam dmaParam = {0};
    // dmaParam.channelSelect = DMA_CHANNEL_0;
    // dmaParam.transferModeSelect = DMA_TRANSFER_SINGLE; // 每次觸發傳一個 Byte
    // dmaParam.triggerSourceSelect = DMA_TRIGGERSOURCE_14; // EUSCI_A0 TX 觸發
    // dmaParam.transferUnitSelect = DMA_SIZE_SRCBYTE_DSTBYTE;
    // dmaParam.triggerTypeSelect = DMA_TRIGGER_RISINGEDGE;
    // DMA_init(&dmaParam);

    // // 設置源地址 (你的緩衝區) 與 目標地址 (UART TX 暫存器)
    // DMA_setSrcAddress(DMA_CHANNEL_0, EUSCI_A_UART_getTransmitBufferAddress(EUSCI_A0_BASE), DMA_DIRECTION_UNCHANGED);

    DMACTL0 = DMA0TSEL_14;
    DMA0CTL = DMADT_0 | DMASBDB | DMASRCINCR_3 | DMADSTINCR_0 | DMALEVEL;
    __data20_write_long((unsigned long)&DMA0DA, (unsigned long)&UCA0TXBUF);

    UART_print("--- DMA INIT FINISHED ---\r\n");
}
// --- SPI 與 系統初始化 ---
void system_init() {
    // Clock: 8MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // GPIO SPI: P1.6(MOSI), P1.7(MISO), P2.2(CLK)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN2, GPIO_SECONDARY_MODULE_FUNCTION);

    // CS Pin: P1.3
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3); // set high 就是初始化，要使用該通道的時候會把電壓壓低

    // GPIO UART: P2.0(TXD), P2.1(RXD)
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION);

    // EUSCI_A0 UART 初始化
    // 針對 8MHz SMCLK, 設定 115200 Baudrate 的官方參數
    EUSCI_A_UART_initParam uartParam = {0};
    uartParam.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    // uartParam.clockSourceFrequency = 8000000; // 沒有這個變數，不需要傳入
    uartParam.clockPrescalar = 4;
    uartParam.firstModReg = 5;
    uartParam.secondModReg = 0x55;
    uartParam.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &uartParam)) return;
    EUSCI_A_UART_enable(EUSCI_A0_BASE);
    
    UART_print("--- UART INIT FINISHED ---\r\n");
    UART_print("--- GPIO INIT FINISHED ---\r\n");
    // EUSCI_B0 SPI 初始化
    EUSCI_B_SPI_initMasterParam spiParam = {0};
    spiParam.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
    spiParam.clockSourceFrequency = 8000000;
    spiParam.desiredSpiClock = 1000000; // 1MHz
    spiParam.msbFirst = EUSCI_B_SPI_MSB_FIRST;
    // spiParam.clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    spiParam.clockPhase = EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    spiParam.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    spiParam.spiMode = EUSCI_B_SPI_3PIN;
    EUSCI_B_SPI_initMaster(EUSCI_B0_BASE, &spiParam);
    EUSCI_B_SPI_enable(EUSCI_B0_BASE);
    UART_print("--- SPI INIT FINISHED ---\r\n");
}

// --- MS5607 通訊底層 ---
uint8_t spi_xfer(uint8_t data) {
    EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, data);
    while (EUSCI_B_SPI_isBusy(EUSCI_B0_BASE));
    return EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
}

uint32_t ms5607_get_raw(uint8_t conv_cmd) {
    uint32_t result = 0;
    
    // 1. 開始轉換
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    spi_xfer(conv_cmd);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    
    __delay_cycles(24000); // OSR 1024 延時約 3ms

    // 2. 讀取結果
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    spi_xfer(CMD_ADC_READ);
    result |= (uint32_t)spi_xfer(0x00) << 16;
    result |= (uint32_t)spi_xfer(0x00) << 8;
    result |= (uint32_t)spi_xfer(0x00);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    
    return result;
}

void ms5607_read_prom() {
    int i;
    for (i = 0; i < 8; i++) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
        spi_xfer(CMD_PROM_RD + (i * 2));
        uint8_t h = spi_xfer(0x00);
        uint8_t l = spi_xfer(0x00);
        ms5607_coeffs[i] = (h << 8) | l;
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    }
}

void ms5607_reset() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    spi_xfer(CMD_RESET);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    __delay_cycles(24000);
}

// --- 數據運算與寫入 ---
void calculate_and_log(uint32_t D1, uint32_t D2) {
    // 1. 溫度與壓力運算 (簡化公式，實際請帶入 MS5607 標準二階補償)
    int32_t dT = D2 - ((int32_t)ms5607_coeffs[5] << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * ms5607_coeffs[6]) >> 23);
    int64_t OFF = ((int64_t)ms5607_coeffs[2] << 17) + (((int64_t)ms5607_coeffs[4] * dT) >> 6);
    int64_t SENS = ((int64_t)ms5607_coeffs[1] << 16) + (((int64_t)ms5607_coeffs[3] * dT) >> 7);
    int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;

    float pressure_hpa = P / 100.0f;
    float altitude = 44330.0f * (1.0f - powf(pressure_hpa / 1013.25f, 0.190295f));

    // 2. 封裝存檔
    altimeter_log[write_index].timestamp = global_tick; // 建議接 RTC 或 Global Tick
    altimeter_log[write_index].pressure = pressure_hpa;
    altimeter_log[write_index].altitude = altitude;
    altimeter_log[write_index].temperature = (uint16_t)TEMP;

    char buf[64];
    sprintf(buf, "%lu,%.2f,%.2f,%u\r\n", 
            (unsigned long)global_tick, // 強制轉換為 unsigned long 以匹配 %lu
            (double)pressure_hpa,       // sprintf 處理 float 時實際上是轉為 double 傳遞
            (double)altitude,           // 同上
            (unsigned int)TEMP);
    UART_print(buf);
    write_index++;
}

void UART_print(char* s) {
    while(*s) {
        // 使用 DriverLib 的方式發送資料
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *s++);
        
        // 等待發送完成，避免資料重疊
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));
    }
}

void export_data(void) {
    char buf[64];
    uint16_t i;

    // 傳送 CSV 表頭，方便電腦識別
    UART_print("\r\n--- START OF LOG ---\r\n");
    UART_print("Time,Pressure(hPa),Altitude(m),Temp(C*100)\r\n");

    for (i = 0; i < 100; i++) {
        // 將每一筆 LogEntry 轉換成 CSV 格式
        // 注意：MSP430 的 printf 預設可能不支援 float，
        // 若高度噴不出來，請檢查 CCS 的 printf support 設定 (設為 full)
        sprintf(buf, "%lu,%.2f,%.2f,%u\r\n", 
                altimeter_log[i].timestamp,
                altimeter_log[i].pressure,
                altimeter_log[i].altitude,
                altimeter_log[i].temperature);
        
        UART_print(buf);
    }
    
    UART_print("--- END OF LOG ---\r\n");
}

// --- 中斷服務 ---
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void) {
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    timer_triggered = 1;
    global_tick++;
    __bic_SR_register_on_exit(LPM0_bits); // 喚醒 CPU
}
