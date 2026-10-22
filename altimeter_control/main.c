#include "driverlib.h"
#include <msp430.h>
#include <math.h>
#include <stdio.h>
#include "ff.h"
#include "diskio.h"
#include <string.h>

// --- MS5607 指令集 ---
#define CMD_RESET       0x1E
#define CMD_ADC_READ    0x00
#define CMD_PROM_RD     0xA0
#define CMD_CONV_D1     0x48 // OSR 4096 (Pressure)
#define CMD_CONV_D2     0x58 // OSR 4096 (Temperature)
// #define CMD_CONV_D1     0x40 // OSR 256 (Pressure)
// #define CMD_CONV_D2     0x50 // OSR 256 (Temperatur)

// --- 資料結構 ---
typedef struct {
    uint32_t timestamp;
    float pressure;
    float altitude;
    uint16_t temperature;
} LogEntry;


typedef struct {
    unsigned long tick;
    double speed;
    float pressure;
    float altitude;
    double rel_alt;
    float temp;
} UartData;

volatile UartData uart_data_buf;
volatile uint8_t uart_tx_flag = 0;

// --- FRAM 持久化資料 ---
#pragma PERSISTENT(altimeter_log)
volatile LogEntry altimeter_log[2][100] = {0};

#pragma PERSISTENT(write_index)
volatile unsigned int write_index = 0;

#pragma PERSISTENT(ms5607_coeffs)
volatile uint16_t ms5607_coeffs[7] = {0}; 

#pragma PERSISTENT(fram_initialized)
volatile uint16_t fram_initialized = 0; 

// --- 全域變數 ---
volatile uint8_t active_buf = 0; 
volatile uint8_t sd_write_flag = 0;  
volatile uint8_t buf_to_write = 0;

volatile uint8_t rocket_launched = 1;
volatile uint32_t global_tick = 0;
volatile uint8_t space_bar_timeout = 0; 
volatile uint8_t in_repeat_mode = 0; 

volatile uint16_t parachute_pulse_timer = 0;

volatile uint8_t minor_parachute_is_triggered = 0;
volatile uint8_t major_parachute_is_triggered = 0;

// --- 新增：感測器狀態機變數 ---
volatile uint8_t ms5607_state = 0;
volatile uint32_t d1_raw = 0;
volatile uint32_t d2_raw = 0;
// volatile uint8_t raw_data_ready = 0;

#define MS_STATE_CONV_P       0  // 狀態 0：啟動壓力轉換 (D1)
#define MS_STATE_READ_P_CONV_T 1  // 狀態 1：讀取 D1，並啟動溫度轉換 (D2)
#define MS_STATE_READ_T_CALC   2  // 狀態 2：讀取 D2，進行二階補償計算與開傘判斷

#define SPEED_WINDOW 5
double speed_history[SPEED_WINDOW] = {0.0};
uint8_t speed_idx = 0;
uint8_t speed_count = 0;
double filtered_speed = 0.0;

#define PRESSURE_WINDOW 4
float pressure_history[4] = {0.0};
int8_t pressure_idx = 0;

#define ALT_WINDOW 10
float alt_history[ALT_WINDOW] = {0.0}; // 儲存近 10 筆高度
uint8_t alt_idx = 0;           // 陣列索引

float ground_alt = 0;
EUSCI_A_SPI_initMasterParam paramA1 = {0};

#pragma PERSISTENT(g_fs)
FATFS g_fs = {0}; 
FRESULT g_res; 

#pragma PERSISTENT(g_file)
FIL g_file = {0}; 
UINT bytes_written; 

char g_filename[32]; 

// --- 函式宣告 ---
void check_fram_integrity(void);
void system_init(void);
void ms5607_reset(void);
void ms5607_read_prom(void);
void ms5607_start_conv(uint8_t conv_cmd); // 非阻塞：發送轉換指令
uint32_t ms5607_read_adc(void);           // 非阻塞：直接讀取轉換結果
float calculate_only_msl(uint32_t D1, uint32_t D2);
void calculate_and_log(uint32_t D1, uint32_t D2);
void UART_print(char* s);
float smooth_pressure(float pressure);
void flush_log_to_sd(uint8_t target_buf);

void find_next_flight_filename(void);
void trigger_minor_parachute(void);
void trigger_major_parachute(void);

void main(void) {
    WDT_A_hold(WDT_A_BASE);
    
    rocket_launched = 1; 
    active_buf = 0;
    sd_write_flag = 0;
    buf_to_write = 0;
    global_tick = 0;

    *(volatile unsigned int *)(0x0160) = 0xA500;
    PMM_unlockLPM5();    
    
    system_init();
    check_fram_integrity();

    // SD 卡掛載
    while(1) {
        g_res = f_mount(&g_fs, "0:", 1); 
        if (g_res == FR_OK) {
            UART_print("SD is ready!\r\n");

            EUSCI_A_SPI_disable(EUSCI_A1_BASE);      // 先關閉 SPI
            paramA1.desiredSpiClock = 4000000;       // 重新設定為 8MHz
            EUSCI_A_SPI_initMaster(EUSCI_A1_BASE, &paramA1);
            EUSCI_A_SPI_enable(EUSCI_A1_BASE);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
            break;
        } 
        else {
            UART_print("SD card is not ready!\r\n");
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
            break;
        }
    }
    
    find_next_flight_filename();

    // 啟動 Timer_A (維持 10ms 中斷, SMCLK=8MHz)
    Timer_A_initUpModeParam timerParam = {0};
    timerParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK; 
    timerParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64; // 8MHz / 64 = 125kHz
    timerParam.timerPeriod = 1250; // 125kHz * 0.01s (10ms) = 1250 counts
    timerParam.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    timerParam.timerClear = TIMER_A_DO_CLEAR;
    timerParam.startTimer = true;
    Timer_A_initUpMode(TIMER_A0_BASE, &timerParam);

    __enable_interrupt();
    // 【發射前地面校準】
    UART_print("Calibrating Ground...\r\n");
    float sum_alt = 0;
    int i;
    for(i = 0; i < 10; i++) {
        // uint32_t d1 = ms5607_get_raw(CMD_CONV_D1);
        // uint32_t d2 = ms5607_get_raw(CMD_CONV_D2);
        ms5607_start_conv(CMD_CONV_D1);
        __delay_cycles(90000); // max delay: 9.04ms on OSR 4096
        uint32_t d1 = ms5607_read_adc();
                        
        ms5607_start_conv(CMD_CONV_D2);
        __delay_cycles(90000); // max delay: 9.04ms on OSR 4096 
        uint32_t d2 = ms5607_read_adc();
                        
        sum_alt += calculate_only_msl(d1, d2);
        __delay_cycles(90000); // max delay: 9.04ms on OSR 4096
    }
    ground_alt = sum_alt / 10.0f;
    char calibration_buf[64];
    sprintf(calibration_buf, "Ground Altitude Set: %.2f m\r\n", (double)ground_alt);
    UART_print(calibration_buf);
    
    while (1) {
        
        
        // 【安全鎖機制】：判斷前關閉中斷
        __disable_interrupt();
        
        if (sd_write_flag == 0 && uart_tx_flag == 0) {
            // 沒事做，進入 LPM0 (會自動重開 GIE)
            __bis_SR_register(LPM0_bits | GIE); 
        } else {
            // 有事做，手動打開中斷
            __enable_interrupt();
        }
        // if(raw_data_ready == 1) {

            
        //     raw_data_ready = 0;
        // }
        // 任務 1：UART 輸出 (SD卡沒在寫入時才印)
        if(uart_tx_flag == 1 && sd_write_flag == 0) {
            char uart_main_buf[64];
            sprintf(uart_main_buf, "AAA, %lu, %.2f, %.2f, %.2f, %.2f, %.f\r\n", 
                uart_data_buf.tick,
                uart_data_buf.speed,
                uart_data_buf.pressure,
                uart_data_buf.altitude,
                uart_data_buf.rel_alt,
                uart_data_buf.temp);
            UART_print(uart_main_buf);
            uart_tx_flag = 0;
        }
                
        // 任務 2：SD 卡寫入
        if (sd_write_flag == 1) {
            // GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); // 亮綠燈      
            flush_log_to_sd(buf_to_write);
            sd_write_flag = 0;
            // GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }
    }    
}

void find_next_flight_filename(void) {
    UINT file_num = 1;
    FRESULT res;
    FILINFO fno;
    char debug_msg[64]; // 用來印出錯誤訊息

    // 迴圈嘗試尋找尚未被佔用的序號
    // sprintf(g_filename, "0:FLIGHT.CSV", file_num);
    // res = f_stat(g_filename, &fno);
    // if (res == FR_NO_FILE) {
    //     UART_print("Available filename found: ");
    // }
    // else {
    //     sprintf(debug_msg, "f_stat FATAL ERROR Code: %d\r\n", res);
    //     UART_print(debug_msg);
    //     sprintf(g_filename, "0:ERROR.CSV");
    // }
    while (file_num < 10000) {
        sprintf(g_filename, "0:FL_%05u.CSV", file_num);
        
        // 檢查這個檔案是否已經存在
        res = f_stat(g_filename, &fno);
        
        if (res == FR_NO_FILE) {
            // 正常情況：找到了空檔名
            UART_print("Available filename found: ");
            UART_print(g_filename);
            UART_print("\r\n");
            break; 
            
        } else if (res == FR_OK) {
            // 正常情況：檔案已存在，繼續往下找
            file_num++; 
            
        } else {
            // 異常情況：發生了嚴重的硬體或系統錯誤！
            // 必須立刻跳出迴圈，否則會卡死 10000 次
            sprintf(debug_msg, "f_stat FATAL ERROR Code: %d\r\n", res);
            UART_print(debug_msg);
            
            // 給定一個備用檔名避免系統崩潰
            sprintf(g_filename, "0:ERROR.CSV");
            break;
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
        write_index = 0;
        active_buf = 0;
        memset(altimeter_log, 0, sizeof(altimeter_log));
        // 讀完後做一個簡單判斷，如果讀到全 0 代表感測器沒接好
        if (ms5607_coeffs[1] != 0 && ms5607_coeffs[1] != 0xFFFF) {
            fram_initialized = 0xABCD; // 標記為初始化成功
        }
    }
}

void system_init() {
    // Clock: 8MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // seting LED
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0); // LED2 green
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN6); // LED1 red
    // -----------------------------------
    // eUSCI_A0 (UART)
    // -----------------------------------
    // GPIO UART: P2.0(TXD), P2.1(RXD)
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION);
    // EUSCI_A0 UART 初始化
    // 針對 8MHz SMCLK, 設定 115200 Baudrate 的官方參數
    EUSCI_A_UART_initParam uartParam = {0};
    uartParam.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    uartParam.clockPrescalar = 4;
    uartParam.firstModReg = 5;
    uartParam.secondModReg = 0x55;
    uartParam.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &uartParam)) return;
    EUSCI_A_UART_enable(EUSCI_A0_BASE);
    
    // Enable UART RX interrupt for space bar detection
    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    UART_print("--- UART INIT FINISHED ---\r\n");
    
    // -----------------------------------
    // eUSCI_B0 (SPI_1) -> 給高度計 MS5607
    // -----------------------------------
    // GPIO SPI: P1.6(MOSI), P1.7(MISO), P2.2(CLK)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN2, GPIO_SECONDARY_MODULE_FUNCTION);
    // CS Pin: P1.3
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3); // set high 就是初始化，要使用該通道的時候會把電壓壓低
    // EUSCI_B0 SPI 初始化
    EUSCI_B_SPI_initMasterParam spiParam = {0};
    spiParam.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
    spiParam.clockSourceFrequency = 8000000;
    spiParam.desiredSpiClock = 1000000; // 1MHz
    // spiParam.desiredSpiClock = 100000;
    spiParam.msbFirst = EUSCI_B_SPI_MSB_FIRST;
    // spiParam.clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    spiParam.clockPhase = EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    spiParam.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    spiParam.spiMode = EUSCI_B_SPI_3PIN;
    EUSCI_B_SPI_initMaster(EUSCI_B0_BASE, &spiParam);
    EUSCI_B_SPI_enable(EUSCI_B0_BASE);
    UART_print("--- SPI_1 INIT FINISHED ---\r\n");

    // -----------------------------------
    // eUSCI_A1 (SPI_2) -> 給SD card
    // -----------------------------------
    // GPIO SPI: P2.4 (SCLK_2), P2.5 (MOSI_2), P2.6 (MISO_2)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6, GPIO_SECONDARY_MODULE_FUNCTION);
    // CS Pin: P1.4
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // set high 就是初始化，要使用該通道的時候會把電壓壓低
    // EUSCI_A1 SPI 初始化
    // EUSCI_A_SPI_initMasterParam paramA1 = {0};
    paramA1.selectClockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK; // 使用 SMCLK (8MHz)
    paramA1.clockSourceFrequency = 8000000;
    // paramA1.desiredSpiClock = 1000000;                          // 關鍵：SD卡初始化必須先跑 400kHz 低速！
    paramA1.desiredSpiClock = 400000;
    paramA1.msbFirst = EUSCI_A_SPI_MSB_FIRST;
    paramA1.clockPhase = EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    paramA1.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    paramA1.spiMode = EUSCI_A_SPI_3PIN;                        // 3Pin 模式
    EUSCI_A_SPI_initMaster(EUSCI_A1_BASE, &paramA1);
    EUSCI_A_SPI_enable(EUSCI_A1_BASE);                         // 實體啟用 SD卡 SPI
    UART_print("--- SPI_2 INIT FINISHED ---\r\n");
    
    
    // setup P3.0 as output pin
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);
    // setup P3.4 as output pin
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN4);
    // setup P1.5 as input pin
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN5);
    // setup P4.3 as input pin
    GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN3);
    UART_print("--- RELAY INIT FINISHED ---\r\n");
    // init_dma_sdcard();
    // UART_print("--- DMA INIT FINISHED ---\r\n");
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
    
    // __delay_cycles(24000); // OSR 1024 延時約 3ms
    __delay_cycles(80000);

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
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3); // CSB Low
        
        spi_xfer(CMD_PROM_RD | (i << 1)); 
        
        uint8_t h = spi_xfer(0x00);
        uint8_t l = spi_xfer(0x00);
        
        ms5607_coeffs[i] = (uint16_t)((h << 8) | l);
        
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3); // CSB High
    }
}

void ms5607_reset() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    spi_xfer(CMD_RESET);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    __delay_cycles(24000);
}

// --- 非阻塞式 MS5607 底層通訊 ---
void ms5607_start_conv(uint8_t conv_cmd) {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3); // CSB Low
    spi_xfer(conv_cmd);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3); // CSB High
}

uint32_t ms5607_read_adc(void) {
    uint32_t result = 0;
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3); // CSB Low
    spi_xfer(CMD_ADC_READ);
    result |= (uint32_t)spi_xfer(0x00) << 16;
    result |= (uint32_t)spi_xfer(0x00) << 8;
    result |= (uint32_t)spi_xfer(0x00);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3); // CSB High
    return result;
}

// --- 數據運算與寫入 ---
float calculate_only_msl(uint32_t D1, uint32_t D2) {
    // 1. 溫度與壓力運算, MS5607 標準二階補償
    // refernece: https://www.cdiweb.com/datasheets/te/eng_ds_ms5607-02ba03_b.pdf (p.8 - p.9)
    int32_t dT = D2 - ((int32_t)ms5607_coeffs[5] << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * ms5607_coeffs[6]) >> 23);
    int64_t OFF = ((int64_t)ms5607_coeffs[2] << 17) + (((int64_t)ms5607_coeffs[4] * (int64_t)dT) >> 6);
    int64_t SENS = ((int64_t)ms5607_coeffs[1] << 16) + (((int64_t)ms5607_coeffs[3] * (int64_t)dT) >> 7);
    
    // 二階補償 for P
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    
    if (TEMP < 2000) { // 如果溫度低於 20°C
        T2 = ((int64_t)dT * (int64_t)dT) >> 31;
        int64_t temp_diff = (int64_t)TEMP - 2000;
        OFF2 = 61 * (temp_diff * temp_diff) >> 4;
        SENS2 = 2 * (temp_diff * temp_diff);
        
        if (TEMP < -1500) { // 如果低於 -15°C (火箭高空可能遇到)
            int64_t temp_diff_low = (int64_t)TEMP + 1500;
            OFF2 += 15 * (temp_diff_low * temp_diff_low);
            SENS2 += 8 * (temp_diff_low * temp_diff_low);
        }
    }

    TEMP -= (int32_t)T2;
    OFF -= OFF2;
    SENS -= SENS2;

    int64_t P_64 = (((int64_t)D1 * SENS) >> 21) - OFF;
    int32_t P = (int32_t)(P_64 >> 15); // 最後才把結果轉回 32 位元
    
    // 無二階補償 for P
    // int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;

    float pressure_hpa = P / 100.0f;
    float altitude = 44330.0f * (1.0f - powf(pressure_hpa / 1013.25f, 0.190295f));

    return altitude;
}

// --- 數據運算與寫入 ---
void calculate_and_log(uint32_t D1, uint32_t D2) {
    int32_t dT = D2 - ((int32_t)ms5607_coeffs[5] << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * ms5607_coeffs[6]) >> 23);
    int64_t OFF = ((int64_t)ms5607_coeffs[2] << 17) + (((int64_t)ms5607_coeffs[4] * (int64_t)dT) >> 6);
    int64_t SENS = ((int64_t)ms5607_coeffs[1] << 16) + (((int64_t)ms5607_coeffs[3] * (int64_t)dT) >> 7);
    
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) { 
        T2 = ((int64_t)dT * (int64_t)dT) >> 31;
        int64_t temp_diff = (int64_t)TEMP - 2000;
        OFF2 = 61 * (temp_diff * temp_diff) >> 4;
        SENS2 = 2 * (temp_diff * temp_diff);
        if (TEMP < -1500) { 
            int64_t temp_diff_low = (int64_t)TEMP + 1500;
            OFF2 += 15 * (temp_diff_low * temp_diff_low);
            SENS2 += 8 * (temp_diff_low * temp_diff_low);
        }
    }
    TEMP -= (int32_t)T2;
    OFF -= OFF2;
    SENS -= SENS2;

    int64_t P_64 = (((int64_t)D1 * SENS) >> 21) - OFF;
    int32_t P = (int32_t)(P_64 >> 15); 
    
    float pressure_hpa = P / 100.0f;
    // float filtered_pressure = smooth_pressure(pressure_hpa);
    float altitude = 44330.0f * (1.0f - powf(pressure_hpa / 1013.25f, 0.190295f));
    float temp = TEMP / 100.0f;

    // 寫入雙緩衝區
    altimeter_log[active_buf][write_index].timestamp = global_tick; 
    altimeter_log[active_buf][write_index].pressure = pressure_hpa;
    altimeter_log[active_buf][write_index].altitude = altitude;
    altimeter_log[active_buf][write_index].temperature = temp;

    write_index++;
}

float smooth_pressure(float pressure) {
    pressure_history[pressure_idx] = pressure;
    pressure_idx = (pressure_idx + 1) % PRESSURE_WINDOW;
    double sum = 0.0;
    int i;
    for(i = 0; i < PRESSURE_WINDOW; i++) {
        sum += pressure_history[i];
    }
    return sum / PRESSURE_WINDOW;
}

void UART_print(char* s) {
    while(*s) {
        // 使用 DriverLib 的方式發送資料
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *s++);
        
        // 等待發送完成，避免資料重疊
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));
    }
}

void flush_log_to_sd(uint8_t target_buf) {
    static char buf[128];
    int i;
    
    // 1. 打開或建立檔案 (檔名為 FLIGHT.CSV)
    // FA_WRITE: 允許寫入 | FA_OPEN_APPEND: 從檔案尾端繼續寫 (不會覆蓋舊資料)
    // g_res = f_open(&g_file, "0:FLIGHT.CSV", FA_WRITE | FA_OPEN_APPEND);
    g_res = f_open(&g_file, g_filename, FA_WRITE | FA_OPEN_APPEND);
    
    if (g_res == FR_OK) {
        // 如果是全新檔案，可以先寫入 CSV 表頭 
        if (f_size(&g_file) == 0) {
            char* header = "TimeStamp, Pressure(hPa),Altitude(m),Temp(C)\r\n";
            f_write(&g_file, header, strlen(header), &bytes_written);
        }

        // 2. 迴圈將 100 筆資料轉成字串並寫入
        for (i = 0; i < 50; i++) {
            // 注意：FR5969 SRAM 極小，我們逐行格式化並寫入，FatFs 底層會自動幫我們湊滿 512 bytes 才物理推入 SD 卡
            sprintf(buf, "%lu,%.2f,%.2f,%u\r\n", 
                (unsigned long)altimeter_log[target_buf][i].timestamp,
                (double)altimeter_log[target_buf][i].pressure,
                (double)altimeter_log[target_buf][i].altitude,
                (unsigned int)altimeter_log[target_buf][i].temperature);
            
            f_write(&g_file, buf, strlen(buf), &bytes_written);
        }
        
        // 3. 強制同步 (Sync) 
        // f_sync 會將 FatFs 緩衝區內的資料強制寫入實體 SD 卡，更新檔案大小。
        // 如果火箭摔壞斷電，只要有 Sync 過，先前的資料就絕對救得回來！
        f_sync(&g_file); 
        
        // 4. 關閉檔案
        f_close(&g_file);
        
        UART_print("--- SD Card Write 50 Records OK ---\r\n");
    } else {
        char err_buf[64];
        sprintf(err_buf, "--- SD Card Open FAILED! Code: %d ---\r\n", g_res);
        UART_print(err_buf);
    }
}

uint8_t voting_450(double rel_alt) {
    alt_history[alt_idx] = rel_alt;
    alt_idx = (alt_idx + 1) % 10; // 循環索引
    uint8_t count = 0;
    int i;
    for(i = 0; i < 10; i++) {
        if(alt_history[i] <= 450.0f) {
            count++;
        }
    }
    return count;
}

// --- 終極 10ms 中斷服務程式 ---
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void) {
    // 建立 0~9 的循環計數器 (10ms * 10 = 100ms)
    
    
    // 1. 每 100ms 閃爍一次綠燈，代表系統存活
    
    static uint16_t counter = 0;
    counter++;
    
    // 每 100ms 閃爍一次 (10ms * 10 = 100ms)，這樣人眼觀察比較明顯
    if (counter >= 10) {
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        counter = 0;
    }
    

    // 2. 火箭發射後的 10Hz (100ms) 取樣狀態機
    if (rocket_launched == 0) {
        static uint8_t cycle_tick = 0; 
        if (cycle_tick == 0) {
            // 【0ms】立刻送出壓力轉換指令
            ms5607_start_conv(CMD_CONV_D1);
        } else if (cycle_tick == 1) {
            // 【10ms】讀取壓力，立刻送出溫度轉換指令
            // d1_raw = ms5607_get_raw(CMD_CONV_D1);
            // __delay_cycles(90000); // max delay: 9.04ms on OSR 4096
            // __delay_cycles(6000);
            d1_raw = ms5607_read_adc();
            ms5607_start_conv(CMD_CONV_D2);
        }
        else if(cycle_tick == 2) {
            // 【10ms】讀取壓力，立刻送出溫度轉換指令
            // d1_raw = ms5607_read_adc();
            // d1_raw = ms5607_get_raw(CMD_CONV_D1);
            // ms5607_start_conv(CMD_CONV_D2);
            
            
            // 【20ms】讀取溫度，進行運算與邏輯判斷
            // __delay_cycles(90000); // max delay: 9.04ms on OSR 4096
            d2_raw = ms5607_read_adc();
            // d2_raw = ms5607_get_raw(CMD_CONV_D2);
            global_tick++; // 現在 global_tick 每跳一次代表過了 100ms (10Hz)
            // raw_data_ready = 1; 

            calculate_and_log(d1_raw, d2_raw);
                
            uint16_t current_idx = (write_index == 0) ? 49 : (write_index - 1);
            double rel_alt = (double)altimeter_log[active_buf][current_idx].altitude - (double)ground_alt;
                
            // 速度計算 (使用 static 變數避免雙緩衝區跨越邊界時的 index 錯誤與週期性突波)
            static double last_altitude = 0.0;
            static uint32_t last_timestamp = 0;
            static uint8_t speed_init = 0;

            double current_alt = (double)altimeter_log[active_buf][current_idx].altitude;
            uint32_t current_timestamp = altimeter_log[active_buf][current_idx].timestamp;

            if (speed_init == 0) {
                last_altitude = current_alt;
                last_timestamp = current_timestamp;
                speed_init = 1;
            } else {
                if (current_timestamp > last_timestamp) {
                    double delta_alt = current_alt - last_altitude;
                    double delta_time = (double)(current_timestamp - last_timestamp);
                    double raw_speed = delta_alt * 10.0 / delta_time;

                    speed_history[speed_idx] = raw_speed;
                    speed_idx = (speed_idx + 1) % SPEED_WINDOW;

                    if (speed_count < SPEED_WINDOW) speed_count++;
                    double sum = 0.0;
                    int i;
                    if (speed_count > 0) {
                        for (i = 0; i < speed_count; i++) {
                            sum += speed_history[i];
                        }
                        filtered_speed = sum / speed_count;
                    } else {
                        filtered_speed = 0.0;
                    }
                }
                last_altitude = current_alt;
                last_timestamp = current_timestamp;
            }
                
            // ==========================================
            // 【降落傘邏輯】(注意：時間閾值已根據 100ms/tick 校正)
            // ==========================================
            // 雙 relay 會改成單 relay 看使用哪個 pin
            uint8_t relay_stat = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5);
            // uint8_t relay_2_stat = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3);

            // 副傘邏輯 (20sec = 200 ticks, 25sec = 250 ticks, 27sec = 270 ticks)
            // 副傘 40m/s 暫定 會觸發，global_tick 判別秒數要改
            if (global_tick >= 200 && global_tick < 250 && filtered_speed <= 40 && minor_parachute_is_triggered == 0) { 
                // GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
                minor_parachute_is_triggered = 1;
                trigger_minor_parachute();
                // UART_print("MINOR PARACHUE IS TRIGGERED\r\n");
            }
            // else if (global_tick >= 250 && relay_1_stat == GPIO_INPUT_PIN_LOW) { 
            //     // GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
            //     trigger_minor_parachute();
            //     UART_print("MINOR PARACHUE IS TRIGGERED\r\n");
            // }
            // else if (global_tick > 270) { 
                    
            //     if (voting_450(rel_alt) >= 8 && rel_alt <= 450.0f) {
            //         // GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4);
            //         trigger_major_parachute();
            //         UART_print("MAIN PARACHUE IS TRIGGERED (minor not triggered)\r\n");
            //     }
            // }

            // 主傘邏輯
            if (voting_450(rel_alt) >= 8 && rel_alt <= 450.0f && global_tick > 800 && major_parachute_is_triggered == 0) {
                // GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4);
                major_parachute_is_triggered = 1;
                trigger_major_parachute();
                // UART_print("MAIN PARACHUE IS TRIGGERED\r\n");
            }
                
            // 準備 UART 數據給 Main 迴圈
            uart_data_buf.tick = altimeter_log[active_buf][current_idx].timestamp;
            uart_data_buf.speed = filtered_speed;
            uart_data_buf.pressure = altimeter_log[active_buf][current_idx].pressure;
            uart_data_buf.altitude = altimeter_log[active_buf][current_idx].altitude;
            uart_data_buf.rel_alt = rel_alt;
            uart_data_buf.temp = altimeter_log[active_buf][current_idx].temperature;

            uart_tx_flag = 1;
            
            
        }
        if(write_index >= 50) {
            if (sd_write_flag == 1) {
                // 安全鎖：上一包資料還沒寫完，退回一步丟棄新資料以保護記憶體
                write_index = 49; 
            } else {
                buf_to_write = active_buf; 
                active_buf = active_buf ^ 1; 
                write_index = 0; 
                sd_write_flag = 1; 
            }
        }
        // 推進計數器
        cycle_tick++;
        if (cycle_tick >= 10) {
            cycle_tick = 0;
        }            
    }
    // 加入降落傘持續轉的邏輯
    if(parachute_pulse_timer > 0) {
        parachute_pulse_timer--;
        if(parachute_pulse_timer == 0) {
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
        }
    }
    // 如果有工作要交接給 Main 迴圈，喚醒 CPU
    if (sd_write_flag == 1 || uart_tx_flag == 1 || rocket_launched == 1) {
        __bic_SR_register_on_exit(LPM0_bits);
    }
}

#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
    switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG: {
            uint8_t rx_val = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
            
            // if (rx_val == ' ') { // 偵測到空白鍵被按下
            //     GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN2); // 立即開啟繼電器
                
            //     // 如果在超時前又收到字元，代表我們成功跨過了 OS 的初始延遲，進入連發狀態
            //     if (space_bar_timeout > 0) {
            //         in_repeat_mode = 1; 
            //     }

            //     // 根據狀態設定不同的超時閾值
            //     if (in_repeat_mode) {
            //         // 連發模式：極高靈敏度，只要 80ms 沒收到訊號就斷電 (4 * 20ms)
            //         // 連發模式：放寬到 15 滴答 (300ms)(15 * 20ms)
            //         space_bar_timeout = 15; 
            //     } else {
            //         // 第一次按下：給予較長的寬容值，等待 OS 的連發訊號到來 (35 * 20ms = 700ms)
            //         space_bar_timeout = 35; 
            //     }
            // }
            if (rx_val == '0') {  // 偵測到 FPGA 傳來的 "發射" 訊號
                rocket_launched = 0; 
                UART_print("--- IGNITION! FLIGHT MODE ENGAGED ---\r\n");
            }
            break;
        }
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
    }
}

// --- 降落傘控制 ---
void trigger_minor_parachute(void) {
    if (parachute_pulse_timer == 0) { // Only start if not already active         
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
        parachute_pulse_timer = 350; // Set for 3 seconds (300 * 10ms)
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
    }
}

void trigger_major_parachute(void) {
    if (parachute_pulse_timer == 0) { // Only start if not already active         
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4);
        parachute_pulse_timer = 1000; // Set for 10 seconds (1000 * 10ms)
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
    }
}