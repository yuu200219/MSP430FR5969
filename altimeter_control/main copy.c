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
// #define CMD_CONV_D1     0x44 // OSR 1024 (Pressure)
// #define CMD_CONV_D2     0x54 // OSR 1024 (Temperature)
#define CMD_CONV_D1     0x48 // OSR 4096 (Pressure)
#define CMD_CONV_D2     0x58 // OSR 4096 (Temperature)

// --- 資料結構 ---
typedef struct {
    uint32_t timestamp;
    float pressure;
    float altitude;
    uint16_t temperature;
} LogEntry;

// --- FRAM 持久化資料 ---
#pragma PERSISTENT(altimeter_log)
LogEntry altimeter_log[2][100] = {0};

#pragma PERSISTENT(write_index)
unsigned int write_index = 0;

#pragma PERSISTENT(ms5607_coeffs)
uint16_t ms5607_coeffs[7] = {0}; // 校準係數

#pragma PERSISTENT(fram_initialized)
uint16_t fram_initialized = 0; // 用來存 Magic Number，例如 0xABCD

// --- 全域變數 ---


volatile uint8_t active_buf = 0; // 0->Buffer_A, 1->Buffer_B
volatile uint8_t sd_write_flag = 0;  
volatile uint8_t buf_to_write = 0;

volatile uint8_t rocket_launched = 1;
volatile uint8_t timer_triggered = 0;
volatile uint32_t global_tick = 0;
volatile uint8_t space_bar_timeout = 0; // 0 = off, >0 = relayer on countdown
volatile uint8_t in_repeat_mode = 0;    // Tracks if we are in auto-repeat

volatile uint8_t motor_triggered = 0;

#define SPEED_WINDOW 5
double speed_history[SPEED_WINDOW] = {0.0};
uint8_t speed_idx = 0;
uint8_t speed_count = 0;
double filtered_speed = 0.0;

#define PRESSURE_WINDOW 4
float pressure_history[4] = {0.0};
int8_t pressure_idx = 0;

#define ALT_WINDOW 10
float alt_history[10] = {0.0};
int8_t alt_idx = 0;

float ground_alt = 0;
EUSCI_A_SPI_initMasterParam paramA1 = {0};

#pragma PERSISTENT(g_fs)
FATFS g_fs = {0}; // 定義一個全域的 FatFs 物件

FRESULT g_res; // 儲存掛載結果

#pragma PERSISTENT(g_file)
FIL g_file = {0}; // 檔案物件

UINT bytes_written; // 用來記錄實際寫入了幾個 Byte 
// --- 函式宣告 ---
void check_fram_integrity(void);
void system_init(void);
void ms5607_reset(void);
void ms5607_read_prom(void);
uint32_t ms5607_get_raw(uint8_t conv_cmd);
float calculate_only_msl(uint32_t D1, uint32_t D2);
void calculate_and_log(uint32_t D1, uint32_t D2);
void UART_print(char* s);
float smooth_pressure(float pressure);
bool altSampling(float alt);
void flush_log_to_sd(uint8_t target_buf);

char g_filename[32]; // 用來存最終決定好的動態檔名

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

// --- 主程式 ---
// current tick rate: 8,000,000 Hz / 64 = 125,000 Hz (每 125,000 跳動一次)
    // 觸發 interrupt 時間: 12500 / 125,000 = 0.1 sec

    // 流程
    // 1. 每 100 ms: Timer_A 倒數結束，觸發中斷
    // 2. 喚醒 CPU: 中斷服務程式(Timer_A0_ISR) 將 timer_triggered 設為 1，並把 CPU 從 LPM0 mode 喚醒
    // 3. sampling
    //      - 呼叫 ms5607_get_raw。裡面包含 __delay_cycles(24000)，在 8MHz 下這大約是 3ms。
    //      - 你讀取了 D1 (壓力) 和 D2 (溫度)，所以總共會卡死大約 6ms 在等待感測器
    // 4. 計算與寫入 FRAM
    //      - calculate_and_log 進行浮點數運算
    //      - 將結果存入 altimeter_log[write_index]（這是寫入 FRAM）
    // 5. UART 傳輸
    //      - 呼叫 UART_print(buf)
    //      - 你的 buf 長度大約 40~50 字元。在 115200 波特率下，傳輸一個字元約 0.086ms
    //      - 傳完這串字串大約花費 4ms
    // 總結 3.-5.: CPU 會空出 10ms (6ms sampling wait + 4ms tranfer latency) 剩下 90ms 在省電睡覺
void main(void) {
    WDT_A_hold(WDT_A_BASE);
    
    rocket_launched = 1; 
    active_buf = 0;
    sd_write_flag = 0;
    buf_to_write = 0;
    timer_triggered = 0;
    global_tick = 0;

    // 1. 解鎖關鍵硬體保護 (FR5969 專用寫法)
    *(volatile unsigned int *)(0x0160) = 0xA500;
    PMM_unlockLPM5();    // 解鎖 GPIO
    
    system_init();
    check_fram_integrity();

    // check sd card FatFs 掛載結果
    
    while(1) {
        g_res = f_mount(&g_fs, "0:", 1); // 強制實體掛載
        if (g_res == FR_OK) {
            // SD 卡完全準備好了
            // 既然開機握手已經成功，在這裡立刻將 eUSCI_A1 的時脈切換到 8MHz 高速！ 
            // paramA1.desiredSpiClock = 8000000; // 8000000 = 8MHz
            // EUSCI_A_SPI_disable(EUSCI_A1_BASE);
            // paramA1.desiredSpiClock = 1000000;
            // EUSCI_A_SPI_initMaster(EUSCI_A1_BASE, &paramA1);
            // EUSCI_A_SPI_enable(EUSCI_A1_BASE);
            UART_print("SD is ready!\r\n");
            // 可以點亮板子上的綠色 LED (P1.0) 代表儲存系統正常

            break;
        } 
        else {
            // 依據 g_res 的錯誤碼進行排錯，並點亮紅色 LED (P4.6) 警告
            // char err_buf[64];
            // sprintf(err_buf, "SD Card Mount FAILED! Code: %d. Retrying...\r\n", g_res);
            // UART_print(err_buf);
            continue;
        }
    }
    // 檢查 sd card 現有檔案
    // GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
    // __delay_cycles(160000);
    // if(disk_initialize(0)!=0)
    //     UART_print("SPI Error: SD Re-init Failed!\r\n");
    // else {
        // UART_print("SPI Re-init success\r\n");
    find_next_flight_filename();
    // }

    // 2. 啟動 Timer_A (10ms 中斷, SMCLK=8MHz)
    Timer_A_initUpModeParam timerParam = {0};
    timerParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK; // SMCLK = 8MHz
    // 修正：使用除以 8 分頻器，讓計數基準變成 1MHz
    timerParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64; // 分頻器 = 64
    // Change to 20ms resolution for higher sensitivity (125kHz * 0.02s = 2500 counts)
    timerParam.timerPeriod = 2500; 
    timerParam.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    timerParam.timerClear = TIMER_A_DO_CLEAR;
    timerParam.startTimer = true;
    Timer_A_initUpMode(TIMER_A0_BASE, &timerParam);

    __enable_interrupt();
    // UART_print("MSP430 Wake Up!\r\n"); // 測試 UART
    
    // 進入 while 迴圈之前 先校正高度 之後吐出的高度會是 related altitude
    
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
    while (1) {
        // UART_print("Alive\r\n");
        // 進入 LPM0 睡眠，等待計時器喚醒
        __bis_SR_register(LPM0_bits | GIE); 
        
        
        if (timer_triggered) {
            timer_triggered = 0;
            if (rocket_launched == 1) {
                // ==========================================
                // 【發射前：地面待機模式】
                // ==========================================
                // 1. 只算高度，不存檔，不影響 write_index
                UART_print("Calibrating Ground...\r\n");
    
                float sum_alt = 0;
                char calibration_buf[64];
                int i;
                // char cur_alt_buf[64];
                for(i = 0; i<10; i++) {
                    uint32_t d1 = ms5607_get_raw(CMD_CONV_D1);
                    uint32_t d2 = ms5607_get_raw(CMD_CONV_D2);
                    sum_alt += calculate_only_msl(d1, d2);

                    __delay_cycles(80000);
                }
                ground_alt = sum_alt/10.0f;
                sprintf(calibration_buf, "Current Altitude: %.2f m\r\n", (double)ground_alt);
                UART_print(calibration_buf); 
                __delay_cycles(80000);

            } else { // rocket_launched == 0
                // uint32_t D1 = ms5607_get_raw(CMD_CONV_D1);
                // uint32_t D2 = ms5607_get_raw(CMD_CONV_D2);
                // calculate_and_log(D1, D2);
            
                uint16_t current_idx = (write_index == 0) ? 99 : (write_index - 1);
                uint16_t prev_idx = (current_idx == 0) ? 99 : (current_idx - 1);
                double rel_alt = (double)altimeter_log[active_buf][current_idx].altitude - (double)ground_alt;
                
                // speed calculation
                double speed = 0.0;
                // if(&altimeter_log[active_buf][prev_idx] != NULL) {
                    if (altimeter_log[active_buf][current_idx].timestamp > altimeter_log[active_buf][prev_idx].timestamp) {
                        double delta_alt = (double)altimeter_log[active_buf][current_idx].altitude - (double)altimeter_log[active_buf][prev_idx].altitude;
                        uint32_t delta_ticks = altimeter_log[active_buf][current_idx].timestamp - altimeter_log[active_buf][prev_idx].timestamp;
                        double delta_time = (double)delta_ticks * 0.1;
                        double raw_speed = delta_alt / delta_time;

                        speed_history[speed_idx] = raw_speed;
                        speed_idx = (speed_idx + 1) % SPEED_WINDOW;

                        if (speed_count < SPEED_WINDOW) {
                            speed_count++;
                        }
                        double sum = 0.0;
                        
                        int i;
                        for (i = 0; i < speed_count; i++) {
                            sum += speed_history[i];
                        }
                        filtered_speed = sum / speed_count;
                    }
                // }
                

                // ==========================================
                // [副傘邏輯]
                // ==========================================
                // 1. 如果時間超過 20sec 且速度小於等於 20m/s，發送開副傘訊號
                // 2. check relay_1_stat == 1?
                // 3. 如果時間超過 25sec relay_1_stat 不等於 1，go 1.
                // 4. 如果時間超過 27sec relay_1_stat 不等於 1，發送主傘開傘訊號
                
                uint8_t relay_1_stat = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5);
                uint8_t relay_2_stat = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3);

                if (global_tick >= 200 && global_tick < 250 && filtered_speed <= 20 && relay_1_stat == GPIO_INPUT_PIN_LOW) { // 1 tick = 100ms, 20sec = 20000ms = 200 ticks
                    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
                    // GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
                }
                else if (global_tick >= 250 && relay_1_stat == GPIO_INPUT_PIN_LOW) { // send signal again if relay_1_stat != 1
                    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
                    // GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
                }
                else if (global_tick >270 && relay_1_stat == GPIO_INPUT_PIN_LOW) { // start main parachute logic
                    if (rel_alt <= 450 && relay_2_stat == GPIO_INPUT_PIN_LOW) {
                        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4);
                        // GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
                    }
                }
                // ==========================================   
                // [主傘邏輯]
                // ==========================================
                // 1. rel_alt <= 450 && direction = 1(downward)，發送主傘開傘訊號
                // 2. check relay_2_stat
                // 3. after 1 sec? 500ms? send go 1.

                if (rel_alt <= 450 && global_tick > 270 && relay_2_stat == GPIO_INPUT_PIN_LOW) {
                    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
                }
                
                char buf[64];
                sprintf(buf, "AAA, %lu, %.2f, %.2f, %.2f, %.2f, %.2f\r\n", 
                    (unsigned long)altimeter_log[active_buf][current_idx].timestamp,
                    filtered_speed,
                    (double)altimeter_log[active_buf][current_idx].pressure,
                    (double)altimeter_log[active_buf][current_idx].altitude,
                    rel_alt,
                    (double)altimeter_log[active_buf][current_idx].temperature);
                UART_print(buf);
                
                if(write_index >= 100) {
                    buf_to_write = active_buf; // 現在寫入的buffer滿了，要開始寫入到 sd card，將現在這個 active_buf index 給 buf_to_write
                    active_buf = active_buf ^ 1; // xor active_buf，切換 active_buf index
                    write_index = 0; // write index 歸零
                    sd_write_flag = 1; // main 可以開始寫入 sd card
                }
                if (sd_write_flag == 1) {
                    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);        
                    flush_log_to_sd(buf_to_write);
                    // g_res = f_open(&g_file, g_filename, FA_WRITE | FA_OPEN_APPEND);
                    sd_write_flag = 0;
                    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
                    // if (g_res == FR_OK) {
                        // 如果是全新檔案，可以先寫入 CSV 表頭 
                        // if (f_size(&g_file) == 0) {
                        //     char* header = "Redundancy, TimeStamp, Speed, Pressure(hPa),Altitude(m), Rel_Alt(m), Temp(C)\r\n";
                        //     f_write(&g_file, header, strlen(header), &bytes_written);
                        // }
                        
                        // f_write(&g_file, buf, strlen(buf), &bytes_written);
                        // f_sync(&g_file);
                        // f_close(&g_file);
                        
                    // }
                }
            }
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

// void init_dma_sdcard(void) {
//     DMA_initParam dmaParam = {0};
//     dmaParam.channelSelect = DMA_CHANNEL_1; 
    
//     // ⚠️ 注意：SPI 必須使用 SINGLE 模式，不能用 BLOCK。
//     // 因為 SPI 需要時間傳輸，每當 TXBUF 空了 (觸發 1 次)，DMA 才塞入 1 個 Byte
//     dmaParam.transferModeSelect = DMA_TRANSFER_SINGLE;
    
//     // 觸發源：22 代表 EUSCI_A1_TX (參考 FR5969 Datasheet)
//     dmaParam.triggerSourceSelect = DMA_TRIGGERSOURCE_22;
    
//     dmaParam.transferUnitSelect = DMA_SIZE_SRCBYTE_DSTBYTE;
//     // dmaParam.srcAddressSourceSelect = DMA_SRCADDRESS_INCREMENT; // 記憶體位址遞增
//     // dmaParam.dstAddressSourceSelect = DMA_DSTADDRESS_UNCHANGED; // SPI TX 暫存器位址不變
    
//     DMA_init(&dmaParam);
    
//     // 開啟 DMA Channel 1 的完成中斷，這樣才能喚醒 CPU
//     DMA_clearInterrupt(DMA_CHANNEL_1);
//     DMA_enableInterrupt(DMA_CHANNEL_1);
// }

// --- SPI 與 系統初始化 ---
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
    // uartParam.clockSourceFrequency = 8000000; // 沒有這個變數，不需要傳入
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
    paramA1.desiredSpiClock = 1000000;                          // 關鍵：SD卡初始化必須先跑 400kHz 低速！
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

void calculate_and_log(uint32_t D1, uint32_t D2) {
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
    float filtered_pressure = smooth_pressure(pressure_hpa);
    float altitude = 44330.0f * (1.0f - powf(filtered_pressure / 1013.25f, 0.190295f));

    float temp = TEMP / 100.0f;

    // 2. 封裝存檔
    altimeter_log[active_buf][write_index].timestamp = global_tick; // 建議接 RTC 或 Global Tick
    altimeter_log[active_buf][write_index].pressure = filtered_pressure;
    altimeter_log[active_buf][write_index].altitude = altitude;
    altimeter_log[active_buf][write_index].temperature = temp;

    // char buf[100];
    // sprintf(buf, "%lu,%.2f,%.2f,%u\r\n", 
    //         (unsigned long)global_tick, // 強制轉換為 unsigned long 以匹配 %lu
    //         (double)pressure_hpa,       // sprintf 處理 float 時實際上是轉為 double 傳遞
    //         (double)altitude,           // 同上
    //         (unsigned int)TEMP);
    // // sprintf(buf, "D1:%lu, C1:%u, C2:%u, C3:%u, C4:%u, C5:%u, C6:%u\r\n", (uint32_t)D1, ms5607_coeffs[1], ms5607_coeffs[2], ms5607_coeffs[3], ms5607_coeffs[4], ms5607_coeffs[5], ms5607_coeffs[6]);
    // UART_print(buf);
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
        for (i = 0; i < 100; i++) {
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
        
        UART_print("--- SD Card Write 100 Records OK ---\r\n");
    } else {
        char err_buf[64];
        sprintf(err_buf, "--- SD Card Open FAILED! Code: %d ---\r\n", g_res);
        UART_print(err_buf);
    }
}

// --- 中斷服務 ---
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void) {
    // 加入 sub_tick 來追蹤 20ms 的次數
    static uint8_t sub_tick = 0; 
    
    // 1. 空白鍵超時控制 (每 20ms 執行一次)
    if (space_bar_timeout > 0) {
        space_bar_timeout--;
        
        // 倒數結束，代表使用者放開了空白鍵
        if (space_bar_timeout == 0) {
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN2); // 關閉繼電器 (馬達斷電)
            in_repeat_mode = 0;                              // 退出連發模式
        }
    }
    
    // 2. 主程式取樣邏輯 (每 5 次 sub_tick = 100ms 才執行一次)
    if (++sub_tick >= 5) {
        sub_tick = 0; // 重置計數器
        
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0); // 閃爍綠燈，代表系統仍在運作
        timer_triggered = 1;                             // 告訴主迴圈該取樣了
        
        uint32_t D1 = ms5607_get_raw(CMD_CONV_D1);
        uint32_t D2 = ms5607_get_raw(CMD_CONV_D2);
        calculate_and_log(D1, D2);

        if (rocket_launched == 0) {
            global_tick++; 
        }
        
        __bic_SR_register_on_exit(LPM0_bits);            // 喚醒 CPU 執行 main loop
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

// #pragma vector=DMA_VECTOR
// __interrupt void DMA_ISR(void) {
//     // 16 是 DMAIV 的最大有效範圍
//     switch(__even_in_range(DMAIV, 16)) {
//         case 0x00: break; // 無中斷
//         case 0x02: break; // DMA Channel 0 
//         case 0x04: // DMA Channel 1 完成 (對應原本的 DMA1IFG)
//             __bic_SR_register_on_exit(LPM0_bits); // 喚醒 CPU
//             break;
//         case 0x06: break; // DMA Channel 2
//         default: break;
//     }
// }
