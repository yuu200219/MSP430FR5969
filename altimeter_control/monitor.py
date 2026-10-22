import serial
import csv
import matplotlib.pyplot as plt
import time

# ================= 設定區 =================
COM_PORT = '/dev/cu.usbmodem1303'      # 每次要跑之前都要先確認是多少
BAUD_RATE = 115200                      # 必須跟 MSP430 的設定一模一樣
CSV_FILENAME = 'rocket_flight_log.csv'
IMAGE_FILENAME = 'rocket_flight.png'
# ==========================================

# 1. 準備畫圖視窗 (建立 5 列 1 行的子圖，並共用 X 軸)
plt.ion() 
fig, axs = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
fig.suptitle("Rocket Flight Telemetry (Live)", fontsize=16)

# 分配 5 個子圖的變數
ax_speed, ax_press, ax_alt, ax_rel, ax_temp = axs

# 設定各個圖表的 Y 軸標籤與線條顏色
ax_speed.set_ylabel("Speed (m/s)")
line_speed, = ax_speed.plot([], [], 'b-') # 藍線

ax_press.set_ylabel("Press (hPa)")
line_press, = ax_press.plot([], [], 'g-') # 綠線

ax_alt.set_ylabel("Alt (m)")
line_alt, = ax_alt.plot([], [], 'm-')     # 紫線

ax_rel.set_ylabel("Rel Alt (m)")
line_rel, = ax_rel.plot([], [], 'r-')     # 紅線

ax_temp.set_ylabel("Temp")
ax_temp.set_xlabel("Time (s)")            # 只有最下面的圖需要顯示 X 軸標籤
line_temp, = ax_temp.plot([], [], 'c-')   # 青色線

plt.tight_layout() # 自動調整間距，避免標籤重疊

# 準備存放數據的陣列
time_data = []
speed_data = []
press_data = []
alt_data = []
rel_data = []
temp_data = []

# 2. 開啟 CSV 檔案準備寫入
with open(CSV_FILENAME, mode='w', newline='') as file:
    writer = csv.writer(file)
    # 寫入 CSV 標題列
    writer.writerow(["Redundancy", "Timestamp", "Speed", "Pressure", "Altitude", "Rel_Alt", "Temp"])

    # 3. 開啟 Serial Port
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        print(f"📡 已連線到 {COM_PORT}，開始接收火箭數據... (按 Ctrl+C 結束)")

        while True:
            # 讀取一行並解碼
            raw_line = ser.readline().decode('utf-8').strip()
            
            if raw_line:
                print(f"收到: {raw_line}") # 可以在終端機印出來看
                
                parts = raw_line.split(',')
                
                # 確保收到的資料欄位數量正確 (你有 7 個變數)
                if len(parts) >= 7:
                    try:
                        # 將字串轉成數字
                        t_tick = float(parts[1]) * 100 * 0.001 # 轉成秒
                        speed  = float(parts[2])
                        press  = float(parts[3])
                        alt    = float(parts[4])
                        rel    = float(parts[5])
                        temp   = float(parts[6]) 
                        # 備註：如果你的 MS5607 溫度原本有 *100，這裡可以除以 100 回復攝氏度 (temp = float(parts[6]) / 100.0)
                        
                        # (A) 即時寫入 CSV
                        writer.writerow(parts)
                        
                        # (B) 更新陣列準備畫圖
                        time_data.append(t_tick)
                        speed_data.append(speed)
                        press_data.append(press)
                        alt_data.append(alt)
                        rel_data.append(rel)
                        temp_data.append(temp)
                        
                        # (C) 即時更新 5 條線的數據
                        line_speed.set_data(time_data, speed_data)
                        line_press.set_data(time_data, press_data)
                        line_alt.set_data(time_data, alt_data)
                        line_rel.set_data(time_data, rel_data)
                        line_temp.set_data(time_data, temp_data)
                        
                        # 重新計算每一個子圖的坐標軸極限
                        for ax in axs:
                            ax.relim()          
                            ax.autoscale_view() 
                            
                        fig.canvas.draw()
                        fig.canvas.flush_events()

                    except ValueError:
                        # 略過轉換失敗的雜訊行 (例如一開機的 "Calibrating Ground...")
                        pass

    except KeyboardInterrupt:
        print("\n🛑 接收中斷！準備存檔...")
        
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        
        # (D) 將最終完整的圖表存成圖片
        plt.ioff() # 關閉互動模式
        plt.savefig(IMAGE_FILENAME, dpi=300, bbox_inches='tight') # 存成 300dpi 的高畫質 PNG
        print(f"✅ 數據已儲存至 {CSV_FILENAME}")
        print(f"✅ 圖表已儲存至 {IMAGE_FILENAME}")
        plt.show() # 最後讓圖表停留在畫面上供你檢視
