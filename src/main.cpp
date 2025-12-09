/**
 * ============================================================================
 * 项目名称: IMSLAS - 自主化学源定位系统 (核心控制与采集单元)
 * 硬件平台: ESP32-S3 (N16R8) + ST7796U (TFT) + FT6336 (Touch) + ADS8681 (ADC)
 * 核心功能: 
 * 1. 产生高精度离子门控制时序 (硬件定时器)
 * 2. 高速 ADC 数据采集 (SPI DMA/Burst)
 * 3. 硬件中断级同步采集 (消除 Jitter)
 * 4. 信号累加平均算法 (降噪)
 * 5. 双核多任务架构 (Core1 采集 / Core0 显示)
 * ============================================================================
 */

#include <Arduino.h> 
#include <SPI.h>
#include <Wire.h>

// --- 第三方库 ---
#include <TFT_eSPI.h> 
#include <FT6336.h>       
#include <lvgl.h>       

// --- 本地头文件 ---
#include "ui/ui.h"        
#include "IMS_ADC.h"   

// ============================================================================
// [SECTION 1] 硬件引脚定义 (Hardware Pin Definitions)
// ============================================================================

// I2C 总线 (触摸屏 & 其他传感器)
#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9

// 触摸屏中断与复位
#define CTP_INT_PIN         4
#define CTP_RST_PIN         13       

// IMS 核心控制引脚
#define ION_GATE_PIN        48      // 离子门控制 (需连接高压驱动光耦)

// ============================================================================
// [SECTION 2] IMS 系统参数配置 (System Configuration)
// ============================================================================

// --- 时序与频率参数 ---
#define IMS_SAMPLE_RATE     1000000 // ADC采样率: 1MSPS (1us/点)
#define IMS_DURATION_MS     24      // 单次采样窗口: 24ms (适配 UI X轴)
#define IMS_CYCLE_FREQ      33      // 工作频率: 33Hz (周期约 30.3ms，留出 6ms 处理时间)
#define IMS_PULSE_WIDTH_US  250     // 离子门开启脉宽: 250us (0.25ms)

// --- 信号处理参数 ---
#define IMS_AVG_COUNT       16      // 平均次数: 累加 16 次后更新显示 (平衡流畅度与信噪比)
#define RAW_DATA_LEN        (IMS_SAMPLE_RATE * IMS_DURATION_MS / 1000) // 缓冲区长度: 24000 点

// --- UI 显示参数 ---
#define SCREEN_WIDTH        480
#define SCREEN_HEIGHT       320
#define TOUCH_RAW_WIDTH     320  
#define TOUCH_RAW_HEIGHT    480 
#define CHART_POINTS        200     // 图表分辨率 (X轴点数)

// ============================================================================
// [SECTION 3] 全局变量与对象 (Global Variables & Objects)
// ============================================================================

// --- 硬件对象 ---
TFT_eSPI tft = TFT_eSPI();
FT6336 ts = FT6336(I2C_SDA_PIN, I2C_SCL_PIN, CTP_INT_PIN, CTP_RST_PIN, TOUCH_RAW_WIDTH, TOUCH_RAW_HEIGHT);
hw_timer_t *ims_timer = NULL;       // 硬件定时器句柄

// --- LVGL 显示缓冲 ---
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;

// --- 数据缓冲区 (动态分配) ---
// 1. 原始数据缓存 (单次采集, uint16)
uint16_t *big_raw_buffer = NULL;
// 2. 累加缓存 (用于平均算法, uint32 防止溢出)
uint32_t *accumulator_buffer = NULL;
// 3. 显示波形缓存 (压缩后)
int16_t waveform_buffer[CHART_POINTS];  

// --- RTOS 句柄 ---
SemaphoreHandle_t dataMutex;        // 数据互斥锁 (保护 UI 显示数据)
SemaphoreHandle_t syncSemaphore;    // 同步信号量 (中断 -> 采集任务)
TaskHandle_t TaskHandle_UI;         // UI 任务句柄

// --- 业务状态变量 ---
volatile bool isScanning = false;   // 扫描开关
float detected_peak_time = 0.0;     // 检测到的峰值时间 (ms)
int detected_peak_amp = 0;          // 检测到的峰值幅度
bool ui_update_needed = false;      // UI 刷新标志位

// --- UI 对象引用 ---
lv_chart_series_t * ui_SignalSeries;    

// ============================================================================
// [SECTION 4] 函数原型声明 (Function Prototypes)
// ============================================================================
void IRAM_ATTR onIMSTimerInterrupt();
void IMS_HW_Init();
void Task_Acquisition(void *pvParameters);
void Task_UI_Handler(void *pvParameters);
void OnScanClick(lv_event_t * e);
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p );
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data );

// ============================================================================
// [SECTION 5] IMS 硬件驱动与中断 (Hardware Drivers & ISR)
// ============================================================================

/**
 * @brief 硬件定时器中断服务函数
 * @note  IRAM_ATTR 确保代码在 RAM 中运行，最小化延迟
 * @desc  1. 产生离子门控制脉冲
 * 2. 发送信号量同步 ADC 采集任务
 */
void IRAM_ATTR onIMSTimerInterrupt() {
    // 仅在扫描状态下工作
    if (isScanning) {
        // [Phase 1] 产生高精度脉冲
        digitalWrite(ION_GATE_PIN, HIGH);
        esp_rom_delay_us(IMS_PULSE_WIDTH_US); // 硬件级微秒延时 (250us)
        digitalWrite(ION_GATE_PIN, LOW);

        // [Phase 2] 触发 ADC 采集 (硬同步)
        // 唤醒采集任务，使其在离子门关闭的瞬间立即开始工作
        xSemaphoreGiveFromISR(syncSemaphore, NULL);
    }
}

/**
 * @brief 初始化 IMS 专用硬件 (定时器 + GPIO + 同步机制)
 */
void IMS_HW_Init() {
    // 1. GPIO 配置
    pinMode(ION_GATE_PIN, OUTPUT);
    digitalWrite(ION_GATE_PIN, LOW);

    // 2. RTOS 对象创建
    syncSemaphore = xSemaphoreCreateBinary();

    // 3. 硬件定时器配置 (Timer 0, 80MHz/80 = 1MHz, 1 tick = 1us)
    ims_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(ims_timer, &onIMSTimerInterrupt, true);
    timerAlarmWrite(ims_timer, 1000000 / IMS_CYCLE_FREQ, true); // 设定周期
    timerAlarmEnable(ims_timer);

    Serial.println(">> IMS Hardware Driver Initialized (Timer + Sync)");
}

// ============================================================================
// [SECTION 6] RTOS 任务定义 (RTOS Tasks)
// ============================================================================

/**
 * @brief 任务 1: ADC 采集与信号处理 (运行于 Core 1)
 * @desc  高优先级任务，负责同步采集、累加、平均运算
 */
void Task_Acquisition(void *pvParameters) {
    (void) pvParameters;
    int average_counter = 0; // 累加计数器

    while (true) {
        // 内存安全检查
        if (big_raw_buffer == NULL || accumulator_buffer == NULL) {
            vTaskDelay(100);
            continue;
        }

        // [BLOCKING] 等待硬件中断的同步信号 (硬同步核心)
        if (xSemaphoreTake(syncSemaphore, portMAX_DELAY) == pdTRUE) {
            
            // 停止状态下的清理逻辑
            if (!isScanning) {
                average_counter = 0;
                memset(accumulator_buffer, 0, RAW_DATA_LEN * sizeof(uint32_t));
                continue; 
            }

            // --- 阶段 1: 高速采集 ---
            // 此时 T=0 (离子门刚动作)，立即读取 24ms 数据
            IMS_ADC_ReadBurst(big_raw_buffer, RAW_DATA_LEN);

            // --- 阶段 2: 数据累加 ---
            for (int i = 0; i < RAW_DATA_LEN; i++) {
                uint16_t raw_val = big_raw_buffer[i];
                // 大小端转换 (ADS8681 SPI 协议适配)
                raw_val = (raw_val << 8) | (raw_val >> 8); 
                accumulator_buffer[i] += raw_val;
            }
            average_counter++;

            // --- 阶段 3: 平均与压缩 (当攒够 IMS_AVG_COUNT 次) ---
            if (average_counter >= IMS_AVG_COUNT) {
                
                // 尝试获取 UI 数据锁 (不阻塞太久，丢帧保流)
                if (xSemaphoreTake(dataMutex, 10) == pdTRUE) { 
                    
                    int ratio = RAW_DATA_LEN / CHART_POINTS;
                    int global_max_val = 0;
                    int global_max_idx = 0;

                    // 降采样与平均值计算
                    for (int i = 0; i < CHART_POINTS; i++) {
                        int local_max_avg = 0;
                        
                        // Peak Hold 算法: 在区间内找最大值
                        for (int j = 0; j < ratio; j++) {
                            int idx = i * ratio + j;
                            if (idx >= RAW_DATA_LEN) break;

                            // 核心计算: 累加值 / N
                            int avg_val = accumulator_buffer[idx] / IMS_AVG_COUNT;
                            if (avg_val > local_max_avg) local_max_avg = avg_val;
                        }

                        // 存入显示 Buffer (缩小比例适配屏幕)
                        waveform_buffer[i] = local_max_avg / 16;

                        // 寻找全谱最大值 (用于数值显示)
                        if (local_max_avg > global_max_val) {
                            global_max_val = local_max_avg;
                            global_max_idx = i * ratio;
                        }
                    }

                    // 物理量换算
                    detected_peak_time = (float)global_max_idx / 1000.0; // us -> ms
                    detected_peak_amp = global_max_val / 16;

                    ui_update_needed = true; // 通知 UI 任务刷新
                    xSemaphoreGive(dataMutex);
                }

                // 清空累加器，准备下一轮
                memset(accumulator_buffer, 0, RAW_DATA_LEN * sizeof(uint32_t));
                average_counter = 0;
            }
        }
    }
}

/**
 * @brief 任务 2: UI 界面刷新 (运行于 Core 0)
 * @desc  中优先级任务，负责 LVGL 渲染和屏幕刷新，不干扰采集
 */
void Task_UI_Handler(void *pvParameters) {
    (void) pvParameters;

    while (true) {
        // 1. 检查数据更新
        if (ui_update_needed) {
            // 非阻塞尝试拿锁
            if (xSemaphoreTake(dataMutex, 0) == pdTRUE) { 
                
                // A. 刷新图表
                if (ui_SignalSeries != NULL) {
                    lv_chart_set_ext_y_array(ui_Chart1, ui_SignalSeries, (lv_coord_t*)waveform_buffer);
                    lv_chart_refresh(ui_Chart1); 
                }

                // B. 刷新数值标签
                if (isScanning && detected_peak_amp > 0) {
                    static char buf_time[16];
                    static char buf_amp[16];
                    sprintf(buf_time, "%.2f ms", detected_peak_time);
                    sprintf(buf_amp, "%d", detected_peak_amp);
                    
                    if(ui_Label18) lv_label_set_text(ui_Label18, buf_time); 
                    if(ui_Label19) lv_label_set_text(ui_Label19, buf_amp);
                } else {
                    // 停止时显示占位符
                    if(ui_Label18) lv_label_set_text(ui_Label18, "--.--");
                }
                
                ui_update_needed = false;
                xSemaphoreGive(dataMutex);
            }
        }

        // 2. LVGL 心跳维护
        lv_timer_handler();
        lv_tick_inc(5);
        
        // 3. 任务延时 (防止看门狗复位，让渡 Core 0 资源)
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

// ============================================================================
// [SECTION 7] 回调与辅助函数 (Callbacks & Helpers)
// ============================================================================

// SquareLine 导出的按钮事件回调
void OnScanClick(lv_event_t * e) {
    isScanning = !isScanning;
    
    lv_obj_t * ui_Button9 = lv_event_get_target(e); 
    lv_obj_t * label = lv_obj_get_child(ui_Button9, 0); 

    if (isScanning) {
        lv_label_set_text(label, "暂停扫描"); 
        lv_obj_set_style_bg_color(ui_Button9, lv_color_hex(0x00AA00), LV_PART_MAIN); 
        Serial.println("Action: IMS Start Scanning");
    } else {
        lv_label_set_text(label, "开始扫描");
        lv_obj_set_style_bg_color(ui_Button9, lv_color_hex(0x0869B4), LV_PART_MAIN); 
        Serial.println("Action: IMS Stop Scanning");
    }
}

// LVGL 显示驱动刷新回调
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p ) {
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();
    lv_disp_flush_ready( disp_drv );
}

// LVGL 触摸驱动读取回调
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data ) {
    ts.read(); 
    if( ts.touches > 0 ) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = ts.points[0].y;        
        data->point.y = 319 - ts.points[0].x; // 坐标旋转适配
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// ============================================================================
// [SECTION 8] 系统初始化 (Setup)
// ============================================================================

void setup() {
    // 1. 基础通信初始化
    Serial.begin(115200);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    Serial.println("\n\n==================================");
    Serial.println("IMSLAS System Booting...");
    Serial.println("==================================");

    // 2. RTOS 资源初始化
    dataMutex = xSemaphoreCreateMutex();

    // 3. 硬件设备初始化
    IMS_ADC_Init(); // ADS8681 初始化
    Serial.println("[OK] ADC Hardware Initialized.");

    // 4. 大容量内存分配 (PSRAM 优先)
    size_t raw_size_bytes = RAW_DATA_LEN * sizeof(uint16_t);
    size_t acc_size_bytes = RAW_DATA_LEN * sizeof(uint32_t); 

    Serial.printf("[INFO] Memory Req: Raw=%.2f KB, Acc=%.2f KB\n", raw_size_bytes/1024.0, acc_size_bytes/1024.0);

    // 尝试在 SPIRAM 分配
    big_raw_buffer = (uint16_t *)heap_caps_malloc(raw_size_bytes, MALLOC_CAP_SPIRAM);
    accumulator_buffer = (uint32_t *)heap_caps_malloc(acc_size_bytes, MALLOC_CAP_SPIRAM);

    // 失败降级处理 (尝试内部 RAM)
    if (big_raw_buffer == NULL) {
        Serial.println("[WARN] PSRAM Raw Buffer Alloc Failed. Trying Internal RAM.");
        big_raw_buffer = (uint16_t *)malloc(raw_size_bytes);
    }
    if (accumulator_buffer == NULL) {
        Serial.println("[WARN] PSRAM Acc Buffer Alloc Failed. Trying Internal RAM.");
        accumulator_buffer = (uint32_t *)malloc(acc_size_bytes);
    }

    // 致命错误检查
    if (big_raw_buffer == NULL || accumulator_buffer == NULL) {
        Serial.println("[CRITICAL] Memory Alloc Failed! Halted.");
        while(1);
    }
    
    // 内存清零
    memset(big_raw_buffer, 0, raw_size_bytes);
    memset(accumulator_buffer, 0, acc_size_bytes);
    Serial.println("[OK] Memory Allocated & Cleared.");

    // 5. 启动 IMS 核心控制 (定时器 + 中断)
    IMS_HW_Init();

    // 6. UI 子系统初始化 (TFT + LVGL)
    tft.init();
    tft.setRotation(1);
    tft.invertDisplay(true);
    tft.fillScreen(TFT_BLACK);
    ts.begin();

    lv_init();
    size_t buffer_size = SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(lv_color_t);
    buf = (lv_color_t*) heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (buf == NULL) buf = (lv_color_t*) malloc(SCREEN_WIDTH * 32 * sizeof(lv_color_t));
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, SCREEN_WIDTH * (buf==NULL?32:SCREEN_HEIGHT));

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init(); // SquareLine 生成的 UI 初始化
    
    // 获取图表句柄并配置
    ui_SignalSeries = lv_chart_get_series_next(ui_Chart1, NULL);
    lv_chart_set_update_mode(ui_Chart1, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_point_count(ui_Chart1, CHART_POINTS);
    
    Serial.println("[OK] UI System Initialized.");

    // 7. 启动多任务处理
    Serial.println("[INFO] Starting Tasks...");

    // 任务 A: 采集 (绑定 Core 1, 高优)
    xTaskCreatePinnedToCore(
        Task_Acquisition, "IMS_ADC_Core1", 8192, NULL, 10, NULL, 1 
    );
    
    // 任务 B: UI 显示 (绑定 Core 0, 中优)
    xTaskCreatePinnedToCore(
        Task_UI_Handler, "IMS_UI_Core0", 8192, NULL, 5, &TaskHandle_UI, 0 
    );

    Serial.println(">> IMS System Ready & Running. Waiting for user command.");
}

// ============================================================================
// [SECTION 9] 主循环 (Main Loop)
// ============================================================================

void loop() {
    // 主循环已空置，所有逻辑均由 RTOS 任务接管
    // 仅用于防止编译器优化或作为空闲钩子
    vTaskDelay(pdMS_TO_TICKS(1000));
}