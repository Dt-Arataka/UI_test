// ==========================================
// IMS 分析仪 - UI+ADC + (新增: 同步与平均)
// 硬件: ESP32-S3 (N16R8) + ST7796U + FT6336 + ADS8681
// ==========================================

#include <Arduino.h> 
#include <SPI.h>
#include <Wire.h>
#include <TFT_eSPI.h> 
#include <FT6336.h>       
#include <lvgl.h>       

#include "ui/ui.h"        
#include "IMS_ADC.h"   

// ==========================================
// [NEW] 1. 硬件引脚与IMS核心参数定义
// ==========================================
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define CTP_INT_PIN 4
#define CTP_RST_PIN 13       
#define TOUCH_RAW_WIDTH 320  
#define TOUCH_RAW_HEIGHT 480 

// [NEW] 离子门控制引脚
#define ION_GATE_PIN    46   // 请根据你的实际电路修改此引脚!

// [MODIFIED] IMS 时序参数调整
// 为了保证处理时间，将采样时长从 24ms 缩短到 20ms，给数据处理留出 5ms (25ms - 20ms)
#define IMS_SAMPLE_RATE     1000000 // 1MSPS
#define IMS_DURATION_MS     20      // [MODIFIED] 采样 20ms (避开周期末尾)
#define IMS_CYCLE_FREQ      40      // [NEW] 工作频率 40Hz (周期 25ms)
#define IMS_PULSE_WIDTH_US  250     // [NEW] 离子门脉宽 250us
#define IMS_AVG_COUNT       16      // [NEW] 平均次数 (累加16次显示一次)

#define RAW_DATA_LEN        (IMS_SAMPLE_RATE * IMS_DURATION_MS / 1000) // 20000点

// --- 2. 屏幕与图表参数 ---
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;
#define CHART_POINTS 200    // 图表分辨率

// --- 3. 内存缓冲区 ---
// [MODIFIED] 原始数据缓存 (单次采集)
uint16_t *big_raw_buffer = NULL;
// [NEW] 累加缓存 (用于平均算法，使用 uint32_t 防止溢出)
uint32_t *accumulator_buffer = NULL;

// --- 4. 全局对象 ---
TFT_eSPI tft = TFT_eSPI();
FT6336 ts = FT6336(I2C_SDA_PIN, I2C_SCL_PIN, CTP_INT_PIN, CTP_RST_PIN, TOUCH_RAW_WIDTH, TOUCH_RAW_HEIGHT);
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;

// --- 5. 业务逻辑控制变量 ---
volatile bool isScanning = false;       
SemaphoreHandle_t dataMutex;            
// [NEW] 同步信号量：用于中断通知任务“可以开始采了”
SemaphoreHandle_t syncSemaphore;        

lv_chart_series_t * ui_SignalSeries;    

// 共享数据区
int16_t waveform_buffer[CHART_POINTS];  
float detected_peak_time = 0.0;         
int detected_peak_amp = 0;              
bool ui_update_needed = false;          

// [NEW] 硬件定时器指针
hw_timer_t *ims_timer = NULL;

// =====================================================================
// [NEW] 模块化库函数：IMS 硬件驱动 (离子门 + 同步)
// =====================================================================

// 中断服务函数 (ISR) - 极高优先级，产生波形并同步
void IRAM_ATTR onIMSTimerInterrupt() {
    // 只有在扫描状态下才产生脉冲
    if (isScanning) {
        // 1. 产生离子门脉冲 (高精度控制)
        digitalWrite(ION_GATE_PIN, HIGH);
        esp_rom_delay_us(IMS_PULSE_WIDTH_US); // 维持 250us
        digitalWrite(ION_GATE_PIN, LOW);

        // 2. 发送同步信号给采集任务
        // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // xSemaphoreGiveFromISR(syncSemaphore, &xHigherPriorityTaskWoken);
        // if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
        // 简化写法，直接给信号量
        xSemaphoreGiveFromISR(syncSemaphore, NULL);
    }
}

// 初始化 IMS 硬件控制系统
void IMS_HW_Init() {
    // 1. 配置离子门引脚
    pinMode(ION_GATE_PIN, OUTPUT);
    digitalWrite(ION_GATE_PIN, LOW);

    // 2. 创建同步信号量 (二进制信号量)
    syncSemaphore = xSemaphoreCreateBinary();

    // 3. 配置硬件定时器 (Timer 0, 80分频 -> 1us 计数)
    ims_timer = timerBegin(0, 80, true);
    
    // 绑定中断函数
    timerAttachInterrupt(ims_timer, &onIMSTimerInterrupt, true);
    
    // 设置触发周期 (40Hz -> 25000us)
    timerAlarmWrite(ims_timer, 1000000 / IMS_CYCLE_FREQ, true);
    
    // 启动定时器
    timerAlarmEnable(ims_timer);

    Serial.println(">> IMS Hardware Driver Initialized (Timer + Sync)");
}

// =====================================================================
// 任务修改
// =====================================================================
void Task_Acquisition(void *pvParameters); 
void OnScanClick(lv_event_t * e);
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p );
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data );

// ==========================================
// [MODIFIED] 任务 1 (Core 1): 同步采集 + 平均算法
// ==========================================
void Task_Acquisition(void *pvParameters) {
    (void) pvParameters;
    
    int average_counter = 0; // 平均计数器

    while (true) {
        // 检查缓冲区是否准备好
        if (big_raw_buffer == NULL || accumulator_buffer == NULL) {
            vTaskDelay(100);
            continue;
        }

        // [MODIFIED] 等待同步信号
        // 这里的逻辑变了：不是直接跑，而是卡在这里等中断说“GO”
        // portMAX_DELAY 表示一直死等，直到中断触发
        if (xSemaphoreTake(syncSemaphore, portMAX_DELAY) == pdTRUE) {
            
            // 如果用户点了停止，就不处理数据，清空累加器
            if (!isScanning) {
                average_counter = 0;
                memset(accumulator_buffer, 0, RAW_DATA_LEN * sizeof(uint32_t));
                continue; 
            }

            // -----------------------------------------------------
            // 阶段 1: 硬件高速采集 (硬同步)
            // -----------------------------------------------------
            // 程序运行到这里，说明离子门刚刚关闭(或开启)，T=0时刻
            // 立即开始吸入 20ms 的数据
            IMS_ADC_ReadBurst(big_raw_buffer, RAW_DATA_LEN);

            // -----------------------------------------------------
            // 阶段 2: 累加算法 (Accumulation)
            // -----------------------------------------------------
            for (int i = 0; i < RAW_DATA_LEN; i++) {
                // 大小端转换 (ADS8681 SPI 数据修复)
                uint16_t raw_val = big_raw_buffer[i];
                raw_val = (raw_val << 8) | (raw_val >> 8); 
                
                // 累加到 32位 缓冲区
                accumulator_buffer[i] += raw_val;
            }
            average_counter++;

            // -----------------------------------------------------
            // 阶段 3: 平均与处理 (仅当攒够 N 次后执行)
            // -----------------------------------------------------
            if (average_counter >= IMS_AVG_COUNT) {
                
                // 获取 UI 锁
                if (xSemaphoreTake(dataMutex, 10) == pdTRUE) { // 等待10 ticks，如果不空闲就算了，丢帧保流
                    
                    int ratio = RAW_DATA_LEN / CHART_POINTS;
                    int global_max_val = 0;
                    int global_max_idx = 0;

                    // 压缩并计算平均值
                    for (int i = 0; i < CHART_POINTS; i++) {
                        int local_max_avg = 0; // 局部区间的最大值（平均后）
                        
                        // Peak Hold 降采样算法
                        for (int j = 0; j < ratio; j++) {
                            int idx = i * ratio + j;
                            if (idx >= RAW_DATA_LEN) break;

                            // [KEY] 核心平均公式： 累加值 / 次数
                            int avg_val = accumulator_buffer[idx] / IMS_AVG_COUNT;
                            
                            if (avg_val > local_max_avg) local_max_avg = avg_val;
                        }

                        // 填充显示波形 (除以系数适应屏幕)
                        waveform_buffer[i] = local_max_avg / 16;

                        // 记录全谱最大值
                        if (local_max_avg > global_max_val) {
                            global_max_val = local_max_avg;
                            global_max_idx = i * ratio;
                        }
                    }

                    // 计算物理量
                    detected_peak_time = (float)global_max_idx / 1000.0; // us -> ms
                    detected_peak_amp = global_max_val / 16;

                    ui_update_needed = true;
                    xSemaphoreGive(dataMutex);
                }

                // [IMPORTANT] 一轮平均结束，清空累加器，重置计数器
                memset(accumulator_buffer, 0, RAW_DATA_LEN * sizeof(uint32_t));
                average_counter = 0;
            }

            // 这里不需要 vTaskDelay 了，因为我们是靠 40Hz 的中断控制节奏的
            // 任务会自动阻塞在下一次 xSemaphoreTake 上
        }
    }
}

// ==========================================
//  SquareLine 事件回调函数 (保持不变)
// ==========================================
void OnScanClick(lv_event_t * e) {
    isScanning = !isScanning;
    
    lv_obj_t * ui_Button9 = lv_event_get_target(e); 
    lv_obj_t * label = lv_obj_get_child(ui_Button9, 0); 

    if (isScanning) {
        lv_label_set_text(label, "暂停扫描"); 
        lv_obj_set_style_bg_color(ui_Button9, lv_color_hex(0x00AA00), LV_PART_MAIN); 
        Serial.println("Action: IMS Start");
    } else {
        lv_label_set_text(label, "开始扫描");
        lv_obj_set_style_bg_color(ui_Button9, lv_color_hex(0x0869B4), LV_PART_MAIN); 
        Serial.println("Action: IMS Stop");
    }
}

// ==========================================
//  标准驱动回调 (保持不变)
// ==========================================
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p ) {
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();
    lv_disp_flush_ready( disp_drv );
}

void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data ) {
    ts.read(); 
    if( ts.touches > 0 ) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = ts.points[0].y;        
        data->point.y = 319 - ts.points[0].x;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// ==========================================
//  Setup 初始化
// ==========================================
void setup() {
    Serial.begin(115200);
    
    // 1. 创建互斥锁
    dataMutex = xSemaphoreCreateMutex();

    // 2. 硬件初始化
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    IMS_ADC_Init(); 
    Serial.println("ADC Hardware Initialized.");

    // [MODIFIED] 内存分配：分配 raw buffer 和 accumulator buffer
    size_t raw_size_bytes = RAW_DATA_LEN * sizeof(uint16_t);
    size_t acc_size_bytes = RAW_DATA_LEN * sizeof(uint32_t); // 32位，体积翻倍

    Serial.printf("Allocating Memory: Raw=%d bytes, Acc=%d bytes\n", raw_size_bytes, acc_size_bytes);

    // 尝试 PSRAM 分配
    big_raw_buffer = (uint16_t *)heap_caps_malloc(raw_size_bytes, MALLOC_CAP_SPIRAM);
    accumulator_buffer = (uint32_t *)heap_caps_malloc(acc_size_bytes, MALLOC_CAP_SPIRAM);

    // 简单的内存检查逻辑
    if (big_raw_buffer == NULL || accumulator_buffer == NULL) {
        Serial.println("PSRAM Alloc Failed! Trying Internal...");
        if(big_raw_buffer == NULL) big_raw_buffer = (uint16_t *)malloc(raw_size_bytes);
        if(accumulator_buffer == NULL) accumulator_buffer = (uint32_t *)malloc(acc_size_bytes);
    }
    
    if (big_raw_buffer == NULL || accumulator_buffer == NULL) {
        Serial.println("CRITICAL: Memory Alloc Failed!");
        while(1);
    }

    // 清零内存
    memset(big_raw_buffer, 0, raw_size_bytes);
    memset(accumulator_buffer, 0, acc_size_bytes);

    // [NEW] 初始化 IMS 定时器与同步系统
    IMS_HW_Init();

    // --- LVGL 初始化部分 (保持不变) ---
    tft.init();
    tft.setRotation(1);
    tft.invertDisplay(true);
    tft.fillScreen(TFT_BLACK);
    ts.begin();

    lv_init();
    size_t buffer_size = screenWidth * screenHeight * sizeof(lv_color_t);
    buf = (lv_color_t*) heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (buf == NULL) buf = (lv_color_t*) malloc(screenWidth * 32 * sizeof(lv_color_t));
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * (buf==NULL?32:screenHeight));

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();
    ui_SignalSeries = lv_chart_get_series_next(ui_Chart1, NULL);
    lv_chart_set_update_mode(ui_Chart1, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_point_count(ui_Chart1, CHART_POINTS);

    // 4. 启动采集任务
    xTaskCreatePinnedToCore(Task_Acquisition, "IMS_ADC", 4096, NULL, 10, NULL, 1);

    // [REMOVED] 删除了之前的 ledcSetup 测试代码，因为现在由硬件定时器接管了
    
    Serial.println("IMS System Ready. Waiting for Scan Start...");
}

// ==========================================
//  Loop
// ==========================================
void loop() {
    // 逻辑保持不变，只负责刷新 UI
    if (ui_update_needed) {
        if (xSemaphoreTake(dataMutex, 0) == pdTRUE) { 
            if (ui_SignalSeries != NULL) {
                lv_chart_set_ext_y_array(ui_Chart1, ui_SignalSeries, (lv_coord_t*)waveform_buffer);
                lv_chart_refresh(ui_Chart1); 
            }
            if (isScanning && detected_peak_amp > 0) {
                static char buf_time[16];
                static char buf_amp[16];
                sprintf(buf_time, "%.2f ms", detected_peak_time);
                sprintf(buf_amp, "%d", detected_peak_amp);
                
                // 请确保这些 label 在 ui.h 中存在
                if(ui_Label18) lv_label_set_text(ui_Label18, buf_time); 
                if(ui_Label19) lv_label_set_text(ui_Label19, buf_amp);
            }
            ui_update_needed = false;
            xSemaphoreGive(dataMutex);
        }
    }
    lv_timer_handler();
    lv_tick_inc(5);
    delay(5);
}