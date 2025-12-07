// ==========================================
// IMS 分析仪 - UI+ADC
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
// --- 1. 硬件引脚定义 ---
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define CTP_INT_PIN 4
#define CTP_RST_PIN 13       
#define TOUCH_RAW_WIDTH 320  
#define TOUCH_RAW_HEIGHT 480 

#define TEST_SIGNAL_PIN 48

// --- 2. 屏幕与图表参数 ---
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;
#define CHART_POINTS 200    // 图表分辨率 (必须与 SquareLine 一致)

// IMS 核心参数
#define IMS_SAMPLE_RATE     1000000 // 1MSPS
#define IMS_DURATION_MS     24      // 采样时长 24ms
#define RAW_DATA_LEN        (IMS_SAMPLE_RATE * IMS_DURATION_MS / 1000) // = 24000点
// 定义一个全局指针，指向大内存区域
// 初始化为 NULL，防止未分配直接使用导致崩溃
uint16_t *big_raw_buffer = NULL;


// --- 3. 全局对象 ---
TFT_eSPI tft = TFT_eSPI();
FT6336 ts = FT6336(I2C_SDA_PIN, I2C_SCL_PIN, CTP_INT_PIN, CTP_RST_PIN, TOUCH_RAW_WIDTH, TOUCH_RAW_HEIGHT);
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;

// --- 4. 业务逻辑控制变量 ---
volatile bool isScanning = false;       // 扫描状态开关
SemaphoreHandle_t dataMutex;            // 数据锁
lv_chart_series_t * ui_SignalSeries;    // 图表线条句柄

// 共享数据区
int16_t waveform_buffer[CHART_POINTS];  // 最终要显示的波形数组
float detected_peak_time = 0.0;         // 识别到的漂移时间
int detected_peak_amp = 0;              // 识别到的幅值
bool ui_update_needed = false;          // 标志位：通知 UI 刷新

// =====================================================================
// 函数声明
// =====================================================================
void Task_Acquisition(void *pvParameters); 
void OnScanClick(lv_event_t * e);
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p );
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data );

// ==========================================
//  任务 1 (Core 1): 真实 ADC 采集与信号处理
// ==========================================
void Task_Acquisition(void *pvParameters) {
    (void) pvParameters;
    
    while (true) {
        // 只有当点击了“开始扫描” 且 内存分配成功时才运行
        if (isScanning && big_raw_buffer != NULL) {
            
            // -----------------------------------------------------
            // 阶段 1: 硬件高速采集 (占用约 25ms)
            // -----------------------------------------------------
            
            // TODO: 这里未来需要添加一行代码打开离子门 (Open Gate)
            // digitalWrite(ION_GATE_PIN, HIGH); delayMicroseconds(200); digitalWrite(ION_GATE_PIN, LOW);
            
            // 调用刚才写的驱动，一口气吸入 25000 个点
            // 这时候 CPU 会全速运转 SPI，不再有 delay
            IMS_ADC_ReadBurst(big_raw_buffer, RAW_DATA_LEN);
            
            // -----------------------------------------------------
            // 阶段 2: 数据压缩与显示 (Peak Hold 算法)
            // -----------------------------------------------------
            
            // 尝试获取锁，准备更新 UI 数据
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                
                // 计算压缩比：25000点 / 200像素 = 125
                // 也就是每 125 个原始数据合成 1 个屏幕像素
                int ratio = RAW_DATA_LEN / CHART_POINTS; 
                
                int global_max_val = 0;
                int global_max_idx = 0;

                // 遍历屏幕的每一个像素点
                for (int i = 0; i < CHART_POINTS; i++) {
                    int local_max = 0;
                    
                    // --- 峰值保持算法 (解决 50us 窄峰看不见的问题) ---
                    // 在属于这个像素的 125 个原始数据中找最大值
                    for (int j = 0; j < ratio; j++) {
                        // 防止越界安全检查
                        if ((i * ratio + j) >= RAW_DATA_LEN) break;

                        // 取出原始数据
                        uint16_t val = big_raw_buffer[i * ratio + j];
                        
                        // 【非常重要】大小端转换 (Byte Swap)
                        // SPI 传回来是 [高8位][低8位]，但在 ESP32 内存里这代表错误的值
                        // 我们需要交换一下位置
                        val = (val << 8) | (val >> 8);

                        // 记录这微小时间段内的最大值
                        if (val > local_max) local_max = val;
                    }
                    
                    // 将找到的峰值赋给显示缓存 (除以16是为了适应屏幕高度)
                    waveform_buffer[i] = local_max / 16 ; 
                    
                    // 顺便记录整张谱图的最高峰，用于显示数值
                    if (local_max > global_max_val) {
                        global_max_val = local_max;
                        global_max_idx = i * ratio; // 记录原始索引位置
                    }
                }
                
                // 计算物理时间：索引 * 1us (因为是 1MSPS)
                detected_peak_time = (float)global_max_idx / 1000.0; // us -> ms
                detected_peak_amp = global_max_val / 16;

                // 标记刷新，解锁
                ui_update_needed = true;
                xSemaphoreGive(dataMutex);
            }
            
            // 采集完成一次，稍微休息一下，控制 FPS
            // 例如延时 100ms，代表每秒刷新 10 次谱图
            vTaskDelay(pdMS_TO_TICKS(100)); 
            
        } else {
            // 暂停状态，降低 CPU 占用
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ==========================================
//  SquareLine 事件回调函数
// ==========================================
void OnScanClick(lv_event_t * e) {
    isScanning = !isScanning;
    
    lv_obj_t * ui_Button9 = lv_event_get_target(e); 
    lv_obj_t * label = lv_obj_get_child(ui_Button9, 0); 

    if (isScanning) {
        lv_label_set_text(label, "暂停扫描"); // 修改文案以体现真实功能
        lv_obj_set_style_bg_color(ui_Button9, lv_color_hex(0x00AA00), LV_PART_MAIN); 
        Serial.println("Action: ADC Start");
    } else {
        lv_label_set_text(label, "开始扫描");
        lv_obj_set_style_bg_color(ui_Button9, lv_color_hex(0x0869B4), LV_PART_MAIN); 
        Serial.println("Action: ADC Stop");
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
    
    // 【新增 2】初始化 ADC 芯片
    // 这一步非常重要，必须在任务启动前完成
    IMS_ADC_Init(); 
    Serial.println("ADC Hardware Initialized.");

    // 计算所需字节数 (每个数据是 uint16_t，占 2 字节)
    // 25000 * 2 = 50,000 Bytes (约 48.8KB)
    size_t buffer_size_bytes = RAW_DATA_LEN * sizeof(uint16_t);
    Serial.printf("Attempting to allocate %d bytes for ADC buffer...\n", buffer_size_bytes);
    // 1. 优先尝试从 PSRAM (SPIRAM) 分配
    // MALLOC_CAP_SPIRAM: 指定从外部 PSRAM 分配
    big_raw_buffer = (uint16_t *)heap_caps_malloc(buffer_size_bytes, MALLOC_CAP_SPIRAM);
    if (big_raw_buffer != NULL) {
        Serial.println("Success! Buffer allocated in PSRAM (External Memory).");
    } else {
        // 2. 如果 PSRAM 分配失败（或者板子没开启 PSRAM），回退尝试内部 RAM
        Serial.println("Warning: PSRAM allocation failed. Trying Internal RAM...");
        big_raw_buffer = (uint16_t *)malloc(buffer_size_bytes);
        if (big_raw_buffer != NULL) {
            Serial.println("Success! Buffer allocated in Internal RAM.");
        } else {
            // 3. 如果内部 RAM 也不够，那是严重的致命错误
            Serial.println("CRITICAL ERROR: Failed to allocate memory! System Halted.");
            while (1) { delay(1000); } // 死循环卡住，防止后面程序崩溃
        }
    }
    
    // 4. (可选) 这是一个好习惯：把内存清零，防止显示上次残留的垃圾数据
    if (big_raw_buffer != NULL) {
        memset(big_raw_buffer, 0, buffer_size_bytes);
    }


    tft.init();
    tft.setRotation(1);
    tft.invertDisplay(true);
    tft.fillScreen(TFT_BLACK);
    ts.begin();

    // 3. LVGL 初始化
    lv_init();
    size_t buffer_size = screenWidth * screenHeight * sizeof(lv_color_t);
    buf = (lv_color_t*) heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    
    if (buf == NULL) {
        Serial.println("PSRAM Fail. Using internal RAM.");
        buf = (lv_color_t*) malloc(screenWidth * 32 * sizeof(lv_color_t));
        lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 32 );
    } else {
        Serial.println("PSRAM OK. Full buffer.");
        lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight );
    }

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
    
    // 获取图表句柄
    ui_SignalSeries = lv_chart_get_series_next(ui_Chart1, NULL);
    lv_chart_set_update_mode(ui_Chart1, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_point_count(ui_Chart1, CHART_POINTS);

    // 4. 启动采集任务 (Core 1)
    xTaskCreatePinnedToCore(
        Task_Acquisition, "IMS_ADC", 4096, NULL, 10, NULL, 1
    ); // 改名后的任务

    // 配置 LEDC 通道 0，频率 5 Hz，分辨率 8 位
    // 5Hz 意味着波形每秒跳变 5 次，在图表上很容易看清
    ledcSetup(0, 1000, 8); 
    
    // 将通道 0 绑定到测试引脚
    ledcAttachPin(TEST_SIGNAL_PIN, 0);
    
    // 输出 50% 占空比的方波 (256/2 = 128)
    ledcWrite(0, 128); 


    // 1. 设置引脚为输出模式
    // pinMode(TEST_SIGNAL_PIN, OUTPUT);
    // // 2. 强制拉高 (输出 3.3V)
    // digitalWrite(TEST_SIGNAL_PIN, HIGH);


    // 查看 loop() 属于哪个核心
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    Serial.print("loop() task core: ");
    Serial.println(xTaskGetAffinity(h));

    Serial.println("IMS System Ready.");
}

// ==========================================
//  Loop (Core 1 UI 刷新)
// ==========================================
void loop() {
    // 1. 检查是否有新波形需要绘制
    if (ui_update_needed) {
        if (xSemaphoreTake(dataMutex, 0) == pdTRUE) { 
            
            // A. 刷新波形
            if (ui_SignalSeries != NULL) {
                // 将采集到的 ADC 数组直接推给图表
                lv_chart_set_ext_y_array(ui_Chart1, ui_SignalSeries, (lv_coord_t*)waveform_buffer);
                lv_chart_refresh(ui_Chart1); 
            }

            // B. 刷新数值 (峰值检测结果)
            if (isScanning && detected_peak_amp > 0) {
                static char buf_time[16];
                static char buf_amp[16];
                
                sprintf(buf_time, "%.2f ms", detected_peak_time);
                sprintf(buf_amp, "%d", detected_peak_amp);
                
                // 注意：这里需要根据你 SquareLine 里真实的 Label 名字来改
                // 假设 ui.h 里叫 ui_Label18 和 ui_Label19
                if(ui_Label18) lv_label_set_text(ui_Label18, buf_time); 
                if(ui_Label19) lv_label_set_text(ui_Label19, buf_amp);
            } else {
                 if(ui_Label18) lv_label_set_text(ui_Label18, "--.--");
                 if(ui_Label19) lv_label_set_text(ui_Label19, "STOP");
            }
            
            ui_update_needed = false;
            xSemaphoreGive(dataMutex);
        }
    }

    // 2. LVGL 保持心跳
    lv_timer_handler();
    lv_tick_inc(5);
    delay(5);
}