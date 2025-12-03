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
    
    // 简单的峰值搜索辅助变量
    int local_max_val = 0;
    int local_max_idx = 0;

    while (true) {
        // 1. 如果处于扫描状态，开始采集
        if (isScanning) {
            
            // 锁定内存，防止 UI 在我们写数据的时候读数据导致花屏
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                
                local_max_val = 0;
                local_max_idx = 0;

                // ---【核心修改：采集循环】---
                // 这里我们快速读取 200 次 ADC，填满一个屏幕的波形
                // 此时还没做离子门同步，所以这是一个“滚动示波器”模式
                for(int i = 0; i < CHART_POINTS; i++) {
                    
                    // 1. 读取真实硬件数据 (16位: 0-65535)
                    uint16_t raw_val = IMS_ADC_ReadRaw(); 

                    // 2. 数据缩放 (适配 UI)
                    // ADS8681 是 16位 (0-65535)
                    // SquareLine 图表通常默认范围较小 (0-1000 或 0-4096)
                    // 这里我们将数据除以 10 (或右移 4 位)，让波形能完整显示在屏幕上
                    int16_t scaled_val = raw_val / 16; 
                    
                    waveform_buffer[i] = scaled_val;

                    // 3. 顺便找一下最大值 (简单的峰值检测)
                    if (scaled_val > local_max_val) {
                        local_max_val = scaled_val;
                        local_max_idx = i;
                    }
                    
                    // 控制采样率：
                    // 如果不加延时，ESP32 读取这 200 个点可能只需要 1ms
                    // 真实的 IMS 谱图通常横坐标总长是 20ms - 30ms
                    delayMicroseconds(85); // 可选：调节横轴时间跨度
                }

                // --- 简单的信号处理结果更新 ---
                // 只有当信号强度大于一定底噪 (例如 raw > 500 => scaled > 50)
                if (local_max_val > 50) {
                    // 假设横轴总长对应 25ms (根据你的实际采样率计算)
                    detected_peak_time = (float)local_max_idx / CHART_POINTS * 25.0;
                    detected_peak_amp = local_max_val;
                } else {
                    detected_peak_time = 0.0;
                    detected_peak_amp = 0;
                }

                ui_update_needed = true; // 告诉 UI 可以画了
                xSemaphoreGive(dataMutex); // 解锁
            }
        } else {
            // 如果暂停扫描，休息一下，避免死循环占用 CPU
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // 这里的延时决定了屏幕刷新的“帧率”
        // 50ms = 20FPS，对于人眼观察足够了
        vTaskDelay(pdMS_TO_TICKS(50)); 
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
        Task_Acquisition, "IMS_ADC", 4096, NULL, 1, NULL, 1
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



    Serial.println("IMS System Ready.");
}

// ==========================================
//  Loop (Core 0 UI 刷新)
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