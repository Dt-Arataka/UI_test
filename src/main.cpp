// ==========================================
// IMS 分析仪 - PlatformIO 移植版
// 硬件: ESP32-S3 (N16R8) + ST7796U + FT6336
// ==========================================

#include <Arduino.h> 

#include <SPI.h>
#include <Wire.h>
#include <TFT_eSPI.h> 
#include <FT6336.h>       
#include <lvgl.h>       

#include "ui/ui.h"        

// --- 1. 硬件引脚定义 ---
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define CTP_INT_PIN 4
#define CTP_RST_PIN 13       
#define TOUCH_RAW_WIDTH 320  
#define TOUCH_RAW_HEIGHT 480 

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
// 【修改】函数前向声明 (Prototypes)
// C++ 编译器要求函数在使用前必须声明，这是与 .ino 最大的区别
// =====================================================================
void addGaussianPeak(int16_t* buffer, int pos, int width, int height);
void Task_PhysicsEngine(void *pvParameters);
void OnScanClick(lv_event_t * e);
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p );
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data );

// ==========================================
//  辅助算法：生成高斯峰 (模拟波形用)
// ==========================================
// pos: 峰中心位置(0-199), width: 峰宽, height: 峰高
void addGaussianPeak(int16_t* buffer, int pos, int width, int height) {
    for (int i = 0; i < CHART_POINTS; i++) {
        // 简单的钟形曲线公式
        float val = height * exp(-0.5 * pow((i - pos) / (float)width, 2));
        buffer[i] += (int16_t)val; // 叠加到现有波形上
        
        // 限制最大值防止溢出
        if (buffer[i] > 4095) buffer[i] = 4095;
    }
}

// ==========================================
//  任务 1 (Core 1): 物理引擎与信号处理
// ==========================================
void Task_PhysicsEngine(void *pvParameters) {
    (void) pvParameters;
    
    // 模拟变量
    float rip_height = 3500.0;  // RIP 峰高度 (基准)
    float target_conc = 0.0;    // 目标物质浓度 (0.0 - 1.0)
    
    // 物理参数
    int rip_base_pos = 40;           // RIP 在约 5ms 处 (40/200 * 25ms)
    int target_base_pos = 120;       // 目标物质在约 15ms 处
    
    while (true) {
        // 1. 状态机：模拟物质进入和清洗的过程
        if (isScanning) {
            // 开启扫描：浓度慢慢上升
            if (target_conc < 1.0) target_conc += 0.02; 
        } else {
            // 停止扫描：浓度慢慢下降 (清洗过程)
            if (target_conc > 0.0) target_conc -= 0.05;
            if (target_conc < 0.0) target_conc = 0.0;
        }

        // 2. 锁定内存，开始合成波形
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            
            // A. 清空并添加底噪
            for(int i=0; i<CHART_POINTS; i++) {
                waveform_buffer[i] = random(200, 250); // 基线底噪
            }

            // 真实仪器受温度气流影响，峰的位置会轻微跳动
            // random(-2, 3) 会产生 -2, -1, 0, 1, 2 的随机偏移
            // ---------------------------------------------------------
            int current_rip_pos = rip_base_pos + random(-1, 2);
            int current_target_pos = target_base_pos + random(-3, 4);

            // B. 叠加 RIP 峰 (一直存在)
            // 模拟电荷竞争：如果目标物质多了，RIP 会稍微降低
            float current_rip = rip_height - (target_conc * 800);
            addGaussianPeak(waveform_buffer, current_rip_pos, 8, current_rip);

            // C. 叠加目标物质峰 (根据浓度)
            if (target_conc > 0.01) {
                float current_target = target_conc * 2500; // 最大高度 2500
                addGaussianPeak(waveform_buffer, current_target_pos, 10, current_target);
                
                // 我们不直接用 current_target_pos，而是去数组里“实测”它在哪
                // 关键：从 index 70 开始找 (跳过 RIP 所在的 0-60 区域)
                // ---------------------------------------------------------
                int search_start_index = 70; // 约 8.75ms 之后
                int max_val = 0;
                int max_idx = 0;

                for (int k = search_start_index; k < CHART_POINTS; k++) {
                    if (waveform_buffer[k] > max_val) {
                        max_val = waveform_buffer[k];
                        max_idx = k;
                    }
                }

                // 只有当找到的峰足够高 (大于底噪+余量，例如400) 时才更新显示
                if (max_val > 400) {
                    // 实时计算：把数组下标转回时间 (0-200 -> 0-25ms)
                    detected_peak_time = (float)max_idx / CHART_POINTS * 25.0;
                    detected_peak_amp = max_val;
                }
                
            } else {
                // 浓度太低或是停止状态，归零
                detected_peak_time = 0.0;
                detected_peak_amp = 0;
            }

            ui_update_needed = true; // 告诉 UI 可以画了
            xSemaphoreGive(dataMutex); // 解锁
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 刷新率约 30FPS
    }
}

// ==========================================
//  SquareLine 事件回调函数 (按钮点击)
// ==========================================
// 请在 SquareLine 里把按钮的 Click 事件绑定到这个函数名: OnScanClick
void OnScanClick(lv_event_t * e) {
    // 1. 切换状态
    isScanning = !isScanning;
    
    // 2. 获取按钮和里面的 Label
    // 【修改建议】在 C++ 中 e->target 是 void*，最好使用 LVGL 提供的标准获取函数
    lv_obj_t * ui_Button9 = lv_event_get_target(e); 
    lv_obj_t * label = lv_obj_get_child(ui_Button9, 0); // 获取按钮里的第一个子控件(Label)

    if (isScanning) {
        // --- 变为：暂停扫描 ---
        lv_label_set_text(label, "暂停扫描");
        lv_obj_set_style_bg_color(ui_Button9, lv_color_hex(0x00AA00), LV_PART_MAIN); // 变绿色
        Serial.println("State: SCANNING");
    } else {
        // --- 变为：开始扫描 ---
        lv_label_set_text(label, "开始扫描");
        lv_obj_set_style_bg_color(ui_Button9, lv_color_hex(0x0869B4), LV_PART_MAIN); // 变回蓝色 (根据你原来的颜色调)
        Serial.println("State: PAUSED");
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
        data->point.x = ts.points[0].y;         // 翻转坐标 (根据你的屏幕调整)
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
    tft.init();
    tft.setRotation(1);
    tft.invertDisplay(true);
    tft.fillScreen(TFT_BLACK);
    ts.begin();

    // 3. LVGL 初始化
    lv_init();
    
    // 计算全屏所需的字节数
    size_t buffer_size = screenWidth * screenHeight * sizeof(lv_color_t);

    // 使用 heap_caps_malloc 强制在 PSRAM (SPIRAM) 中分配
    buf = (lv_color_t*) heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    
    // 添加分配失败的“后悔药” (Fallback)
    if (buf == NULL) {
        Serial.println("PSRAM Malloc FAILED! 使用内部 RAM 局部缓冲 (可能会闪烁)");
        
        // 如果 PSRAM 没开或满了，回退到旧方案：只申请 1/10 屏幕大小
        // 使用普通 malloc (默认分配内部 RAM)
        buf = (lv_color_t*) malloc(screenWidth * 32 * sizeof(lv_color_t));
        
        // 初始化局部缓冲
        lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 32 );
    } else {
        Serial.println("PSRAM Malloc SUCCESS! 全屏缓冲已启用 (丝滑模式)");
        
        // 初始化全屏缓冲
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

    // 4. UI 初始化
    ui_init();
    
    // 5. 获取图表句柄 (假设 SquareLine 里图表叫 ui_Chart1)
    // Series 指针必须在 ui_init 后获取
    ui_SignalSeries = lv_chart_get_series_next(ui_Chart1, NULL);
    // 设置为直接刷新模式，不显示点，只显示线
    lv_chart_set_update_mode(ui_Chart1, LV_CHART_UPDATE_MODE_SHIFT);
    // 确保点数一致
    lv_chart_set_point_count(ui_Chart1, CHART_POINTS);

    // 6. 启动物理模拟任务 (Core 1)
    xTaskCreatePinnedToCore(
        Task_PhysicsEngine, "Physics", 4096, NULL, 1, NULL, 1
    );

    Serial.println("IMS System Started.");
}

// ==========================================
//  Loop (Core 0 UI 刷新)
// ==========================================
void loop() {
    // 1. 检查是否有新波形需要绘制
    if (ui_update_needed) {
        if (xSemaphoreTake(dataMutex, 0) == pdTRUE) { // 尝试获取锁
            
            // A. 刷新波形
            if (ui_SignalSeries != NULL) {
                lv_chart_set_ext_y_array(ui_Chart1, ui_SignalSeries, (lv_coord_t*)waveform_buffer);
                lv_chart_refresh(ui_Chart1); // 强制重绘
            }

            // B. 刷新下方的数值 (只有在有物质时才显示)
            if (isScanning && detected_peak_amp > 100) {
                // 格式化字符串
                static char buf_time[16];
                static char buf_amp[16];
                
                sprintf(buf_time, "%.2f ms", detected_peak_time);
                sprintf(buf_amp, "%d", detected_peak_amp);
                
                // 这里需要你替换成你真实的 Label 变量名
                // 如果你的 ui.h 里没有 ui_Label18/19，编译会报错，请修改为你真实的名字
                if(ui_Label18) lv_label_set_text(ui_Label18, buf_time); 
                if(ui_Label19) lv_label_set_text(ui_Label19, buf_amp);
            } else {
                // 没扫到或暂停时，显示横杠
                 if(ui_Label18) lv_label_set_text(ui_Label18, "--.--");
                 if(ui_Label19) lv_label_set_text(ui_Label19, "---");
            }
            
            ui_update_needed = false;
            xSemaphoreGive(dataMutex);
        }
    }

    // 2. LVGL 心跳
    lv_timer_handler();
    lv_tick_inc(5);
    delay(5);
}