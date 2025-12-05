#include "IMS_ADC.h"
#include <SPI.h>
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

// 定义引脚 
#define ADS_RST   14
#define ADS_CS    15
#define ADS_MOSI  16  // SDI
#define ADS_MISO  17  // SDO0
#define ADS_SCLK  18

// 定义 SPI 速度 ：1M * 32bit
#define ADS_SPI_SPEED 40000000 

// 创建一个独立的 SPI 实例 (使用 FSPI 硬件通道)
SPIClass adcSPI(FSPI);

// ================= 内部私有函数 =================
// 仅供本文件内部调用，用于底层的 SPI 收发
uint16_t _adc_transfer(uint32_t cmd) {
    uint16_t result = 0;
    // 启动事务：配置速度、高位优先、模式0
    adcSPI.beginTransaction(SPISettings(ADS_SPI_SPEED, MSBFIRST, SPI_MODE0));
    
    digitalWrite(ADS_CS, LOW); // 拉低片选，开始说话
    
    // 发送高16位 (通常是命令)
    uint16_t highWord = adcSPI.transfer16((cmd >> 16) & 0xFFFF);
    // 发送低16位 (通常是数据或填充)
    uint16_t lowWord = adcSPI.transfer16(cmd & 0xFFFF);
    
    digitalWrite(ADS_CS, HIGH); // 拉高片选，结束说话
    
    adcSPI.endTransaction();
    
    // ADS8681 在 SDO0 线上传回的是高16位部分
    return highWord;
}

// ================= 对外公开函数的实现 =================

void IMS_ADC_Init() {
    // 1. 引脚模式初始化
    pinMode(ADS_CS, OUTPUT);
    pinMode(ADS_RST, OUTPUT);
    digitalWrite(ADS_CS, HIGH); // 默认不选中

    // 2. 硬件复位 (Reset)
    digitalWrite(ADS_RST, LOW);
    delay(1);
    digitalWrite(ADS_RST, HIGH);
    delay(20); // 等待芯片复位完成

    // 3. SPI 总线初始化
    // 参数顺序: SCLK, MISO, MOSI, SS
    adcSPI.begin(ADS_SCLK, ADS_MISO, ADS_MOSI, ADS_CS);

    // 4. 发送配置指令：将量程设为 0 ~ 5.12V (单极性)
    // 命令码构造: Write(0xD0) + Addr(0x14) + Data(0x000B)
    _adc_transfer(0xD014000B);
    
    delay(10); // 等待配置生效
    
    // 5. 发送一次 NOP (空指令) 来刷新内部管道
    _adc_transfer(0x00000000);
}

uint16_t IMS_ADC_ReadRaw() {
    // 发送 NOP (0x00000000) 读取最新转换结果
    // 由于没接 RVS 线，我们依赖 SPI 传输本身的耗时来等待转换
    // (传输 32bit @ 20MHz 需要 1.6us，大于转换所需的 0.665us，所以是安全的)
    return _adc_transfer(0x00000000);
}

float IMS_ADC_ReadVoltage() {
    uint16_t raw = IMS_ADC_ReadRaw();
    // 将 0-65535 映射到 0-5.12V
    return (raw / 65536.0f) * 5.12f;
}

// 
void IMS_ADC_ReadBurst(uint16_t *buffer, size_t count) {
    // 1. 启动 SPI 事务 (保持 40MHz)
    adcSPI.beginTransaction(SPISettings(ADS_SPI_SPEED, MSBFIRST, SPI_MODE0));

    // 2. 准备 GPIO 寄存器操作 (避开 digitalWrite 的巨大开销)
    // 注意：这只适用于 GPIO 0-31。你的 CS 是 15，完全适用。
    // 如果 CS > 31，需要用 GPIO.out1_w1tc.val
    const uint32_t cs_mask = (1 << ADS_CS); 

    // 3. 循环展开 (Loop Unrolling)
    // 原理：减少 for 循环 i++ 和判断 i < count 的次数，节省 CPU 周期
    // 我们一次读 10 个数据，这样循环开销就减少了 90%
    size_t main_loop = count / 10;
    size_t remainder = count % 10;
    
    uint16_t *ptr = buffer;

    // --- 主循环 (每次处理 10 个点) ---
    for (size_t i = 0; i < main_loop; i++) {
        
        // 第 1 个点
        GPIO.out_w1tc = cs_mask;       // [极速] CS 拉低 (Clear)
        *ptr++ = adcSPI.transfer16(0);     // 读数据 (耗时约 0.4us)
        GPIO.out_w1ts = cs_mask;       // [极速] CS 拉高 (Set)
        
        // 第 2 个点
        GPIO.out_w1tc = cs_mask; 
        *ptr++ = adcSPI.transfer16(0); 
        GPIO.out_w1ts = cs_mask;

        // 第 3 个点
        GPIO.out_w1tc = cs_mask; 
        *ptr++ = adcSPI.transfer16(0); 
        GPIO.out_w1ts = cs_mask;

        // 第 4 个点
        GPIO.out_w1tc = cs_mask; 
        *ptr++ = adcSPI.transfer16(0); 
        GPIO.out_w1ts = cs_mask;

        // 第 5 个点
        GPIO.out_w1tc = cs_mask; 
        *ptr++ = adcSPI.transfer16(0); 
        GPIO.out_w1ts = cs_mask;

        // 第 6 个点
        GPIO.out_w1tc = cs_mask; 
        *ptr++ = adcSPI.transfer16(0); 
        GPIO.out_w1ts = cs_mask;

        // 第 7 个点
        GPIO.out_w1tc = cs_mask; 
        *ptr++ = adcSPI.transfer16(0); 
        GPIO.out_w1ts = cs_mask;

        // 第 8 个点
        GPIO.out_w1tc = cs_mask; 
        *ptr++ = adcSPI.transfer16(0); 
        GPIO.out_w1ts = cs_mask;

        // 第 9 个点
        GPIO.out_w1tc = cs_mask; 
        *ptr++ = adcSPI.transfer16(0); 
        GPIO.out_w1ts = cs_mask;

        // 第 10 个点
        GPIO.out_w1tc = cs_mask; 
        *ptr++ = adcSPI.transfer16(0); 
        GPIO.out_w1ts = cs_mask;
    }

    // --- 处理剩下的尾数 (不足10个的部分) ---
    for (size_t i = 0; i < remainder; i++) {
        GPIO.out_w1tc = cs_mask;
        *ptr++ = adcSPI.transfer16(0);
        GPIO.out_w1ts = cs_mask;
    }

    // 4. 结束事务
    adcSPI.endTransaction();
}