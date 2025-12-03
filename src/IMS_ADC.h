#ifndef IMS_ADC_H
#define IMS_ADC_H

#include <Arduino.h>

// --- 这里是对外公开的功能接口 ---

/**
 * @brief 初始化 ADC 相关的引脚和 SPI 总线
 * 包括：引脚模式设置、SPI 启动、硬件复位、量程配置
 */
void IMS_ADC_Init();

/**
 * @brief 读取一次 ADC 的原始数值 (Raw Data)
 * @return 16位无符号整数 (0 ~ 65535)
 */
uint16_t IMS_ADC_ReadRaw();

/**
 * @brief 读取一次转换后的电压值
 * @return 浮点数电压 (0.0V ~ 5.12V)
 */
float IMS_ADC_ReadVoltage();

#endif