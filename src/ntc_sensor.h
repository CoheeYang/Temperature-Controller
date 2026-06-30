/*
 * ntc_sensor.h — NTC 温度传感器读取
 *
 * 电路: VCC(5V) → 10kΩ → ADC ← NTC → GND
 * 温度↑ → NTC 阻值↓ → ADC 电压↓
 * 短路: ADC≈0;  断路: ADC≈1023
 */

#ifndef NTC_SENSOR_H
#define NTC_SENSOR_H

#include "config.h"

// ===== 多路滤波窗口大小 =====
#define NTC_FILTER_SIZE  8   // 移动平均窗口

enum NTCStatus {
    NTC_OK       = 0,
    NTC_SHORT    = 1,  // 传感器短路
    NTC_OPEN     = 2,  // 传感器断路
    NTC_DISABLED = 3   // 该通道未启用
};

class NTCSensor {
public:
    NTCSensor();

    // 绑定 ADC 引脚
    void attach(uint8_t pin);

    // 读取原始 ADC 值 (已滤波)
    int   readADC();

    // 故障检测
    NTCStatus checkFault(int adc);

    // ADC → 温度 (℃)
    float adcToTempC(int adc);

    // 便捷: 读 ADC + 查故障 + 转温度
    float readTemperature(NTCStatus &status);

    // 获取最后状态
    NTCStatus lastStatus() const { return _lastStatus; }

private:
    uint8_t   _pin;
    uint16_t  _buf[NTC_FILTER_SIZE];
    uint8_t   _bufIdx;
    bool      _bufFull;
    NTCStatus _lastStatus;
};

#endif // NTC_SENSOR_H
