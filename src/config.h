/*
 * config.h — 输液瓶加热模块 全局配置 & 引脚定义
 *
 * 基于 Netlist_Schematic1_3_2026-06-22.tel + BOM_12761250A_Y2.xls
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* ================================================================
 * 引脚映射（Netlist 精确确认）
 * ============================================================== */
// --- NTC 温度传感器 (模拟分压输入) ---
#define PIN_NTC_CH1   A0   // P3 → R3(10kΩ)→VCC
#define PIN_NTC_CH2   A1   // P2 → R4(10kΩ)→VCC
#define PIN_NTC_CH3   A2   // P8 → R14(10kΩ)→VCC (预留)
#define PIN_NTC_CH4   A3   // P9 → R15(10kΩ)→VCC (预留)

// --- OLED I2C ---
#define PIN_OLED_SDA  A4
#define PIN_OLED_SCL  A5

// --- 按键 (低电平有效, 外部10kΩ上拉到VCC) ---
#define PIN_BTN_SET   2    // SW1
#define PIN_BTN_UP    3    // SW2
#define PIN_BTN_DOWN  4    // SW3

// --- 蜂鸣器 (有源, 高电平响) ---
#define PIN_BUZZER    5

// --- 加热 PWM (MOSFET gate) ---
#define PIN_HEAT_CH1  11   // PWM1 → R1(100Ω) → U3(100N03) Gate
#define PIN_HEAT_CH2  10   // PWM2 → R8(100Ω) → U7(100N03) Gate
#define PIN_HEAT_CH3  9    // PWM3 → R10(100Ω) → U8(100N03) Gate
#define PIN_HEAT_CH4  6    // PWM4 → R12(100Ω) → U9(100N03) Gate

/* ================================================================
 * NTC 参数
 * ============================================================== */
#define NTC_R25       10000.0    // 25℃ 标称阻值 (Ω)
#define NTC_B_VALUE   3950.0     // B 值 (25/50℃)
#define NTC_R_REF     10000.0    // 分压电阻 (Ω, ±1%)
#define NTC_VCC       5.0        // 参考电压 (V)

// --- 故障检测阈值 ---
#define ADC_SHORT_THRESHOLD   20     // ADC < 20  → NTC 短路
#define ADC_OPEN_THRESHOLD    1000   // ADC > 1000 → NTC 断路

// --- 瓶口温度补偿 (NTC 测硅胶面 → 瓶口水温的 offset, 待实测标定) ---
#define TEMP_OFFSET_BOTTLE  0.0

/* ================================================================
 * 温度控制参数
 * ============================================================== */
#define TEMP_MIN       37.0      // 最低设定温度 (℃)
#define TEMP_MAX       41.0      // 最高设定温度 (℃)
#define TEMP_STEP      0.5       // 按键步进 (℃)
#define TEMP_DEFAULT   38.0      // 默认目标温度 (℃)
#define TEMP_OVERHEAT  42.0      // 超温保护阈值 (℃)

/* ================================================================
 * PID 参数（初值, 需实测整定）
 * ============================================================== */
#define PID_KP         20.0
#define PID_KI         0.3
#define PID_KD         4.0
#define PID_SAMPLE_MS  500       // PID 采样周期 (ms)
#define PID_OUTPUT_MAX 255       // PWM 最大占空比 (8-bit)

// --- 软启动 ---
#define SOFTSTART_DURATION_MS  30000   // 前 30s 限制输出 ≤ 50%
#define SOFTSTART_MAX_OUTPUT   128

/* ================================================================
 * 软件 PWM 参数
 * ============================================================== */
#define PWM_PERIOD_MS  1000      // PWM 周期 1 秒 (热惯性大)
#define PWM_TICK_MS    10        // 最小时间片 (ms)

/* ================================================================
 * UI 参数
 * ============================================================== */
#define BTN_DEBOUNCE_MS     50   // 按键消抖 (ms)
#define BTN_LONG_PRESS_MS    2000 // 长按阈值 (ms)
#define UI_REFRESH_MS        500  // 显示刷新周期 (ms)
#define SETTING_TIMEOUT_MS   10000 // 设置模式无操作自动退出 (ms)

// OLED 假设: SSD1306 128×64 I2C 0x3C
#define OLED_ADDR     0x3C
#define OLED_WIDTH    128
#define OLED_HEIGHT   64

/* ================================================================
 * 加热通道数
 * ============================================================== */
#define CHANNEL_COUNT 4
// 当前只测试 1 路 (CH1), 其余屏蔽
#define ACTIVE_CHANNELS_MASK  0b0001   // bit0=CH1

/* ================================================================
 * 调试开关
 * ============================================================== */
// 启用后每 500ms 通过 Serial 打印 setpoint, temp, output (用于 PID 整定)
#define SERIAL_PLOTTER_ENABLE  1

#endif // CONFIG_H
