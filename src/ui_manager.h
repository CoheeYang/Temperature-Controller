/*
 * ui_manager.h — OLED 显示 + 按键处理
 *
 * 页面: 主页(温度概览) / 设置页(改目标温度) / 报警页
 * 3 按键: SET(D2)/UP(D3)/DOWN(D4), 低电平有效
 */

#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== 系统状态 =====
enum UIState {
    UI_NORMAL   = 0,  // 主页
    UI_SETTING  = 1,  // 设置
    UI_ALARM    = 2   // 报警
};

// ===== 按键事件 =====
enum BtnEvent {
    BTN_NONE        = 0,
    BTN_SET_SHORT   = 1,
    BTN_SET_LONG    = 2,   // 长按 SET
    BTN_UP          = 3,
    BTN_DOWN        = 4,
};

class UIManager {
public:
    UIManager();

    // 初始化 OLED + 按键引脚
    void begin();

    // 每循环一次调用, 返回按键事件
    BtnEvent pollButtons();

    // 渲染当前页面
    // 参数: temps[4], setpoints[4], outputs[4], faults[4], activeMask, currentCh, uiState
    void render(float temps[4], float setpoints[4], uint8_t outputs[4],
                bool faults[4], uint8_t activeMask, uint8_t currentCh, UIState state,
                float editingTemp);

    // 蜂鸣控制
    void beepOn();
    void beepOff();
    void beepPattern(bool alarm);  // 报警=急促, 按键=短滴

    Adafruit_SSD1306& display() { return _oled; }

private:
    Adafruit_SSD1306 _oled;
    unsigned long _lastBtnRead;
    uint8_t _lastBtnState;      // bit0=SET, bit1=UP, bit2=DOWN
    unsigned long _btnPressTime[3];  // 各按键按下时刻
    bool _btnHandled[3];        // 已处理标志(防重复触发长按)
    unsigned long _lastBeep;
    uint8_t _beepPhase;

    void drawMainPage(float temps[4], float setpoints[4], uint8_t outputs[4],
                      bool faults[4], uint8_t activeMask, uint8_t currentCh);
    void drawSettingPage(uint8_t ch, float editingTemp);
    void drawAlarmPage(uint8_t ch, float temp, const char* reason);
};

#endif // UI_MANAGER_H
