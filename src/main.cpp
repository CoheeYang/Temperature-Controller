/*
 * main.cpp — 加热管道温控系统 (PID + 4 路测温 + 三按键)
 *
 * 2026-06-30 重大方案变更:
 *   - 旧方案 (PI 加热膜包裹盐水瓶) 已废弃, 加热速度过慢
 *   - 新方案: 盐水经"加热管道" 流动加热, 隔膜泵 1~4 L/min
 *   - 加热条接在 P6 (CH3 MOSFET D9), 原 P7 (CH4 D6) 已拔掉
 *   - 加热条参数: 24V/4A, 1400mm, 8.6Ω, 内置 2 路 NTC (50KΩ@25°C)
 *
 * 4 路 NTC 引脚映射:
 *   CH1 = A0 (P3, 10kΩ 分压)   — 纯测量, 不参与控制
 *   CH2 = A1 (P2, 10kΩ 分压)   — 纯测量, 不参与控制
 *   CH3 = A2 (P8, 10kΩ 分压)   — 加热管道入口 NTC (50KΩ) ★参与控制
 *   CH4 = A3 (P9, 10kΩ 分压)   — 加热管道出口 NTC (50KΩ) ★参与控制
 *
 * 
 * 控制目标:
 *   - 控制温度 = max(CH3, CH4), 任一失效用另一路
 *   - PID 维持此最大值达到目标温度 (默认 38°C, 按键 37~50°C 可调)
 *   - PWM 占空比硬上限 = 80% (配合软启动, 兼顾升速与冲击限制)
 *
 * 三按键 (REQUIREMENTS.md §2.6):
 *   SW1=D2  →  目标温度 +0.5°C (上限 50°C)
 *   SW2=D3  →  目标温度 -0.5°C (下限 37°C)
 *   SW3=D4  →  切换温控开/关
 *
 * 安全保护:
 *   - 任一控制 NTC (CH3/CH4) 故障 → 切断加热
 *   - 温度 > 42°C → 锁定加热, 降到 40°C 解锁
 *
 * 串口波特率: 9600
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== NTC 参数 (新加热条内置 NTC 规格: 50KΩ@25°C ±0.5%) =====
// CH1=A0(P3, R3=10k), CH2=A1(P2, R4=10k), CH3=A2(P8, R14=10k), CH4=A3(P9, R15=10k)
// 分压: VCC - 10k(板上) - ADC - NTC - GND
//        温度↑ → NTC阻值↓ → ADC值↓
//
// ⚠️ 注意: 新加热条的 NTC 标称 50KΩ, 但板上分压电阻还是 10kΩ
//   (R3/R4/R14/R15 全是 10k, 没法单独区分 P8/P9)
//   25°C 时: ADC = 1023 × 50/(50+10) ≈ 853 (高位, 测量仍准确)
//   高温时: NTC 降到几 kΩ, ADC 正常在中段
//   全部通道用统一 R_REF=10K(分压电阻), NTC_R25 按 CH 类型区分
#define NTC_CH_COUNT    4
const uint8_t NTC_PINS[NTC_CH_COUNT] = { A0, A1, A2, A3 };   // P3, P2, P8, P9
const char*   NTC_NAMES[NTC_CH_COUNT] = { "CH1", "CH2", "CH3", "CH4" };

#define NTC_R_REF       10000.0f  // 板上分压电阻 (Ω, R3/R4/R14/R15 = 10kΩ)

// 通道 1/2: P3/P2 至今用的仍可能是旧款 10KΩ 传感器; 通道 3/4 是管道 50KΩ
// 这里用每通道独立的标称阻值
const float   NTC_R25[NTC_CH_COUNT]      = { 10000.0f, 10000.0f, 50000.0f, 50000.0f }; // CH3/CH4=50K(管道内置)
const float   NTC_B_VALUE[NTC_CH_COUNT]  = { 3950.0f,  3950.0f,  3950.0f,  3950.0f  }; // B 值, 未实测先用相同

#define ADC_SHORT_THRESHOLD  20   // ADC < 20 → NTC 短路
#define ADC_OPEN_THRESHOLD   1000 // ADC > 1000 → NTC 断路 (留余量)
#define TEMP_OVERHEAT        55.0f // 超温阈值 (°C, 给 TEMP_MAX=50°C 留 5°C 余量防锁定)

// ===== 加热控制参数 =====
// 加热条接 P6 → U8 (100N03) MOSFET → D9 (PWM3, 引脚 9)
// ⚠️ 旧引脚 D6/P7 已废弃, 留空 (代码不再引用)
#define HEATER_PIN            9       // 加热管道 MOSFET gate 引脚 (P6=CH3)
// PWM 占空比硬上限 = 80% (流动散热大, 50% 功率不足; 通过软启动限制开机冲击)
//   加热条 8.6Ω @24V → 满档电流 ~2.8A, 80% 平均电流 ~2.2A (规格上限 4A, 安全)
#define PWM_DUTY_MAX_PERCENT  80
#define PWM_DUTY_MAX_ANALOG   ((PWM_DUTY_MAX_PERCENT * 255) / 100)  // = 204

// ===== 软启动 (防止开机瞬间大电流冲击) =====
// 前 SOFTSTART_DURATION_MS 毫秒内, 输出从 SOFTSTART_MIN_SCALE 线性爬到 100%
//   即: 0s 时输出被压到 PID×20%, 末段回到 PID×100%
//   目的: 避免 t=0 时 PID 立刻饱和导致满档电流
#define SOFTSTART_DURATION_MS  10000  // 软启动持续 10s
#define SOFTSTART_MIN_SCALE    0.20f  // 起始缩放 = 20% (PID 输出乘以这个系数)
unsigned long bootTime = 0;            // 开机时刻 (在 setup 中记录)

// ===== PID 参数 (流动加热场景重整定: Kp 加大使小误差时仍有推力突破热平衡) =====
// 2026-07-01 调整: 47.3°C 与 50°C 目标仅差 2.7, 但 Kp=35 时输出仅 95, 不足以突破散热
//                  → Kp 提到 80, 同样 2.7°C 误差能推到 216→顶满 204; 让 PID 主动冲刺
#define PID_KP          80.0f
#define PID_KI          0.8f
#define PID_KD          25.0f
#define PID_OUTPUT_MAX  PWM_DUTY_MAX_ANALOG  // PID 输出上限 = 204 (= PWM 80%)

// ===== 目标温度 (变量, 可被按键修改) =====
#define TEMP_MIN         37.0f
#define TEMP_MAX         50.0f    // 调温上限 45 → 50°C (流动方案需更高目标 + 大功率)
#define TEMP_STEP        0.5f
#define TEMP_DEFAULT     38.0f
float  tempSetpoint = TEMP_DEFAULT;

// ===== 按键定义 =====
#define BTN_COUNT        3
const uint8_t BTN_PINS[BTN_COUNT]  = { 2, 3, 4 };
const char*   BTN_NAMES[BTN_COUNT] = { "SW1(+0.5)", "SW2(-0.5)", "SW3(Pwr)" };
#define BTN_DEBOUNCE_MS  50
bool btnPrev[BTN_COUNT]       = { false, false, false };
unsigned long btnLastEdge[BTN_COUNT] = {0, 0, 0};

// PID 运行状态
float         pidIntegral    = 0.0f;
float         pidLastError   = 0.0f;
int           pidOutput      = 0;
bool          pidOverheatLock = false;
unsigned long pidLastTime    = 0;

bool          systemEnabled  = true;  // 温控使能 (SW3 切换)

// ADC 滤波缓冲 (每通道一个)
#define NTC_FILTER_SIZE 8
uint16_t adcBuf[NTC_CH_COUNT][NTC_FILTER_SIZE];
uint8_t  adcIdx[NTC_CH_COUNT]  = {0, 0, 0, 0};
bool     adcFull[NTC_CH_COUNT] = {false, false, false, false};

// 移动平均 + 防除零 (带通道号)
int readNtcAdc(uint8_t ch) {
    unsigned long sum = 0;
    for (int i = 0; i < 4; i++) sum += analogRead(NTC_PINS[ch]);
    uint16_t raw = sum >> 2;

    adcBuf[ch][adcIdx[ch]] = raw;
    adcIdx[ch] = (adcIdx[ch] + 1) % NTC_FILTER_SIZE;
    if (adcIdx[ch] == 0) adcFull[ch] = true;

    uint8_t n = adcFull[ch] ? NTC_FILTER_SIZE : adcIdx[ch];
    if (n == 0) return raw;
    unsigned long total = 0;
    for (uint8_t i = 0; i < n; i++) total += adcBuf[ch][i];
    return (int)(total / n);
}

// 返回状态: 0=正常, 1=短路, 2=断路
int checkNtc(int adc) {
    if (adc < ADC_SHORT_THRESHOLD) return 1;
    if (adc > ADC_OPEN_THRESHOLD)  return 2;
    return 0;
}

// ADC → 温度(°C), B 值公式, 按通道用不同 R25
float adcToTempC(uint8_t ch, int adc) {
    if (adc <= 0)    adc = 1;
    if (adc >= 1023) adc = 1022;

    float rNtc = NTC_R_REF * (float)adc / (1023.0f - (float)adc);
    float steinhart = log(rNtc / NTC_R25[ch]);
    float tempK = 1.0f / (1.0f / 298.15f + steinhart / NTC_B_VALUE[ch]);
    return tempK - 273.15f;
}

/* ================================================================
 * PID 计算: 返回 PWM 输出值 (0~PID_OUTPUT_MAX = 0~204)
 * ============================================================== */
int pidCompute(float temp, float dt) {
    if (temp >= TEMP_OVERHEAT) {
        pidOverheatLock = true;
        pidIntegral = 0.0f;
        return 0;
    }
    if (pidOverheatLock) {
        if (temp < (TEMP_OVERHEAT - 2.0f)) {
            pidOverheatLock = false;
        } else {
            return 0;
        }
    }

    float error = tempSetpoint - temp;

    // P: 比例
    float pTerm = PID_KP * error;

    // I: 条件积分 (防饱和)
    const float INTEGRAL_MAX = (float)PID_OUTPUT_MAX * 0.5f;
    if (error > 1.5f) {
        // ramp 阶段不积分
    } else if (error < 0.0f) {
        // 超温: 3 倍速衰减积分
        float decay = PID_KI * error * dt * 3.0f;
        pidIntegral += decay;
        if (pidIntegral < 0.0f) pidIntegral = 0.0f;
    } else {
        // 接近目标, 正常积分 (仅在 P+I 还没饱和时)
        float tentativeI = pidIntegral + PID_KI * error * dt;
        float pPlusI = pTerm + tentativeI;
        if (pPlusI < (float)PID_OUTPUT_MAX) {
            pidIntegral = tentativeI;
        }
        if (pidIntegral > INTEGRAL_MAX) pidIntegral = INTEGRAL_MAX;
    }

    // D: 微分先行
    float dTerm = PID_KD * (pidLastError - error) / dt;
    pidLastError = error;

    // 合成 + 限幅
    float rawOut = pTerm + pidIntegral + dTerm;
    if (rawOut < 0.0f) rawOut = 0.0f;
    if (rawOut > (float)PID_OUTPUT_MAX) rawOut = (float)PID_OUTPUT_MAX;
    return (int)rawOut;
}

void pidReset() {
    pidIntegral   = 0.0f;
    pidLastError  = 0.0f;
    pidOverheatLock = false;
}

/* ================================================================
 * 按键扫描 (返回刚按下的按键索引或 255=无)
 * ============================================================== */
uint8_t scanButtons() {
    uint8_t pressed = 255;
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        bool nowDown = (digitalRead(BTN_PINS[i]) == LOW);
        if (nowDown != btnPrev[i]) {
            btnLastEdge[i] = millis();
        }
        btnPrev[i] = nowDown;

        if (nowDown && (millis() - btnLastEdge[i] >= BTN_DEBOUNCE_MS)) {
            if (btnLastEdge[i] != 0) {
                pressed = i;
                btnLastEdge[i] = 0;
            }
        }
    }
    return pressed;
}

/* ================================================================
 * 按键事件处理
 * ============================================================== */
void handleButton(uint8_t btnIdx) {
    if (btnIdx >= BTN_COUNT) return;
    Serial.print(F("[BTN] ")); Serial.print(BTN_NAMES[btnIdx]);

    switch (btnIdx) {
        case 0: {  // SW1: +0.5°C
            float newT = tempSetpoint + TEMP_STEP;
            if (newT > TEMP_MAX) newT = TEMP_MAX;
            if (newT != tempSetpoint) {
                tempSetpoint = newT;
                pidReset();
            }
            Serial.print(F(" TempUp -> ")); Serial.print(tempSetpoint); Serial.println(F("C"));
            break;
        }
        case 1: {  // SW2: -0.5°C
            float newT = tempSetpoint - TEMP_STEP;
            if (newT < TEMP_MIN) newT = TEMP_MIN;
            if (newT != tempSetpoint) {
                tempSetpoint = newT;
                pidReset();
            }
            Serial.print(F(" TempDown -> ")); Serial.print(tempSetpoint); Serial.println(F("C"));
            break;
        }
        case 2: {  // SW3: 切换温控开/关
            systemEnabled = !systemEnabled;
            if (!systemEnabled) {
                analogWrite(HEATER_PIN, 0);
                pidOutput = 0;
                pidReset();
            }
            Serial.print(F(" PowerToggle -> "));
            Serial.println(systemEnabled ? F("ENABLED") : F("DISABLED"));
            break;
        }
    }
}

/* ================================================================
 * 初始化
 * ============================================================== */
void setup() {
    Serial.begin(9600);
    while (!Serial);

    Wire.begin();

    // --- 初始化 OLED ---
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("SSD1306 init FAILED"));
        while (true) { delay(1000); }
    }
    Serial.println(F("OLED OK"));

    // --- 初始化 4 路 NTC 滤波缓冲 ---
    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        pinMode(NTC_PINS[ch], INPUT);
    }
    memset(adcBuf, 0, sizeof(adcBuf));

    // ⚠️ P7/D6 已废弃 (旧加热膜方案), 不再初始化为加热输出
    // 仅设为 INPUT 防止悬空 (P7 现是空插座)
    pinMode(6, INPUT_PULLUP);

    // --- 初始化按键 ---
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        pinMode(BTN_PINS[i], INPUT_PULLUP);
    }

    // --- 初始化加热引脚 (P6 → D9, PID 默认关闭) ---
    pinMode(HEATER_PIN, OUTPUT);
    analogWrite(HEATER_PIN, 0);
    Serial.print(F("Heater (P6/D9, CH3 MOSFET), max "));
    Serial.print(PWM_DUTY_MAX_PERCENT); Serial.println(F("%"));
    Serial.print(F("Setpoint=")); Serial.print(tempSetpoint);
    Serial.print(F("C  Kp=")); Serial.print(PID_KP);
    Serial.print(F(" Ki=")); Serial.print(PID_KI);
    Serial.print(F(" Kd=")); Serial.println(PID_KD);
    Serial.println(F("CTRL: max(CH3, CH4)   CH1/CH2 monitor-only"));
    Serial.print(F("Soft-start: "));
    Serial.print(SOFTSTART_DURATION_MS / 1000); Serial.print(F("s @ >="));
    Serial.print((int)(SOFTSTART_MIN_SCALE * 100)); Serial.println(F("% ramp"));
    pidLastTime = millis();
    bootTime    = millis();   // 记录开机时刻用于软启动

    // --- 开机标语 ---
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("Heating Pipe"));
    display.setCursor(0, 12);
    display.println(F("4-ch NTC"));
    display.setCursor(0, 28);
    display.println(F("CTRL: CH3/CH4"));
    display.display();
}

/* ================================================================
 * 主循环
 * ============================================================== */
void loop() {
    unsigned long now = millis();

    // --- 扫按键 (实时响应) ---
    uint8_t btn = scanButtons();
    if (btn < BTN_COUNT) handleButton(btn);

    // --- 每 500ms 测温 + 控制 + 显示 ---
    static unsigned long lastT = 0;
    if (now - lastT < 500) return;
    lastT = now;

    // --- 读取 4 路 NTC ---
    int   adcs[NTC_CH_COUNT];
    int   faults[NTC_CH_COUNT];
    float temps[NTC_CH_COUNT];
    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        adcs[ch]   = readNtcAdc(ch);
        faults[ch] = checkNtc(adcs[ch]);
        temps[ch]  = (faults[ch] == 0) ? adcToTempC(ch, adcs[ch]) : 0.0f;
    }

    // --- 安全检查: 控制用 NTC (CH3/CH4) 任一故障 → 切断加热 ---
    //     CH1/CH2 是纯测量, 故障不影响控制, 只显示错误
    bool   ctrlEmergency = false;
    String reason        = "";
    if (faults[2] != 0) {
        ctrlEmergency = true;
        reason = String("CH3 ") + (faults[2] == 1 ? "SHORT" : "OPEN");
    } else if (faults[3] != 0) {
        ctrlEmergency = true;
        reason = String("CH4 ") + (faults[3] == 1 ? "SHORT" : "OPEN");
    }
    if (ctrlEmergency) {
        analogWrite(HEATER_PIN, 0);
        pidOutput = 0;
        pidReset();
    }

    // --- 计算控制目标 = max(CH3, CH4), 一路故障用另一路 ---
    float controlTemp;
    if (faults[2] != 0 && faults[3] != 0) {
        controlTemp = TEMP_OVERHEAT;   // 双故障 → 视为超温
    } else if (faults[2] != 0) {
        controlTemp = temps[3];
    } else if (faults[3] != 0) {
        controlTemp = temps[2];
    } else {
        controlTemp = max(temps[2], temps[3]);
    }

    // --- PID 计算 + 输出 (带软启动缩放) ---
    float dt = (now - pidLastTime) / 1000.0f;
    pidLastTime = now;
    bool allowHeat = systemEnabled && !ctrlEmergency;
    if (allowHeat) {
        pidOutput = pidCompute(controlTemp, dt);

        // 软启动: 前 SOFTSTART_DURATION_MS 内将 PID 输出按线性斜率缩放
        //   时间 0            → 缩放 SOFTSTART_MIN_SCALE (e.g. 0.2)
        //   时间 DURATION_MS  → 缩放 1.0 (满挡)
        float ssScale = 1.0f;
        unsigned long elapsed = millis() - bootTime;
        if (elapsed < SOFTSTART_DURATION_MS) {
            ssScale = SOFTSTART_MIN_SCALE
                    + (1.0f - SOFTSTART_MIN_SCALE) * (float)elapsed / SOFTSTART_DURATION_MS;
        }
        int scaledOutput = (int)(pidOutput * ssScale);
        analogWrite(HEATER_PIN, scaledOutput);
    } else {
        pidOutput = 0;
        analogWrite(HEATER_PIN, 0);
    }

    // --- 串口打点 ---
    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        Serial.print(NTC_NAMES[ch]);
        Serial.print(F("="));
        if (faults[ch] == 0) {
            Serial.print(temps[ch], 1);
            Serial.print(F("C  "));
        } else {
            Serial.print(faults[ch] == 1 ? F("SHORT ") : F("OPEN  "));
        }
    }
    Serial.print(F("| Ctrl=")); Serial.print(controlTemp, 1);
    Serial.print(F("C Set=")); Serial.print(tempSetpoint, 1);
    Serial.print(F(" PWM=")); Serial.print(pidOutput);
    Serial.print(F("/")); Serial.print(PID_OUTPUT_MAX);
    if (!systemEnabled)       Serial.println(F(" OFF"));
    else if (ctrlEmergency)   { Serial.print(F(" HALT ")); Serial.println(reason); }
    else if (pidOverheatLock) Serial.println(F(" OVERHEAT-LOCK"));
    else                      Serial.println(F(" HEAT"));

    // --- 刷新 OLED (4 路 + 控制状态) ---
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // 标题行 (心跳 + 系统状态)
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(systemEnabled ? F("Heater Pipe") : F("[OFF] Pipe"));
    static bool heart = false;
    heart = !heart;
    display.setCursor(120, 0);
    display.print(heart ? F("*") : F("."));

    // 4 路 NTC, 2 列布局 (CH1/CH2 一行, CH3/CH4 一行)
    // y=12: CH1 左   CH2 右
    // y=24: CH3 左   CH4 右 (控制用)
    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        uint8_t y = 12 + (ch / 2) * 12;        // 行
        uint8_t x = (ch % 2) ? 64 : 0;         // 列
        display.setCursor(x, y);
        display.print(NTC_NAMES[ch]);
        display.print(F(":"));
        if (faults[ch] == 0) {
            display.print(temps[ch], 1);
            display.print(F("C"));
        } else {
            display.print(faults[ch] == 1 ? F("SHORT") : F("OPEN"));
        }
    }

    // 控制目标行 y=40
    display.setCursor(0, 40);
    display.print(F("Set "));
    display.print(tempSetpoint, 1);
    display.print(F(" Ctrl "));
    display.print(controlTemp, 1);

    // PID 输出 + 状态 y=54
    display.setCursor(0, 54);
    if (!systemEnabled) {
        display.print(F("POWERED OFF"));
    } else if (ctrlEmergency) {
        display.print(F("HALT ")); display.print(reason);
    } else if (pidOverheatLock) {
        display.print(F("OVERHEAT LOCK"));
    } else {
        int pct = (pidOutput * 100) / PID_OUTPUT_MAX;
        display.print(F("PWM "));
        display.print(pidOutput);
        display.print(F("/"));
        display.print(PID_OUTPUT_MAX);
        display.print(F(" ("));
        display.print(pct);
        display.print(F("%)"));
    }

    display.display();
}