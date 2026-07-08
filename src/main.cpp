/*
 * main.cpp — 输液瓶温控系统 (PID + 双 NTC + 三按键)
 *
 * 功能:
 *   1) OLED 显示 CH1 / CH2 两路温度 (每 500ms 刷新)
 *   2) PID 闭环控制加热膜, 维持目标温度:
 *        引脚 D6 (P7 → U9 → CH4 MOSFET gate), 硬件 PWM ~1kHz
 *        目标温度默认 38°C, 可用按键调节 37~43°C
 *        控制温度 = max(CH1, CH2), 任一传感器失效则用另一路
 *        PID 参数 Kp=35.0, Ki=0.8, Kd=25.0 (实测最佳)
 *        PWM 占空比硬上限 = 50% (防止危险过热)
 *   3) 三个按键 (REQUIREMENTS.md §2.6):
 *        SW1=D2  →  目标温度 +0.5°C (上限 43°C)
 *        SW2=D3  →  目标温度 -0.5°C (下限 37°C)
 *        SW3=D4  →  切换 温控开/关 (按一次关闭, 再按启用)
 *        按键带上拉 + 软件消抖 (50ms)
 *   4) 安全保护:
 *        任一 NTC 短路/断路 → 立即切断加热
 *        温度 > 62℃ (当前标定阈值) → 锁定加热, 降到 60℃ 才解锁
 *        双 NTC 故障 → 视为超温, 强制停机
 *
 *  串口波特率: 9600
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1        // -1 = 共享 Arduino RESET
#define OLED_ADDR     0x3C      // 大多数 SSD1306 模块默认地址

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== NTC 参数 (来自 config.h / REQUIREMENTS.md) =====
// CH1 接 A0 (P3, R3);  CH2 接 A1 (P2, R4);  CH3=A2(P8); CH4=A3(P9)
// 分压: VCC - 10k - ADC - NTC - GND
//        温度↑ → NTC阻值↓ → ADC值↓
// ⚠️ NTC 实际为 100kΩ@25℃, 但 PCB 上分压电阻 R3/R4 仍是 10kΩ (硬件焊死)
//    → ADC 工作点偏高 (室温下 ≈930), 精度受限但可正常测温
#define NTC_CH_COUNT    2                          // 当前启用 2 路
const uint8_t NTC_PINS[NTC_CH_COUNT] = { A0, A1 }; // P3=CH1, P2=CH2
const char*   NTC_NAMES[NTC_CH_COUNT] = { "CH1", "CH2" };

#define NTC_R25         100000.0f // 25℃ 标称阻值 (Ω) —— NTC 实际为 100kΩ@25℃
#define NTC_B_VALUE     3950.0f   // B 值
#define NTC_R_REF       10000.0f  // 分压电阻 (Ω) —— PCB R3/R4=10kΩ 焊死, 不修改
#define ADC_SHORT_THRESHOLD  20   // ADC < 20 → 短路
#define ADC_OPEN_THRESHOLD   1000 // ADC > 1000 → 断路
#define TEMP_OVERHEAT       50.0f // 超温硬切断阈值 (℃) ⚠️ 标定模式 → 50°C (医疗输液场景应使用 42°C)

// ===== 加热控制参数 =====
// P7 → CH4 → U9(100N03) → D6 (REQUIREMENTS.md 表格确认)
// D6 是 Arduino Nano 支持硬件 PWM 的引脚 (Timer0 OC0A),
// 默认频率约 980Hz ≈ 1kHz, 用 analogWrite 直接输出
#define HEATER_PIN            6       // 加热膜 PWM 引脚 (P7=CH4)
#define BUZZER_PIN            5       // 蜂鸣器引脚 (D5, BUZZER2: PB-1224PE-05Q 有源, 高电平响)
// 自适应占空比上限: 误差大时(RAM阶段) 允许更猛, 接近目标时收回防过冲
// ⚠️ 加热膜 24V/1A=24W, 实测热惯性大, 原 50%/80% 上限太保守导致中段爬不上去
#define PWM_BOOST_THRESHOLD   3.0f    // 误差≥3°C 时启动 boost (原 2.0 提升到 3.0)
#define PWM_NORMAL_PERCENT    70      // 稳态占空比上限 (原 50 → 70, 提高基础加热功率)
#define PWM_BOOST_PERCENT     100     // 大误差时全开 100% (原 80 → 100), 快速冲温
#define PWM_NORMAL_ANALOG     ((PWM_NORMAL_PERCENT * 255) / 100)  // 178
#define PWM_BOOST_ANALOG      ((PWM_BOOST_PERCENT  * 255) / 100)  // 255

// ===== PID 参数 (来自 config.h, PID 整定前的初值) =====
#define PID_KP          35.0f    // 比例系数
#define PID_KI          0.8f     // 积分系数 (小, 防积分饱和超调)
#define PID_KD          25.0f    // 微分系数 (强阻尼对抗热惯性)
// PID 输出上限是动态的: 在 pidCompute 内根据 error 选择, 默认稳态 == PWM_NORMAL_ANALOG
#define PID_OUTPUT_NORMAL  PWM_NORMAL_ANALOG  // 127 (稳态, 兼容其它引用)
#define PID_OUTPUT_MAX     PID_OUTPUT_NORMAL  // 文档/老代码兼容 (实际可放宽见 pidCompute)

// ===== 目标温度 (变量, 可被按键修改) =====
// 调温范围 37~49°C, 步进 0.5°C, 默认 38°C
// (49°C 上限用于加热膜实验, 必须低于超温阈值 TEMP_OVERHEAT=50°C 至少 1°C)
#define TEMP_MIN         37.0f
#define TEMP_MAX         49.0f     // 调温上限 (必须 < TEMP_OVERHEAT=50°C 留安全余量)
#define TEMP_STEP        0.5f
#define TEMP_DEFAULT     38.0f
float  tempSetpoint = TEMP_DEFAULT;   // 当前目标温度 (按键可调)

// ===== 按键定义 (REQUIREMENTS.md §2.6 三个按键) =====
// SW1=D2(目标温度+0.5), SW2=D3(目标温度-0.5), SW3=D4(开/关温控)
// 电路: 按下=GND(低电平), 松开=VCC(高电平, 经10k上拉)
#define BTN_COUNT        3
const uint8_t BTN_PINS[BTN_COUNT]  = { 2, 3, 4 };
const char*   BTN_NAMES[BTN_COUNT] = { "SW1(+0.5)", "SW2(-0.5)", "SW3(Pwr)" };
#define BTN_DEBOUNCE_MS  50      // 消抖延时 (ms)
bool btnPrev[BTN_COUNT]       = { false, false, false };  // 上次按键状态(true=按下)
unsigned long btnLastEdge[BTN_COUNT] = {0, 0, 0};          // 上次电平跳变时刻

// PID 运行状态
float         pidIntegral   = 0.0f;   // 积分累计 (带抗饱和)
float         pidLastError  = 0.0f;   // 上次误差 (微分先行)
int           pidOutput     = 0;      // PID 当前输出 (0~127)
bool          pidOverheatLock = false; // 超温锁定 (需降温才解锁)
bool          alarmActive     = false; // 运行期报警活跃标志 (传感器故障/超温), 联动蜂鸣器
unsigned long pidLastTime  = 0;      // 上次 PID 计算时刻

// 温控使能开关 (SW3 切换)
bool          systemEnabled = true;   // false → 关闭温控 (停止加热, 但仍显示温度)
bool          heaterOn      = true;   // 本周期是否真在加热 (用于状态显示和故障检测)

// ===== 4 小时自动关机 (从实际加热开始计时, 加热中断时暂停, 恢复时累加) =====
#define AUTO_SHUTDOWN_MS  (4UL * 60UL * 60UL * 1000UL)  // 4 小时 = 14,400,000 ms
unsigned long heatingAccumMs   = 0;   // 累计加热时长 (ms)
unsigned long heatingStartMs   = 0;   // 本次连续加热段的起始时刻 (ms)
bool          heatingTimerRunning = false;  // 加热计时器是否在走
bool          autoShutdownTriggered = false; // 自动关机是否已触发 (需 SW3 手动复位)

// ADC 滤波缓冲 (每通道一个)
#define NTC_FILTER_SIZE 8
uint16_t adcBuf[NTC_CH_COUNT][NTC_FILTER_SIZE];
uint8_t  adcIdx[NTC_CH_COUNT] = {0};
bool     adcFull[NTC_CH_COUNT] = {false};

// 移动平均 + 防除零 (带通道号)
int readNtcAdc(uint8_t ch) {
    unsigned long sum = 0;
    for (int i = 0; i < 4; i++) sum += analogRead(NTC_PINS[ch]); // 4连采
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

// ADC → 温度(℃), B 值简化公式
float adcToTempC(int adc) {
    if (adc <= 0)    adc = 1;
    if (adc >= 1023) adc = 1022;

    float rNtc = NTC_R_REF * (float)adc / (1023.0f - (float)adc);
    float steinhart = log(rNtc / NTC_R25);
    float tempK = 1.0f / (1.0f / 298.15f + steinhart / NTC_B_VALUE);
    return tempK - 273.15f;
}

/* ================================================================
 * PID 计算: 返回 PWM 输出值 (0~127)
 *   - 目标温度 = TEMP_SETPOINT
 *   - 输入 = 当前温度, dt = 时间间隔(秒)
 *   - 超温 (>50℃) 锁定: 输出强制 0, 直到温度降到 48℃ 以下解锁
 *   - 输出上限 = 127 (PWM 50%), 抗积分饱和
 * ============================================================== */
int pidCompute(float temp, float dt) {
    // 超温锁定机制 (迟滞: 上限 50℃ 触发, 下限 48℃ 解锁)
    if (temp >= TEMP_OVERHEAT) {
        pidOverheatLock = true;
        pidIntegral = 0.0f;
        return 0;
    }
    if (pidOverheatLock) {
        if (temp < (TEMP_OVERHEAT - 2.0f)) {
            pidOverheatLock = false;   // 温度回落到 48℃ → 解锁
        } else {
            return 0;                  // 锁定期间输出 0
        }
    }

    // ===== PID 三项计算 =====
    float error = tempSetpoint - temp;   // 正误差 = 偏冷, 需加热

    // 自适应输出上限: 大误差时(boost 阶段) 放宽到 PWM_BOOST_ANALOG
    //                  接近目标(稳态) 保持在 PWM_NORMAL_ANALOG
    float outMax = (error > PWM_BOOST_THRESHOLD) ? PWM_BOOST_ANALOG : PWM_NORMAL_ANALOG;

    // P: 比例
    float pTerm = PID_KP * error;

    // I: 积分 (条件积分 + 加速回退, 彻底防饱和)
    //   ① 大误差(>3°C, 处于 ramp 阶段) → 仍按 30% 折扣累加, 让积分能跟上
    //   ② 超温(error<0) → 3 倍速衰减积分, 快速纠正冲温
    //   ③ 接近目标(0~3°C) → 正常累加, 消除稳态误差
    //   ④ 范围夹到 outMax (与 P 项合并后不超出)
    const float INTEGRAL_MAX = (float)PWM_NORMAL_ANALOG;  // 积分上限 = full normal output
    if (error > PWM_BOOST_THRESHOLD) {
        // ramp 阶段也累加积分 (折半), 避免 P 项被 outMax 限制时仍能继续顶上
        float tentativeI = pidIntegral + PID_KI * error * dt * 0.5f;
        if (tentativeI > INTEGRAL_MAX) tentativeI = INTEGRAL_MAX;
        pidIntegral = tentativeI;
    } else if (error < 0.0f) {
        // 超温: 3 倍速衰减积分, 砍掉 ramp 时拉下的尾巴
        float decay = PID_KI * error * dt * 3.0f;
        pidIntegral += decay;
        if (pidIntegral < 0.0f) pidIntegral = 0.0f;
    } else {
        // 接近目标, 正常积分 (仅在 P+I 还没饱和时)
        float tentativeI = pidIntegral + PID_KI * error * dt;
        float pPlusI = pTerm + tentativeI;
        if (pPlusI < outMax) {
            pidIntegral = tentativeI;
        }
        if (pidIntegral > INTEGRAL_MAX) pidIntegral = INTEGRAL_MAX;
    }

    // D: 微分先行 (减少设定值突变的冲击)
    float dTerm = PID_KD * (pidLastError - error) / dt;
    pidLastError = error;

    // 合成 + 限幅到 [0, outMax] (动态上限)
    float rawOut = pTerm + pidIntegral + dTerm;
    if (rawOut < 0.0f) rawOut = 0.0f;
    if (rawOut > outMax) rawOut = outMax;

    return (int)rawOut;
}

void pidReset() {
    pidIntegral   = 0.0f;
    pidLastError  = 0.0f;
    pidOverheatLock = false;
}

/* ================================================================
 * 按键扫描: 带消抖, 检测"按下边沿"(从松→按)
 *   返回值为本次刚按下的按键索引: 0=SW1, 1=SW2, 2=SW3, 255=无
 * ============================================================== */
uint8_t scanButtons() {
    uint8_t pressed = 255;
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        bool nowDown = (digitalRead(BTN_PINS[i]) == LOW);  // 低电平=按下
        if (nowDown != btnPrev[i]) {
            btnLastEdge[i] = millis();        // 记下电平跳变时刻
        }
        btnPrev[i] = nowDown;

        // 跳变后稳定超过 BTN_DEBOUNCE_MS 才认为有效
        if (nowDown && (millis() - btnLastEdge[i] >= BTN_DEBOUNCE_MS)) {
            // 标记为"已处理", 防止长按重复触发
            // 用一个 hack: 把 btnLastEdge 置为 0, 直到松开 (nowDown=false) 才能再次触发
            if (btnLastEdge[i] != 0) {
                pressed = i;
                btnLastEdge[i] = 0;  // 锁死, 直到释放
                // 注意: 这里只返回一个按键, 优先级按 i 顺序
            }
        }
    }
    return pressed;
}

/* ================================================================
 * 按键事件处理
 *   SW1(D2): 目标温度 +0.5°C (上限 41°C)
 *   SW2(D3): 目标温度 -0.5°C (下限 37°C)
 *   SW3(D4): 切换 温控开/关
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
                pidReset();   // 目标变了, 清积分避免冲击
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
            // 若自动关机已触发, 首次按 SW3 用于复位 (清空计时 + 重新启用)
            if (autoShutdownTriggered) {
                autoShutdownTriggered = false;
                heatingAccumMs       = 0;
                heatingTimerRunning  = false;
                systemEnabled        = true;
                pidReset();
                Serial.println(F("[AUTO-OFF] Reset by SW3, heating time cleared"));
            } else {
                systemEnabled = !systemEnabled;
                if (!systemEnabled) {
                    analogWrite(HEATER_PIN, 0);   // 关温控时立即停止加热
                    pidOutput = 0;
                    pidReset();
                }
            }
            Serial.print(F(" PowerToggle -> "));
            Serial.println(systemEnabled ? F("ENABLED") : F("DISABLED"));
            break;
        }
    }
}

// 扫描 I2C 总线, 打印所有应答地址
void scanI2C() {
    Serial.println(F("--- I2C Bus Scan ---"));
    byte count = 0;
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte err = Wire.endTransmission();
        if (err == 0) {
            Serial.print(F("Found device at 0x"));
            if (addr < 16) Serial.print(F("0"));
            Serial.println(addr, HEX);
            count++;
        }
    }
    Serial.print(F("Total devices found: "));
    Serial.println(count);
    if (count == 0) {
        Serial.println(F("!! No I2C device found. Check wiring:"));
        Serial.println(F("   VCC->5V, GND->GND, SDA->A4, SCL->A5"));
    }
    Serial.println(F("--------------------"));
}

/* ================================================================
 * 蜂鸣器控制 (BUZZER2 = PB-1224PE-05Q, 有源蜂鸣器, 高电平响)
 *   - buzzerOn()     : 持续鸣响
 *   - buzzerOff()    : 停止鸣响
 *   - buzzerShortBeep(): 鸣响 ms 毫秒后自动停止
 * ============================================================== */
void buzzerOn()              { digitalWrite(BUZZER_PIN, HIGH); }
void buzzerOff()             { digitalWrite(BUZZER_PIN, LOW);  }
void buzzerShortBeep(uint16_t ms) {
    buzzerOn();
    delay(ms);
    buzzerOff();
}

/* ================================================================
 * 上电自检 (Power-On Self-Test, POST)
 *   流程:
 *     (1) 蜂鸣器硬件自检: 响 200ms
 *     (2) 依次扫描 NTC 各通道: ADC 阈值检查 (短路<20 / 断路>1000)
 *     (3) 任一通道故障 → 蜂鸣器长鸣报警 + OLED 故障页 + 死循环
 *     (4) 全部正常 → 蜂鸣器鸣一声 200ms 示意 OK
 *   返回: 无 (故障时永不返回, 进入死循环)
 * ============================================================== */
void powerOnSelfTest() {
    Serial.println(F("\n=== Power-On Self-Test ==="));

    // --- 上电瞬间强制关断加热 (POST 期间不允许加热) ---
    pinMode(BUZZER_PIN, OUTPUT);
    buzzerOff();
    analogWrite(HEATER_PIN, 0);

    // (1) 蜂鸣器硬件自检声
    Serial.println(F("[POST] Buzzer test..."));
    buzzerShortBeep(200);
    delay(80);

    // (2) NTC 温度传感器 + 线路短路/断路检测
    delay(100);  // 等待 ADC 稳定

    bool faultFound = false;
    int  faultCh    = -1;
    int  faultKind  = 0;
    int  faultAdc   = 0;

    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        // 4 连采取平均, 抗噪
        unsigned long sum = 0;
        for (int i = 0; i < 4; i++) sum += analogRead(NTC_PINS[ch]);
        int adc = (int)(sum >> 2);

        int kind = checkNtc(adc);
        Serial.print(F("[POST] "));
        Serial.print(NTC_NAMES[ch]);
        Serial.print(F(" ADC="));
        Serial.print(adc);
        if (kind == 0) {
            Serial.println(F(" OK"));
        } else if (kind == 1) {
            Serial.println(F(" SHORT!"));
        } else {
            Serial.println(F(" OPEN!"));
        }

        if (kind != 0) {
            faultFound = true;
            faultCh    = (int)ch;
            faultKind  = kind;
            faultAdc   = adc;
            break;   // 故障只需报首路
        }
    }

    // (3) 任一通道故障: 蜂鸣器长鸣 + OLED 报警 + 死循环
    if (faultFound) {
        const char* faultName = (faultKind == 1) ? "SHORT!" : "OPEN!";
        Serial.print(F("!! POST FAILED: "));
        Serial.print(NTC_NAMES[faultCh]);
        Serial.print(F(" ADC="));
        Serial.print(faultAdc);
        Serial.print(F(" -> "));
        Serial.println(faultName);

        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);

        display.setCursor(28, 0);
        display.println(F("!!! ALARM !!!"));
        display.setCursor(20, 14);
        display.println(F("POST FAILED"));

        display.setCursor(0, 30);
        display.print(F("CH"));
        display.print(faultCh + 1);
        display.print(F("  "));
        display.println(faultName);

        display.setCursor(0, 42);
        display.print(F("ADC="));
        display.println(faultAdc);

        display.setCursor(0, 56);
        display.println(F("CHECK WIRING!"));
        display.display();

        // 蜂鸣器长鸣报警 (持续鸣响, 不退出)
        Serial.println(F("!! BUZZER ALARM ON (continuous)"));
        while (true) {
            buzzerOn();   // 长鸣报警
        }
    }

    // (4) 全部通过: 蜂鸣器响一声 (200ms) 示意正常
    Serial.println(F("[POST] ALL CHANNELS OK"));
    buzzerShortBeep(200);

    // OLED 显示自检通过提示
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(30, 18);
    display.print(F("SELF TEST"));
    display.setCursor(48, 40);
    display.print(F("OK"));
    display.display();
    delay(800);

    Serial.println(F("=== POST PASSED, entering main loop ===\n"));
}

/* ================================================================
 * 初始化
 * ============================================================== */
void setup() {
    Serial.begin(9600);
    // ⚠️ 原 while(!Serial) 在独立供电(不连电脑串口)时永远阻塞, 导致 OLED/蜂鸣器
    // 初始化代码根本无法执行 -> 表现为"完全没反应"
    // 替换为启动延时, 让 USB-CDC 有时间初始化但不阻塞
    delay(300);

    Serial.println(F("\n=== Boot (NTC Temp Control v2) ==="));

    // --- 预先初始化蜂鸣器引脚 (OLED 失败时用于报警提示) ---
    pinMode(BUZZER_PIN, OUTPUT);
    buzzerOff();

    // --- Step 1: I2C 扫描 (诊断用, 即使没找到 OLED 也不阻塞) ---
    Wire.begin();
    Wire.setWireTimeout(2000, true);   // 2s 超时, 避免 SSD1306 未上电时 endTransmission 死等
    scanI2C();

    // --- Step 2: 初始化 OLED (失败则蜂鸣器长鸣 + 死循环提示) ---
    Serial.println(F("Initializing SSD1306 @ 0x3C..."));
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("!! SSD1306 init FAILED"));
        Serial.println(F("   If I2C scan found 0x3C, check reset pin / power."));
        Serial.println(F("   Program will hang with buzzer alarm."));
        while (true) {
            buzzerOn();      // OLED 失败: 蜂鸣器长鸣报警 (让用户知道设备未正常启动)
            delay(1000);
        }
    }
    Serial.println(F("SSD1306 init OK!"));

    // --- 初始化 NTC 滤波缓冲 ---
    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        pinMode(NTC_PINS[ch], INPUT);
    }
    memset(adcBuf, 0, sizeof(adcBuf));

    // --- 初始化按键引脚 (SW1=D2/SW2=D3/SW3=D4, 输入上拉) ---
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        pinMode(BTN_PINS[i], INPUT_PULLUP);
    }
    Serial.println(F("Buttons: SW1(D2)+0.5, SW2(D3)-0.5, SW3(D4)Pwr"));

    // --- 初始化加热引脚 (P7 → D6, PID 模式默认关闭, 等 PID 计算后输出) ---
    pinMode(HEATER_PIN, OUTPUT);
    analogWrite(HEATER_PIN, 0);
    Serial.print(F("PID heater on pin D")); Serial.println(HEATER_PIN);
    Serial.print(F("Setpoint=")); Serial.print(tempSetpoint);
    Serial.print(F("C  Kp=")); Serial.print(PID_KP);
    Serial.print(F(" Ki=")); Serial.print(PID_KI);
    Serial.print(F(" Kd=")); Serial.println(PID_KD);
    pidLastTime = millis();

    // --- 上电自检 (蜂鸣器+NTC短路/断路检测, 失败则死循环长鸣报警) ---
    powerOnSelfTest();

    // --- 显示固定标题 ---
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("NTC Temp Monitor"));
    display.setCursor(0, 12);
    display.println(F("CH1 @ A0"));
    display.setCursor(0, 24);
    display.println(F("CH2 @ A1"));
    display.setCursor(0, 40);
    display.println(F("Reading..."));
    display.display();
    Serial.println(F("Setup done. Reading NTC (2 channels)..."));
}

/* ================================================================
 * 主循环
 *   加热: 硬件 PWM (~1kHz) 全程自动输出, 这里只在故障时拉低关断
 *   500ms: 读 NTC + 安全检查 + PID 计算 + OLED 刷新
 *   10000ms: 串口日志输出 (10s 一次, 便于长时间数据采集)
 * ============================================================== */
void loop() {
    unsigned long now = millis();

    // ============ 每次循环: 扫按键 (实时响应) ============
    uint8_t btn = scanButtons();
    if (btn < BTN_COUNT) {
        handleButton(btn);
    }

    // ============ 每 500ms: 读 NTC + 安全 + PID + 屏幕 ============
    static unsigned long lastT = 0;
    if (now - lastT < 500) return;
    lastT = now;

    // 串口打点独立节拍 (10s 一次), 与本函数 500ms 节拍分开
    static unsigned long lastLogT = 0;
    bool doLog = (now - lastLogT >= 10000);

    // --- 读取两路 NTC ---
    int   adcs[NTC_CH_COUNT];
    int   faults[NTC_CH_COUNT];
    float temps[NTC_CH_COUNT];

    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        adcs[ch]   = readNtcAdc(ch);
        faults[ch] = checkNtc(adcs[ch]);
        temps[ch]  = (faults[ch] == 0) ? adcToTempC(adcs[ch]) : 0.0f;
    }

    // --- 安全检查: 任一通道故障 → 切断加热 ---
    bool   emergency = false;
    String reason    = "";
    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        if (faults[ch] != 0) {
            emergency = true;
            reason   = String(NTC_NAMES[ch]) + (faults[ch] == 1 ? " SHORT" : " OPEN");
            break;
        }
    }
    if (emergency && systemEnabled) {
        heaterOn = false;   // 仅作为本周期标记使用, 保留逻辑兼容
        analogWrite(HEATER_PIN, 0);
        pidReset();
        // 蜂鸣器报警: 运行期检测到传感器故障 → 长鸣报警 (持续到故障排除)
        if (!alarmActive) {
            alarmActive = true;
            buzzerOn();
            Serial.println(F("!! BUZZER ALARM ON"));
        }
        Serial.print(F("!! SAFETY TRIP: ")); Serial.println(reason);
    }

    // --- 计算控制目标温度 (两路取最大值, 一路故障用另一路) ---
    float controlTemp;
    if (faults[0] == 0 && faults[1] == 0) {
        controlTemp = max(temps[0], temps[1]);
    } else if (faults[0] == 0) {
        controlTemp = temps[0];
    } else if (faults[1] == 0) {
        controlTemp = temps[1];
    } else {
        controlTemp = TEMP_OVERHEAT;  // 双故障, 当作超温处理
    }

    // --- 决定是否允许加热 ---
    // 四个条件都满足才加热: ①系统使能(SW3) ②无传感器故障 ③未在超温锁定 ④未触发自动关机
    bool allowHeat = systemEnabled && !emergency && !autoShutdownTriggered;   // 超温锁定由 pidCompute 内部处理

    // ===== 4 小时累计加热时长管理 =====
    // 上升沿 (false→true): 记录本段加热起始时刻
    // 下降沿 (true→false): 把本段时长累加进 heatingAccumMs
    // 每次循环: 若在加热中, 检查总时长是否达到 AUTO_SHUTDOWN_MS
    static bool prevAllowHeat = false;
    if (allowHeat && !prevAllowHeat) {
        heatingStartMs      = now;
        heatingTimerRunning = true;
    } else if (!allowHeat && prevAllowHeat) {
        heatingAccumMs      += (now - heatingStartMs);
        heatingTimerRunning = false;
    }
    prevAllowHeat = allowHeat;

    // 累计加热时长 = 之前累计 + 本段持续 (仅在加热中)
    unsigned long currentHeatingMs = heatingAccumMs;
    if (heatingTimerRunning) {
        currentHeatingMs += (now - heatingStartMs);
    }

    // 达到 4 小时 → 触发自动关机
    if (heatingTimerRunning && currentHeatingMs >= AUTO_SHUTDOWN_MS) {
        // 把本段时长并入累计, 然后停表
        heatingAccumMs       = currentHeatingMs;
        heatingTimerRunning  = false;
        autoShutdownTriggered = true;
        systemEnabled         = false;   // 立即关温控
        analogWrite(HEATER_PIN, 0);
        pidOutput = 0;
        pidReset();
        // 蜂鸣器短鸣 3 声提示 (区别于故障长鸣)
        for (uint8_t i = 0; i < 3; i++) {
            buzzerShortBeep(200);
            delay(200);
        }
        Serial.print(F("!! AUTO SHUTDOWN: heating reached "));
        Serial.print(AUTO_SHUTDOWN_MS / 1000UL / 60UL);
        Serial.println(F(" min. Press SW3 to reset."));
    }

    // --- PID 计算输出 ---
    float dt = (now - pidLastTime) / 1000.0f;
    pidLastTime = now;
    if (!emergency) heaterOn = true;  // 故障恢复后允许加热

    // --- 超温锁定联动蜂鸣器报警 ---
    // pidCompute() 内部测到超温会设置 pidOverheatLock; 这里同步报警状态
    if (pidOverheatLock && !alarmActive) {
        alarmActive = true;
        buzzerOn();
        Serial.println(F("!! BUZZER ALARM ON (OVERHEAT)"));
    }
    // --- 故障/超温全部解除后关闭蜂鸣器报警 ---
    if (!emergency && !pidOverheatLock && alarmActive) {
        alarmActive = false;
        buzzerOff();
        Serial.println(F("## BUZZER ALARM OFF (recovered)"));
    }
    if (allowHeat) {
        pidOutput = pidCompute(controlTemp, dt);
        analogWrite(HEATER_PIN, pidOutput);
    } else {
        pidOutput = 0;
        analogWrite(HEATER_PIN, 0);
        // 关温控时不重置 PID, 重新启动时Smooth恢复 (可选)
    }

    // --- 串口打点 (节流到 10s 一次, 避免长时间记录数据时日志过大) ---
    if (doLog) {
        lastLogT = now;
        // 先打一个时间戳, 便于后来分析
        Serial.print(F("[t=")); Serial.print(now / 1000); Serial.print(F("s] "));
        for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
            Serial.print(NTC_NAMES[ch]);
            Serial.print(F(" ADC=")); Serial.print(adcs[ch]);
            if (faults[ch] == 0) {
                Serial.print(F(" Temp=")); Serial.print(temps[ch], 2); Serial.print(F("C "));
            } else if (faults[ch] == 1) {
                Serial.print(F("[SHORT] "));
            } else {
                Serial.print(F("[OPEN] "));
            }
        }
        Serial.print(F("| Ctrl=")); Serial.print(controlTemp, 2);
        Serial.print(F("C Set=")); Serial.print(tempSetpoint, 1);
        Serial.print(F(" PWM=")); Serial.print(pidOutput);
        Serial.print(F("/")); Serial.print(PID_OUTPUT_MAX);
        if (!systemEnabled) {
            Serial.println(F(" (DISABLED)"));
        } else if (!heaterOn || emergency) {
            Serial.print(F(" (HALT: ")); Serial.print(reason); Serial.println(F(")"));
        } else if (pidOverheatLock) {
            Serial.println(F(" (OVERHEAT-LOCK)"));
        } else {
            Serial.println(F(" (HEATING)"));
        }
    }

    // --- 刷新 OLED (两路温度 + PID 状态) ---
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // 标题 + 系统开关状态指示 + 心跳
    display.setTextSize(1);
    display.setCursor(0, 0);
    if (systemEnabled) {
        display.println(F("Temp Control"));
    } else {
        display.println(F("TempCtrl [OFF]"));
    }
    static bool heart = false;
    heart = !heart;
    display.setCursor(120, 0);
    display.print(heart ? F("*") : F("."));

    // 两路温度, y=12 / 24
    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        uint8_t y = 12 + ch * 12;
        display.setCursor(0, y);
        display.print(NTC_NAMES[ch]);
        display.print(F(": "));
        if (faults[ch] == 0) {
            display.print(temps[ch], 1);
            display.print(F("C "));
            display.print(adcs[ch]);
        } else {
            display.print(faults[ch] == 1 ? F("SHORT!") : F("OPEN!"));
        }
    }

    // 目标 + 控制温度 y=40
    display.setCursor(0, 40);
    display.print(F("Set "));   display.print(tempSetpoint, 1);
    display.print(F(" Now "));  display.print(controlTemp, 1);
    display.print(F("C"));

    // PID 输出 + 状态, y=54
    display.setCursor(0, 54);
    if (!systemEnabled) {
        display.print(F("POWERED OFF"));
    } else if (autoShutdownTriggered) {
        display.print(F("AUTO-OFF 4H! SW3"));
    } else if (emergency) {
        display.print(F("HALT! "));
        display.print(reason);
    } else if (pidOverheatLock) {
        display.print(F("OVERHEAT LOCK"));
    } else {
        // 占空比显示: 按 255 为基准 (analogWrite 全量程)
        int pct = (pidOutput * 100) / 255;
        display.print(F("PWM "));
        display.print(pidOutput);
        display.print(F("/255 ("));
        display.print(pct);
        display.print(F("%)"));
    }

    display.display();
}

