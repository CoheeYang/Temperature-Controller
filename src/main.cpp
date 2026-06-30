/*
 * main.cpp 鈥?鍔犵儹绠￠亾娓╂帶绯荤粺 (PID + 4 璺祴娓?+ 涓夋寜閿?
 *
 * 2026-06-30 閲嶅ぇ鏂规鍙樻洿:
 *   - 鏃ф柟妗?(PI 鍔犵儹鑶滃寘瑁圭洂姘寸摱) 宸插簾寮? 鍔犵儹閫熷害杩囨參
 *   - 鏂版柟妗? 鐩愭按缁?鍔犵儹绠￠亾" 娴佸姩鍔犵儹, 闅旇啘娉?1~4 L/min
 *   - 鍔犵儹鏉℃帴鍦?P6 (CH3 MOSFET D9), 鍘?P7 (CH4 D6) 宸叉嫈鎺?
 *   - 鍔犵儹鏉″弬鏁? 24V/4A, 1400mm, 8.6惟, 鍐呯疆 2 璺?NTC (50K惟@25掳C)
 *
 * 4 璺?NTC 寮曡剼鏄犲皠:
 *   CH1 = A0 (P3, 10k惟 鍒嗗帇)   鈥?绾祴閲? 涓嶅弬涓庢帶鍒?
 *   CH2 = A1 (P2, 10k惟 鍒嗗帇)   鈥?绾祴閲? 涓嶅弬涓庢帶鍒?
 *   CH3 = A2 (P8, 10k惟 鍒嗗帇)   鈥?鍔犵儹绠￠亾鍏ュ彛 NTC (50K惟) 鈽呭弬涓庢帶鍒?
 *   CH4 = A3 (P9, 10k惟 鍒嗗帇)   鈥?鍔犵儹绠￠亾鍑哄彛 NTC (50K惟) 鈽呭弬涓庢帶鍒?
 *
 * 鎺у埗鐩爣:
 *   - 鎺у埗娓╁害 = max(CH3, CH4), 浠讳竴澶辨晥鐢ㄥ彟涓€璺?
 *   - PID 缁存寔姝ゆ渶澶у€艰揪鍒扮洰鏍囨俯搴?(榛樿 38掳C, 鎸夐敭 37~41掳C 鍙皟)
 *   - PWM 鍗犵┖姣旂‖涓婇檺 = 80% (鍔犵儹鏉￠瀹?4A, 鍏佽鍔犲ぇ鍔熺巼)
 *
 * 涓夋寜閿?(REQUIREMENTS.md 搂2.6):
 *   SW1=D2  鈫? 鐩爣娓╁害 +0.5掳C (涓婇檺 41掳C)
 *   SW2=D3  鈫? 鐩爣娓╁害 -0.5掳C (涓嬮檺 37掳C)
 *   SW3=D4  鈫? 鍒囨崲娓╂帶寮€/鍏?
 *
 * 瀹夊叏淇濇姢:
 *   - 浠讳竴鎺у埗 NTC (CH3/CH4) 鏁呴殰 鈫?鍒囨柇鍔犵儹
 *   - 娓╁害 > 42掳C 鈫?閿佸畾鍔犵儹, 闄嶅埌 40掳C 瑙ｉ攣
 *
 * 涓插彛娉㈢壒鐜? 9600
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

// ===== NTC 鍙傛暟 (鏂板姞鐑潯鍐呯疆 NTC 瑙勬牸: 50K惟@25掳C 卤0.5%) =====
// CH1=A0(P3, R3=10k), CH2=A1(P2, R4=10k), CH3=A2(P8, R14=10k), CH4=A3(P9, R15=10k)
// 鍒嗗帇: VCC - 10k(鏉夸笂) - ADC - NTC - GND
//        娓╁害鈫?鈫?NTC闃诲€尖啌 鈫?ADC鍊尖啌
//
// 鈿狅笍 娉ㄦ剰: 鏂板姞鐑潯鐨?NTC 鏍囩О 50K惟, 浣嗘澘涓婂垎鍘嬬數闃昏繕鏄?10k惟
//   (R3/R4/R14/R15 鍏ㄦ槸 10k, 娌℃硶鍗曠嫭鍖哄垎 P8/P9)
//   25掳C 鏃? ADC = 1023 脳 50/(50+10) 鈮?853 (楂樹綅, 娴嬮噺浠嶅噯纭?
//   楂樻俯鏃? NTC 闄嶅埌鍑?k惟, ADC 姝ｅ父鍦ㄤ腑娈?
//   鍏ㄩ儴閫氶亾鐢ㄧ粺涓€ R_REF=10K(鍒嗗帇鐢甸樆), NTC_R25 鎸?CH 绫诲瀷鍖哄垎
#define NTC_CH_COUNT    4
const uint8_t NTC_PINS[NTC_CH_COUNT] = { A0, A1, A2, A3 };   // P3, P2, P8, P9
const char*   NTC_NAMES[NTC_CH_COUNT] = { "CH1", "CH2", "CH3", "CH4" };

#define NTC_R_REF       10000.0f  // 鏉夸笂鍒嗗帇鐢甸樆 (惟, R3/R4/R14/R15 = 10k惟)

// 閫氶亾 1/2: P3/P2 鑷充粖鐢ㄧ殑浠嶅彲鑳芥槸鏃ф 10K惟 浼犳劅鍣? 閫氶亾 3/4 鏄閬?50K惟
// 杩欓噷鐢ㄦ瘡閫氶亾鐙珛鐨勬爣绉伴樆鍊?
const float   NTC_R25[NTC_CH_COUNT]      = { 10000.0f, 10000.0f, 50000.0f, 50000.0f }; // CH3/CH4=50K(绠￠亾鍐呯疆)
const float   NTC_B_VALUE[NTC_CH_COUNT]  = { 3950.0f,  3950.0f,  3950.0f,  3950.0f  }; // B 鍊? 鏈疄娴嬪厛鐢ㄧ浉鍚?

#define ADC_SHORT_THRESHOLD  20   // ADC < 20 鈫?NTC 鐭矾
#define ADC_OPEN_THRESHOLD   1000 // ADC > 1000 鈫?NTC 鏂矾 (鐣欎綑閲?
#define TEMP_OVERHEAT        42.0f // 瓒呮俯闃堝€?(鈩? 鎺у埗 NTC 娓╁害)

// ===== 鍔犵儹鎺у埗鍙傛暟 =====
// 鍔犵儹鏉℃帴 P6 鈫?U8 (100N03) MOSFET 鈫?D9 (PWM3, 寮曡剼 9)
// 鈿狅笍 鏃у紩鑴?D6/P7 宸插簾寮? 鐣欑┖ (浠ｇ爜涓嶅啀寮曠敤)
#define HEATER_PIN            9       // 鍔犵儹绠￠亾 MOSFET gate 寮曡剼 (P6=CH3)
// PWM 鍗犵┖姣旂‖涓婇檺 = 80% (鍔犵儹鏉￠瀹?24V/4A, 鍏佽鍔犲ぇ鍔熺巼淇濊瘉鍗囨俯閫熷害)
#define PWM_DUTY_MAX_PERCENT  80
#define PWM_DUTY_MAX_ANALOG   ((PWM_DUTY_MAX_PERCENT * 255) / 100)  // = 204

// ===== PID 鍙傛暟 (娌跨敤鏃х増绋冲畾鍙傛暟 Kp=35 Ki=0.8 Kd=25, 鍙﹂渶閲嶆柊鏁村畾) =====
#define PID_KP          35.0f
#define PID_KI          0.8f
#define PID_KD          25.0f
#define PID_OUTPUT_MAX  PWM_DUTY_MAX_ANALOG  // PID 杈撳嚭涓婇檺 = 204 (= 80%)

// ===== 鐩爣娓╁害 (鍙橀噺, 鍙鎸夐敭淇敼) =====
#define TEMP_MIN         37.0f
#define TEMP_MAX         41.0f
#define TEMP_STEP        0.5f
#define TEMP_DEFAULT     38.0f
float  tempSetpoint = TEMP_DEFAULT;

// ===== 鎸夐敭瀹氫箟 =====
#define BTN_COUNT        3
const uint8_t BTN_PINS[BTN_COUNT]  = { 2, 3, 4 };
const char*   BTN_NAMES[BTN_COUNT] = { "SW1(+0.5)", "SW2(-0.5)", "SW3(Pwr)" };
#define BTN_DEBOUNCE_MS  50
bool btnPrev[BTN_COUNT]       = { false, false, false };
unsigned long btnLastEdge[BTN_COUNT] = {0, 0, 0};

// PID 杩愯鐘舵€?
float         pidIntegral    = 0.0f;
float         pidLastError   = 0.0f;
int           pidOutput      = 0;
bool          pidOverheatLock = false;
unsigned long pidLastTime    = 0;

bool          systemEnabled  = true;  // 娓╂帶浣胯兘 (SW3 鍒囨崲)

// ADC 婊ゆ尝缂撳啿 (姣忛€氶亾涓€涓?
#define NTC_FILTER_SIZE 8
uint16_t adcBuf[NTC_CH_COUNT][NTC_FILTER_SIZE];
uint8_t  adcIdx[NTC_CH_COUNT]  = {0, 0, 0, 0};
bool     adcFull[NTC_CH_COUNT] = {false, false, false, false};

// 绉诲姩骞冲潎 + 闃查櫎闆?(甯﹂€氶亾鍙?
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

// 杩斿洖鐘舵€? 0=姝ｅ父, 1=鐭矾, 2=鏂矾
int checkNtc(int adc) {
    if (adc < ADC_SHORT_THRESHOLD) return 1;
    if (adc > ADC_OPEN_THRESHOLD)  return 2;
    return 0;
}

// ADC 鈫?娓╁害(鈩?, B 鍊煎叕寮? 鎸夐€氶亾鐢ㄤ笉鍚?R25
float adcToTempC(uint8_t ch, int adc) {
    if (adc <= 0)    adc = 1;
    if (adc >= 1023) adc = 1022;

    float rNtc = NTC_R_REF * (float)adc / (1023.0f - (float)adc);
    float steinhart = log(rNtc / NTC_R25[ch]);
    float tempK = 1.0f / (1.0f / 298.15f + steinhart / NTC_B_VALUE[ch]);
    return tempK - 273.15f;
}

/* ================================================================
 * PID 璁＄畻: 杩斿洖 PWM 杈撳嚭鍊?(0~204)
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

    // P: 姣斾緥
    float pTerm = PID_KP * error;

    // I: 鏉′欢绉垎 (闃查ケ鍜?
    const float INTEGRAL_MAX = (float)PID_OUTPUT_MAX * 0.5f;
    if (error > 1.5f) {
        // ramp 闃舵涓嶇Н鍒?
    } else if (error < 0.0f) {
        // 瓒呮俯: 3 鍊嶉€熻“鍑忕Н鍒?
        float decay = PID_KI * error * dt * 3.0f;
        pidIntegral += decay;
        if (pidIntegral < 0.0f) pidIntegral = 0.0f;
    } else {
        // 鎺ヨ繎鐩爣, 姝ｅ父绉垎 (浠呭湪 P+I 杩樻病楗卞拰鏃?
        float tentativeI = pidIntegral + PID_KI * error * dt;
        float pPlusI = pTerm + tentativeI;
        if (pPlusI < (float)PID_OUTPUT_MAX) {
            pidIntegral = tentativeI;
        }
        if (pidIntegral > INTEGRAL_MAX) pidIntegral = INTEGRAL_MAX;
    }

    // D: 寰垎鍏堣
    float dTerm = PID_KD * (pidLastError - error) / dt;
    pidLastError = error;

    // 鍚堟垚 + 闄愬箙
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
 * 鎸夐敭鎵弿 (杩斿洖鍒氭寜涓嬬殑鎸夐敭绱㈠紩鎴?255=鏃?
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
 * 鎸夐敭浜嬩欢澶勭悊
 * ============================================================== */
void handleButton(uint8_t btnIdx) {
    if (btnIdx >= BTN_COUNT) return;
    Serial.print(F("[BTN] ")); Serial.print(BTN_NAMES[btnIdx]);

    switch (btnIdx) {
        case 0: {  // SW1: +0.5掳C
            float newT = tempSetpoint + TEMP_STEP;
            if (newT > TEMP_MAX) newT = TEMP_MAX;
            if (newT != tempSetpoint) {
                tempSetpoint = newT;
                pidReset();
            }
            Serial.print(F(" TempUp -> ")); Serial.print(tempSetpoint); Serial.println(F("C"));
            break;
        }
        case 1: {  // SW2: -0.5掳C
            float newT = tempSetpoint - TEMP_STEP;
            if (newT < TEMP_MIN) newT = TEMP_MIN;
            if (newT != tempSetpoint) {
                tempSetpoint = newT;
                pidReset();
            }
            Serial.print(F(" TempDown -> ")); Serial.print(tempSetpoint); Serial.println(F("C"));
            break;
        }
        case 2: {  // SW3: 鍒囨崲娓╂帶寮€/鍏?
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
 * 鍒濆鍖?
 * ============================================================== */
void setup() {
    Serial.begin(9600);
    while (!Serial);

    Wire.begin();

    // --- 鍒濆鍖?OLED ---
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("SSD1306 init FAILED"));
        while (true) { delay(1000); }
    }
    Serial.println(F("OLED OK"));

    // --- 鍒濆鍖?4 璺?NTC 婊ゆ尝缂撳啿 ---
    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        pinMode(NTC_PINS[ch], INPUT);
    }
    memset(adcBuf, 0, sizeof(adcBuf));

    // 鈿狅笍 P7/D6 宸插簾寮?(鏃у姞鐑啘鏂规), 涓嶅啀鍒濆鍖栦负鍔犵儹杈撳嚭
    // 浠呰涓?INPUT 闃叉鎮┖ (P7 鐜版槸绌烘彃搴?
    pinMode(6, INPUT_PULLUP);

    // --- 鍒濆鍖栨寜閿?---
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        pinMode(BTN_PINS[i], INPUT_PULLUP);
    }

    // --- 鍒濆鍖栧姞鐑紩鑴?(P6 鈫?D9, PID 榛樿鍏抽棴) ---
    pinMode(HEATER_PIN, OUTPUT);
    analogWrite(HEATER_PIN, 0);
    Serial.print(F("Heater (P6/D9, CH3 MOSFET), max "));
    Serial.print(PWM_DUTY_MAX_PERCENT); Serial.println(F("%"));
    Serial.print(F("Setpoint=")); Serial.print(tempSetpoint);
    Serial.print(F("C  Kp=")); Serial.print(PID_KP);
    Serial.print(F(" Ki=")); Serial.print(PID_KI);
    Serial.print(F(" Kd=")); Serial.println(PID_KD);
    Serial.println(F("CTRL: max(CH3, CH4)   CH1/CH2 monitor-only"));
    pidLastTime = millis();

    // --- 寮€鏈烘爣璇?---
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
 * 涓诲惊鐜?
 * ============================================================== */
void loop() {
    unsigned long now = millis();

    // --- 鎵寜閿?(瀹炴椂鍝嶅簲) ---
    uint8_t btn = scanButtons();
    if (btn < BTN_COUNT) handleButton(btn);

    // --- 姣?500ms 娴嬫俯 + 鎺у埗 + 鏄剧ず ---
    static unsigned long lastT = 0;
    if (now - lastT < 500) return;
    lastT = now;

    // --- 璇诲彇 4 璺?NTC ---
    int   adcs[NTC_CH_COUNT];
    int   faults[NTC_CH_COUNT];
    float temps[NTC_CH_COUNT];
    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        adcs[ch]   = readNtcAdc(ch);
        faults[ch] = checkNtc(adcs[ch]);
        temps[ch]  = (faults[ch] == 0) ? adcToTempC(ch, adcs[ch]) : 0.0f;
    }

    // --- 瀹夊叏妫€鏌? 鎺у埗鐢?NTC (CH3/CH4) 浠讳竴鏁呴殰 鈫?鍒囨柇鍔犵儹 ---
    //     CH1/CH2 鏄函娴嬮噺, 鏁呴殰涓嶅奖鍝嶆帶鍒? 鍙樉绀洪敊璇?
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

    // --- 璁＄畻鎺у埗鐩爣 = max(CH3, CH4), 涓€璺晠闅滅敤鍙︿竴璺?---
    float controlTemp;
    if (faults[2] != 0 && faults[3] != 0) {
        controlTemp = TEMP_OVERHEAT;   // 鍙屾晠闅?鈫?瑙嗕负瓒呮俯
    } else if (faults[2] != 0) {
        controlTemp = temps[3];
    } else if (faults[3] != 0) {
        controlTemp = temps[2];
    } else {
        controlTemp = max(temps[2], temps[3]);
    }

    // --- PID 璁＄畻 + 杈撳嚭 ---
    float dt = (now - pidLastTime) / 1000.0f;
    pidLastTime = now;
    bool allowHeat = systemEnabled && !ctrlEmergency;
    if (allowHeat) {
        pidOutput = pidCompute(controlTemp, dt);
        analogWrite(HEATER_PIN, pidOutput);
    } else {
        pidOutput = 0;
        analogWrite(HEATER_PIN, 0);
    }

    // --- 涓插彛鎵撶偣 ---
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

    // --- 鍒锋柊 OLED (4 璺?+ 鎺у埗鐘舵€? ---
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // 鏍囬琛?(蹇冭烦 + 绯荤粺鐘舵€?
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(systemEnabled ? F("Heater Pipe") : F("[OFF] Pipe"));
    static bool heart = false;
    heart = !heart;
    display.setCursor(120, 0);
    display.print(heart ? F("*") : F("."));

    // 4 璺?NTC, 2 鍒楀竷灞€ (CH1/CH2 涓€琛? CH3/CH4 涓€琛?
    // y=12: CH1 宸?  CH2 鍙?
    // y=24: CH3 宸?  CH4 鍙?(鎺у埗鐢?
    for (uint8_t ch = 0; ch < NTC_CH_COUNT; ch++) {
        uint8_t y = 12 + (ch / 2) * 12;        // 琛?
        uint8_t x = (ch % 2) ? 64 : 0;         // 鍒?
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

    // 鎺у埗鐩爣琛?y=40
    display.setCursor(0, 40);
    display.print(F("Set "));
    display.print(tempSetpoint, 1);
    display.print(F(" Ctrl "));
    display.print(controlTemp, 1);

    // PID 杈撳嚭 + 鐘舵€?y=54
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
