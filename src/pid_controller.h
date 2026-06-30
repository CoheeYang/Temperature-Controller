/*
 * pid_controller.h — 单通道 PID 控制器
 *
 * 单向加热 (无制冷) — 输出钳位 [0, 255]
 * 带积分抗饱和 + 超温硬切断 + 软启动
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "config.h"

class PIDController {
public:
    PIDController();

    // 设置 PID 参数
    void setTunings(float kp, float ki, float kd);
    void setSetpoint(float sp);

    // 核心: 输入当前温度 & 时间增量(s), 返回 PWM [0,255]
    // 安全层叠加在内部
    float compute(float temperature, float dt);

    // 重置积分器 (切换通道 / 故障恢复后调用)
    void reset();

    // 查询
    float getOutput()    const { return _output; }
    float getSetpoint()  const { return _setpoint; }
    float getIntegral()  const { return _integral; }
    bool  isOverheat()   const { return _overheat; }

    // 软启动计时 (需在 loop 中持续调用)
    void  startSoftStart();
    bool  isInSoftStart();

private:
    float _kp, _ki, _kd;
    float _setpoint;
    float _integral;
    float _lastError;
    float _output;
    bool  _overheat;

    unsigned long _softStartUntil;
};

#endif // PID_CONTROLLER_H
