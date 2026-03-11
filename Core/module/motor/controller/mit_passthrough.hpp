#pragma once

#include "motor_measure.hpp"

/// MIT 直通控制器（Header-only）
/// HT/DM 等电机由电机端 MCU 完成 PID，主控只需透传设定值
class MitPassthrough {
public:
    using Ref = float;

    MitPassthrough() = default;

    /// 直接返回 ref，不做任何 PID 处理
    float Compute(const float& ref, const MotorMeasure& /*measure*/, float /*dt*/) {
        return ref;
    }
};
