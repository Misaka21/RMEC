#pragma once

#include <cstdint>

/// 电机编码器反馈数据 + 多圈角度追踪
struct MotorMeasure {
    uint16_t last_ecd = 0;
    uint16_t ecd = 0;
    float angle_single_round = 0;   // 单圈角度 (deg)
    float speed_aps = 0;            // 角速度 (deg/s), 经 LPF 平滑
    int16_t real_current = 0;       // 实际电流/扭矩, 经 LPF 平滑
    uint8_t temperature = 0;
    float total_angle = 0;          // 多圈总角度 (deg)
    int32_t total_round = 0;        // 总圈数

    /// 根据新旧编码器值更新多圈角度
    /// @param ecd_angle_coef 编码器→角度系数 (e.g. 360/8192)
    void UpdateAngle(float ecd_angle_coef) {
        // 计算单圈角度
        angle_single_round = ecd_angle_coef * static_cast<float>(ecd);

        // 多圈检测：编码器跳变超过半圈判定为过零
        int16_t delta = static_cast<int16_t>(ecd) - static_cast<int16_t>(last_ecd);
        if (delta > 4096)
            total_round--;
        else if (delta < -4096)
            total_round++;

        total_angle = total_round * 360.0f + angle_single_round;
    }
};
