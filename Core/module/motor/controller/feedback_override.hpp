#pragma once

/// 外部反馈覆盖（IMU 角度/角速度等）
/// 控制器公共类型，独立于具体控制器实现
struct FeedbackOverride {
    const float* angle_fb  = nullptr;   // 若非空，角度环使用此反馈
    const float* speed_fb  = nullptr;   // 若非空，速度环使用此反馈
};
