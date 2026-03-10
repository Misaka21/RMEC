#include "pid_controller.hpp"

PidController::PidController(const PidConfig& cfg) noexcept {
    Init(cfg);
}

void PidController::Init(const PidConfig& cfg) noexcept {
    // 先清零运行状态
    Reset();

    // 拷贝配置
    kp_ = cfg.kp;
    ki_ = cfg.ki;
    kd_ = cfg.kd;
    max_out_ = cfg.max_out;
    dead_band_ = cfg.dead_band;
    improve_flags_ = cfg.improve_flags;
    integral_limit_ = cfg.integral_limit;
    coef_a_ = cfg.coef_a;
    coef_b_ = cfg.coef_b;
    output_lpf_rc_ = cfg.output_lpf_rc;
    derivative_lpf_rc_ = cfg.derivative_lpf_rc;
}

void PidController::Reset() noexcept {
    measure_ = last_measure_ = 0;
    err_ = last_err_ = 0;
    ref_ = 0;
    pout_ = iout_ = dout_ = 0;
    iterm_ = last_iterm_ = 0;
    output_ = last_output_ = 0;
    last_dout_ = 0;
    error_count_ = 0;
    error_type_ = PidError::NONE;
}

void PidController::ErrorHandle() noexcept {
    if (std::fabs(output_) < max_out_ * 0.001f || std::fabs(ref_) < 0.0001f)
        return;
    if ((std::fabs(ref_ - measure_) / std::fabs(ref_)) > 0.95f) {
        error_count_++;
    } else {
        error_count_ = 0;
    }
    if (error_count_ > 500) {
        error_type_ = PidError::MOTOR_BLOCKED;
    }
}

float PidController::Calculate(float measure, float ref, float dt) noexcept {
    // 堵转检测
    if (improve_flags_ & pid::ERROR_HANDLE)
        ErrorHandle();

    // 更新输入
    measure_ = measure;
    ref_ = ref;
    err_ = ref_ - measure_;

    if (std::fabs(err_) > dead_band_) {
        // ---- 基本位置式 PID ----
        pout_ = kp_ * err_;
        iterm_ = ki_ * err_ * dt;
        dout_ = kd_ * (err_ - last_err_) / dt;

        // 梯形积分
        if (improve_flags_ & pid::TRAPEZOIDAL_INTEGRAL)
            iterm_ = ki_ * ((err_ + last_err_) / 2.0f) * dt;

        // 变速积分
        if (improve_flags_ & pid::CHANGING_INTEGRATION_RATE) {
            if (err_ * iout_ > 0) {
                float abs_err = std::fabs(err_);
                if (abs_err <= coef_b_) {
                    // 全积分区域，不做改变
                } else if (abs_err <= (coef_a_ + coef_b_)) {
                    iterm_ *= (coef_a_ - abs_err + coef_b_) / coef_a_;
                } else {
                    iterm_ = 0;
                }
            }
        }

        // 微分先行（D on measurement）
        if (improve_flags_ & pid::DERIVATIVE_ON_MEASUREMENT)
            dout_ = kd_ * (last_measure_ - measure_) / dt;

        // 微分滤波
        if (improve_flags_ & pid::DERIVATIVE_FILTER)
            dout_ = dout_ * dt / (derivative_lpf_rc_ + dt) +
                    last_dout_ * derivative_lpf_rc_ / (derivative_lpf_rc_ + dt);

        // 积分限幅 + 抗饱和
        if (improve_flags_ & pid::INTEGRAL_LIMIT) {
            float temp_iout = iout_ + iterm_;
            float temp_output = pout_ + iout_ + dout_;
            if (std::fabs(temp_output) > max_out_) {
                if (err_ * iout_ > 0)
                    iterm_ = 0;
            }
            if (temp_iout > integral_limit_) {
                iterm_ = 0;
                iout_ = integral_limit_;
            }
            if (temp_iout < -integral_limit_) {
                iterm_ = 0;
                iout_ = -integral_limit_;
            }
        }

        iout_ += iterm_;
        output_ = pout_ + iout_ + dout_;

        // 输出滤波
        if (improve_flags_ & pid::OUTPUT_FILTER)
            output_ = output_ * dt / (output_lpf_rc_ + dt) +
                      last_output_ * output_lpf_rc_ / (output_lpf_rc_ + dt);

        // 输出限幅
        if (output_ > max_out_)
            output_ = max_out_;
        if (output_ < -max_out_)
            output_ = -max_out_;
    } else {
        // 死区内
        output_ = 0;
        iterm_ = 0;
    }

    // 保存历史
    last_measure_ = measure_;
    last_output_ = output_;
    last_dout_ = dout_;
    last_err_ = err_;
    last_iterm_ = iterm_;

    return output_;
}
