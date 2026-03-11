#pragma once

#include "motor_measure.hpp"

#include <type_traits>

/// 编译期检查 Driver 接口
template <typename D>
constexpr bool CheckDriverInterface() {
    // Driver 必须有: SetOutput(float), Measure(), IsOnline(), TickOffline()
    static_assert(std::is_same_v<
        decltype(std::declval<D>().SetOutput(0.0f)), void>,
        "Driver must have void SetOutput(float)");
    static_assert(std::is_same_v<
        decltype(std::declval<const D>().Measure()), const MotorMeasure&>,
        "Driver must have const MotorMeasure& Measure() const");
    static_assert(std::is_same_v<
        decltype(std::declval<const D>().IsOnline()), bool>,
        "Driver must have bool IsOnline() const");
    static_assert(std::is_same_v<
        decltype(std::declval<D>().TickOffline()), void>,
        "Driver must have void TickOffline()");
    return true;
}

/// 编译期检查 Controller 接口
template <typename C>
constexpr bool CheckControllerInterface() {
    static_assert(std::is_default_constructible_v<C>,
        "Controller must be default-constructible");
    // Controller 必须有: float Compute(float ref, const MotorMeasure& m, float dt)
    static_assert(std::is_same_v<
        decltype(std::declval<C>().Compute(0.0f, std::declval<const MotorMeasure&>(), 0.0f)), float>,
        "Controller must have float Compute(float, const MotorMeasure&, float)");
    return true;
}

/// Motor 模板：组合 Driver（CAN 协议编解码）与 Controller（PID 算法）
///
/// 零虚函数设计，编译器可完全内联 Controller 部分。
///
/// 使用方式:
///   Motor<DjiDriver, CascadePid> m3508(dji_cfg, cascade_cfg);
///   m3508.SetRef(1000.0f);
///   m3508.Update(dt);           // 一步完成: Compute + Apply
///   DjiDriver::FlushAll();       // 批量发送
///
/// 两阶段 API（功率控制场景）:
///   float out = m3508.ComputeOutput(dt);  // 算 PID，不发
///   out *= power_scale;                    // 功率缩放
///   m3508.ApplyOutput(out);               // 写入 Driver
///   DjiDriver::FlushAll();
template <typename Driver, typename Controller>
class Motor {
    static_assert(CheckDriverInterface<Driver>());
    static_assert(CheckControllerInterface<Controller>());

public:
    /// 构造：分别传入 Driver 和 Controller 的配置
    template <typename DriverCfg, typename ControllerCfg>
    Motor(const DriverCfg& drv_cfg, const ControllerCfg& ctrl_cfg)
        : driver_(drv_cfg), controller_(ctrl_cfg) {}

    /// 仅 Driver 配置（Controller 使用默认构造，如 MitPassthrough）
    template <typename DriverCfg>
    explicit Motor(const DriverCfg& drv_cfg)
        : driver_(drv_cfg) {}

    // ---- 设定值 ----
    void SetRef(float ref) { ref_ = ref; }
    float GetRef() const { return ref_; }

    // ---- 使能/失能 ----
    void Enable() { enabled_ = true; }
    void Disable() { enabled_ = false; }
    bool IsEnabled() const { return enabled_; }

    // ---- 状态查询 ----
    bool IsOnline() const { return driver_.IsOnline(); }
    const MotorMeasure& Measure() const { return driver_.Measure(); }

    // ---- 一步更新 ----
    /// Update = ComputeOutput + ApplyOutput
    void Update(float dt) {
        driver_.TickOffline();
        if (!enabled_) {
            driver_.SetOutput(0);
            return;
        }
        float output = controller_.Compute(ref_, driver_.Measure(), dt);
        driver_.SetOutput(output);
    }

    // ---- 两阶段 API（支持功率控制） ----
    /// 计算 PID 输出，但不写入 Driver
    float ComputeOutput(float dt) {
        driver_.TickOffline();
        if (!enabled_)
            return 0;
        return controller_.Compute(ref_, driver_.Measure(), dt);
    }

    /// 将输出写入 Driver（可能经过功率缩放后）
    void ApplyOutput(float output) {
        if (!enabled_) {
            driver_.SetOutput(0);
            return;
        }
        driver_.SetOutput(output);
    }

    // ---- 底层访问 ----
    Driver& GetDriver() { return driver_; }
    const Driver& GetDriver() const { return driver_; }
    Controller& GetController() { return controller_; }
    const Controller& GetController() const { return controller_; }

private:
    Driver driver_;
    Controller controller_{};
    float ref_ = 0;
    bool enabled_ = false;
};
