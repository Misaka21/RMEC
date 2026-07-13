#include <gtest/gtest.h>

#include "pid_controller.hpp"

#include <cmath>

namespace {

PidConfig BareProportional(float kp, float max_out) {
    PidConfig cfg{};
    cfg.kp = kp;
    cfg.ki = 0;
    cfg.kd = 0;
    cfg.max_out = max_out;
    cfg.improve_flags = pid::NONE;
    return cfg;
}

TEST(PidController, ProportionalOnly) {
    PidController pid(BareProportional(2.0f, 1000.0f));
    float out = pid.Calculate(0.0f, 10.0f, 0.001f);
    EXPECT_FLOAT_EQ(out, 20.0f);
}

TEST(PidController, OutputClampedToMaxOut) {
    PidController pid(BareProportional(100.0f, 50.0f));
    float out = pid.Calculate(0.0f, 10.0f, 0.001f);
    EXPECT_LE(std::fabs(out), 50.0f);
}

// 回归: dt=0 时微分项除零曾产生 NaN/Inf 直通电机输出
TEST(PidController, ZeroDtDoesNotProduceNan) {
    PidConfig cfg = BareProportional(1.0f, 1000.0f);
    cfg.kd = 5.0f;
    PidController pid(cfg);

    float out = pid.Calculate(0.0f, 10.0f, 0.0f);
    EXPECT_TRUE(std::isfinite(out));

    // dt=0 后控制器仍可正常工作
    out = pid.Calculate(0.0f, 10.0f, 0.001f);
    EXPECT_TRUE(std::isfinite(out));
}

TEST(PidController, NegativeOrTinyDtAlsoGuarded) {
    PidConfig cfg = BareProportional(1.0f, 1000.0f);
    cfg.kd = 5.0f;
    PidController pid(cfg);
    EXPECT_TRUE(std::isfinite(pid.Calculate(0.0f, 10.0f, 1e-9f)));
}

}  // namespace
