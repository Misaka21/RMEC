#include <gtest/gtest.h>

#include "power_limiter.hpp"

#include <cmath>
#include <cstdint>

namespace {

PowerLimiterConfig M3508Config() {
    PowerLimiterConfig cfg{};
    cfg.model.output_to_torque = (20.0f / 16384.0f) * 0.3f * (3591.0f / 187.0f);
    return cfg;
}

PowerMotorState MakeState(float out) {
    PowerMotorState s{};
    s.pid_output = out;
    s.speed_rad = 20.0f;
    s.set_speed_rad = 30.0f;
    s.max_output = 16384.0f;
    return s;
}

TEST(PowerLimiter, EnergyLoopProducesFiniteMaxPower) {
    PowerLimiter pl;
    pl.Init(M3508Config());
    pl.UpdateEnergyLoop(80.0f, 60.0f, 40.0f, 0.001f);
    EXPECT_TRUE(std::isfinite(pl.GetMaxPower()));
    EXPECT_GT(pl.GetMaxPower(), 0.0f);
}

TEST(PowerLimiter, ZeroDtDoesNotProduceNan) {
    PowerLimiter pl;
    pl.Init(M3508Config());
    pl.UpdateEnergyLoop(80.0f, 60.0f, 40.0f, 0.0f);
    EXPECT_TRUE(std::isfinite(pl.GetMaxPower()));
}

TEST(PowerLimiter, LimitKeepsOutputsFiniteAndClamped) {
    PowerLimiter pl;
    pl.Init(M3508Config());
    pl.UpdateEnergyLoop(80.0f, 60.0f, 40.0f, 0.001f);
    PowerMotorState states[4] = {
        MakeState(16000.0f), MakeState(-16000.0f),
        MakeState(8000.0f), MakeState(0.0f),
    };
    pl.Limit(states, 4);
    for (const auto& s : states) {
        EXPECT_TRUE(std::isfinite(s.pid_output));
        EXPECT_LE(std::fabs(s.pid_output), s.max_output);
    }
}

TEST(PowerLimiter, ZeroSpeedMotorsDoNotCrash) {
    PowerLimiter pl;
    pl.Init(M3508Config());
    pl.UpdateEnergyLoop(80.0f, 0.0f, 0.0f, 0.001f);  // 缓冲能量耗尽
    PowerMotorState states[2] = {MakeState(16000.0f), MakeState(16000.0f)};
    states[0].speed_rad = 0.0f;
    states[1].speed_rad = -0.001f;
    pl.Limit(states, 2);
    for (const auto& s : states) {
        EXPECT_TRUE(std::isfinite(s.pid_output));
    }
}

}  // namespace
