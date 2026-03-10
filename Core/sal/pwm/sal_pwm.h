#pragma once
#include "tim.h"
#include "xTools.hpp"
#include <vector>

namespace sal {

    class PwmInstance {
    public:
        using PwmPtr = PwmInstance*;

        struct PwmConfig {
            TIM_HandleTypeDef *handle  = nullptr;
            uint32_t           channel = 0;
        };

    private:
        static std::vector<PwmPtr> instance_list_;

        TIM_HandleTypeDef *handle_;
        uint32_t           channel_;

    public:
        PwmInstance(const PwmConfig &config);
        ~PwmInstance() = default;

        void Start();
        void Stop();
        void SetCompare(uint32_t compare);
        void SetDutyCycle(float duty);
    };

} // namespace sal
