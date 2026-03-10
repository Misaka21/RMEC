#pragma once
#include "tim.h"
#include "xTools.hpp"
#include <vector>

namespace sal {

    class PWMInstance {
    public:
        using PWMPtr = PWMInstance*;

        struct PWMConfig {
            TIM_HandleTypeDef *handle  = nullptr;
            uint32_t           channel = 0;
        };

    private:
        static std::vector<PWMPtr> instance_list_;

        TIM_HandleTypeDef *handle_;
        uint32_t           channel_;

    public:
        PWMInstance(const PWMConfig &config);
        ~PWMInstance() = default;

        void Start();
        void Stop();
        void SetCompare(uint32_t compare);
        void SetDutyCycle(float duty);
    };

} // namespace sal
