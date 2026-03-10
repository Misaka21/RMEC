#pragma once
#include "main.h"
#include "xTools.hpp"
#include <functional>
#include <vector>

namespace sal {

    class GpioInstance {
        friend void HAL_GPIO_EXTI_Callback(uint16_t);

    public:
        using GpioPtr          = GpioInstance*;
        using GpioExtiCallback = std::function<void()>;

        struct GpioConfig {
            GPIO_TypeDef      *port     = nullptr;
            uint16_t           pin      = 0;
            GpioExtiCallback   exti_cbk = nullptr;
        };

    private:
        static std::vector<GpioPtr> instance_list_;

        GPIO_TypeDef      *port_;
        uint16_t           pin_;
        GpioExtiCallback   exti_cbk_;

    public:
        GpioInstance(const GpioConfig &config);
        ~GpioInstance() = default;

        void Set();
        void Reset();
        void Toggle();
        GPIO_PinState Read();
        void Write(GPIO_PinState state);
    };

} // namespace sal
