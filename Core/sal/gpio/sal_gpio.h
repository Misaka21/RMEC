#pragma once
#include "main.h"
#include "xTools.hpp"
#include <memory>
#include <functional>
#include <vector>

namespace sal {

    class GPIOInstance {
        friend void HAL_GPIO_EXTI_Callback(uint16_t);

    public:
        using GPIOPtr          = std::shared_ptr<GPIOInstance>;
        using GPIOExtiCallback = std::function<void()>;

        struct GPIOConfig {
            GPIO_TypeDef      *port     = nullptr;
            uint16_t           pin      = 0;
            GPIOExtiCallback   exti_cbk = nullptr;
        };

    private:
        static std::vector<GPIOPtr> instance_list_;

        GPIO_TypeDef      *port_;
        uint16_t           pin_;
        GPIOExtiCallback   exti_cbk_;

    public:
        GPIOInstance(const GPIOConfig &config);
        ~GPIOInstance() = default;

        void Set();
        void Reset();
        void Toggle();
        GPIO_PinState Read();
        void Write(GPIO_PinState state);
    };

} // namespace sal
