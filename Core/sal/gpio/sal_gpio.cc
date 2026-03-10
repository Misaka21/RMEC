#include "sal_gpio.h"

namespace sal {

    std::vector<GpioInstance::GpioPtr> GpioInstance::instance_list_;

    /**
     * @brief GPIO实例构造函数
     *
     * @param config GPIO配置结构体
     */
    GpioInstance::GpioInstance(const GpioConfig &config)
        : port_(config.port),
          pin_(config.pin),
          exti_cbk_(config.exti_cbk)
    {
        if (port_ == nullptr)
            DEBUG_DEADLOCK("[sal::GPIO] port is nullptr");
        if (pin_ == 0)
            DEBUG_DEADLOCK("[sal::GPIO] pin is 0");

        instance_list_.push_back(this);
    }

    void GpioInstance::Set()
    {
        HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
    }

    void GpioInstance::Reset()
    {
        HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
    }

    void GpioInstance::Toggle()
    {
        HAL_GPIO_TogglePin(port_, pin_);
    }

    GPIO_PinState GpioInstance::Read()
    {
        return HAL_GPIO_ReadPin(port_, pin_);
    }

    void GpioInstance::Write(GPIO_PinState state)
    {
        HAL_GPIO_WritePin(port_, pin_, state);
    }

    /**
     * @brief HAL GPIO外部中断回调函数,遍历实例列表按pin匹配后调用exti_cbk
     *
     * @param GPIO_Pin 触发中断的引脚
     */
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        for (auto &ins : GpioInstance::instance_list_)
        {
            if (ins->pin_ == GPIO_Pin && ins->exti_cbk_ != nullptr)
            {
                ins->exti_cbk_();
                return;
            }
        }
    }

} // namespace sal
