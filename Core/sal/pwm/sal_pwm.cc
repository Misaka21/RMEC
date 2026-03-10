#include "sal_pwm.h"

namespace sal {

    std::vector<PwmInstance::PwmPtr> PwmInstance::instance_list_;

    /**
     * @brief PWM实例构造函数
     *
     * @param config PWM配置结构体
     */
    PwmInstance::PwmInstance(const PwmConfig &config)
        : handle_(config.handle),
          channel_(config.channel)
    {
        if (handle_ == nullptr)
            DEBUG_DEADLOCK("[sal::PWM] handle is nullptr");

        instance_list_.push_back(this);
    }

    /**
     * @brief 启动PWM输出
     */
    void PwmInstance::Start()
    {
        HAL_TIM_PWM_Start(handle_, channel_);
    }

    /**
     * @brief 停止PWM输出
     */
    void PwmInstance::Stop()
    {
        HAL_TIM_PWM_Stop(handle_, channel_);
    }

    /**
     * @brief 设置比较值(直接设置CCR寄存器值)
     *
     * @param compare 比较值
     */
    void PwmInstance::SetCompare(uint32_t compare)
    {
        __HAL_TIM_SET_COMPARE(handle_, channel_, compare);
    }

    /**
     * @brief 设置占空比
     *
     * @param duty 占空比, 范围0.0~1.0
     */
    void PwmInstance::SetDutyCycle(float duty)
    {
        if (duty < 0.0f) duty = 0.0f;
        if (duty > 1.0f) duty = 1.0f;
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(handle_);
        __HAL_TIM_SET_COMPARE(handle_, channel_, (uint32_t)(duty * arr));
    }

} // namespace sal
