#include "ins_task.hpp"
#include "bmi088.hpp"
#include "ins.hpp"
#include "ahrs_math.hpp"
#include "robot_topics.hpp"
#include "TaskManager.hpp"
#include "sal_dwt.h"

// 文件作用域静态变量
static Bmi088* imu = nullptr;
static Ins* ins = nullptr;
static DwtInstance dwt;
static DwtInstance heat_dwt;
static uint32_t count = 0;

void InsTaskStart() {
    static TaskManager ins_task({
        .name       = "ins",
        .stack_size = 512,
        .priority   = osPriorityRealtime,
        .period_ms  = 1,

        .init_func = []() {
            // 1. 创建 BMI088 (~6s 阻塞校准)
            Bmi088Config bmi_cfg{};
            bmi_cfg.spi_handle = &hspi1;
            // TODO: 根据板级硬件填写 CS 和 EXTI 引脚
            // bmi_cfg.acc_cs_port   = GPIOx;
            // bmi_cfg.acc_cs_pin    = GPIO_PIN_x;
            // bmi_cfg.gyro_cs_port  = GPIOx;
            // bmi_cfg.gyro_cs_pin   = GPIO_PIN_x;
            bmi_cfg.heat_pid_config.kp = 1000;
            bmi_cfg.heat_pid_config.ki = 20;
            bmi_cfg.heat_pid_config.kd = 0;
            bmi_cfg.heat_pid_config.max_out = 2000;
            bmi_cfg.heat_pid_config.integral_limit = 300;
            bmi_cfg.heat_pid_config.improve_flags = pid::INTEGRAL_LIMIT;
            bmi_cfg.heat_tim_handle  = &htim10;
            bmi_cfg.heat_tim_channel = TIM_CHANNEL_1;
            bmi_cfg.heat_target_temp = 40.0f;
            imu = new Bmi088(bmi_cfg);

            // 2. 读 100 样本, 计算初始四元数
            float acc_sum[3] = {};
            Bmi088Data d;
            for (int i = 0; i < 100; ++i) {
                imu->Acquire(d);
                acc_sum[0] += d.acc[0];
                acc_sum[1] += d.acc[1];
                acc_sum[2] += d.acc[2];
                DwtInstance::DwtDelay(0.001f);
            }
            for (auto& a : acc_sum) a /= 100.0f;

            float init_q[4];
            ahrs::GravityToQuaternion(acc_sum, init_q);

            // 3. 创建 Ins
            ins = new Ins();
            ins->Init(init_q);

            // 4. 预热 DWT 计时器
            dwt.DwtGetDeltaT();
            heat_dwt.DwtGetDeltaT();
        },

        .task_func = []() {
            // 1kHz 主循环: 读→算→发→控
            float dt = dwt.DwtGetDeltaT();
            Bmi088Data bmi_data;
            imu->Acquire(bmi_data);

            ins->Update(bmi_data.gyro, bmi_data.acc, dt);

            InsData pub = ins->Data();
            pub.temperature = bmi_data.temperature;
            ins_topic.Publish(pub);

            if (count++ % 2 == 0)
                imu->HeaterCtrl(heat_dwt.DwtGetDeltaT());
        },
    });
}
