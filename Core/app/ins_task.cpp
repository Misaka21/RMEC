#include "ins_task.hpp"
#include "robot_def.hpp"
#include "bmi088.hpp"
#include "ins.hpp"
#include "ahrs_math.hpp"
#include "robot_topics.hpp"
#include "TaskManager.hpp"
#include "sal_dwt.h"

#include "spi.h"
#include "tim.h"

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)

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
            // 1. 从 robot_def.hpp 配置 BMI088
            Bmi088Config bmi_cfg{};
            bmi_cfg.spi_handle     = &IMU_SPI_HANDLE;
            bmi_cfg.acc_cs_port    = IMU_ACC_CS_PORT;
            bmi_cfg.acc_cs_pin     = IMU_ACC_CS_PIN;
            bmi_cfg.gyro_cs_port   = IMU_GYRO_CS_PORT;
            bmi_cfg.gyro_cs_pin    = IMU_GYRO_CS_PIN;
            bmi_cfg.acc_int_port   = IMU_ACC_INT_PORT;
            bmi_cfg.acc_int_pin    = IMU_ACC_INT_PIN;
            bmi_cfg.gyro_int_port  = IMU_GYRO_INT_PORT;
            bmi_cfg.gyro_int_pin   = IMU_GYRO_INT_PIN;

            bmi_cfg.heat_pid_config.kp             = IMU_HEAT_KP;
            bmi_cfg.heat_pid_config.ki             = IMU_HEAT_KI;
            bmi_cfg.heat_pid_config.kd             = IMU_HEAT_KD;
            bmi_cfg.heat_pid_config.max_out        = IMU_HEAT_MAX_OUT;
            bmi_cfg.heat_pid_config.integral_limit = IMU_HEAT_INTEGRAL_LIMIT;
            bmi_cfg.heat_pid_config.improve_flags  = pid::INTEGRAL_LIMIT;
            bmi_cfg.heat_tim_handle  = &IMU_HEAT_TIM;
            bmi_cfg.heat_tim_channel = IMU_HEAT_CHANNEL;
            bmi_cfg.heat_target_temp = IMU_HEAT_TARGET_TEMP;

            imu = new Bmi088(bmi_cfg);   // ~6s 阻塞校准

            // 2. 读 100 样本计算初始四元数
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
            // 1 kHz: 读→算→发→控
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

#else // CHASSIS_BOARD — 不运行 IMU 任务

void InsTaskStart() {}

#endif
