#pragma once

// ======================== 板型 (三选一) ========================

#define ONE_BOARD
// #define CHASSIS_BOARD
// #define GIMBAL_BOARD

#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD))  || \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error "Board type conflict! Define exactly one."
#endif

// ======================== IMU 硬件映射 ========================

// SPI 总线 (main.h / spi.h 中的 extern 声明)
#define IMU_SPI_HANDLE      hspi1

// 加速度计 CS: PA4
#define IMU_ACC_CS_PORT     CS1_ACCEL_GPIO_Port
#define IMU_ACC_CS_PIN      CS1_ACCEL_Pin

// 陀螺仪 CS: PB0
#define IMU_GYRO_CS_PORT    CS1_GYRO_GPIO_Port
#define IMU_GYRO_CS_PIN     CS1_GYRO_Pin

// 加速度计 EXTI: PC4
#define IMU_ACC_INT_PORT    INT_ACC_GPIO_Port
#define IMU_ACC_INT_PIN     INT_ACC_Pin

// 陀螺仪 EXTI: PC5
#define IMU_GYRO_INT_PORT   INT_GYRO_GPIO_Port
#define IMU_GYRO_INT_PIN    INT_GYRO_Pin

// 加热器 PWM: TIM10 CH1, PF6
#define IMU_HEAT_TIM        htim10
#define IMU_HEAT_CHANNEL    TIM_CHANNEL_1

// 加热器 PID 参数
#include <cstdint>
inline constexpr float IMU_HEAT_KP             = 1000.0f;
inline constexpr float IMU_HEAT_KI             = 20.0f;
inline constexpr float IMU_HEAT_KD             = 0.0f;
inline constexpr float IMU_HEAT_MAX_OUT        = 2000.0f;
inline constexpr float IMU_HEAT_INTEGRAL_LIMIT = 300.0f;
inline constexpr float IMU_HEAT_TARGET_TEMP    = 40.0f;

// IMU 预校准值 (PRE_CALIBRATED 模式直接使用, ONLINE 模式超时回退)
inline constexpr float IMU_PRE_GYRO_OFFSET[3]  = {0.000994f, -0.004877f, 0.003771f};
inline constexpr float IMU_PRE_G_NORM           = 9.8909f;

// ======================== 遥控器 ========================

#define RC_UART_HANDLE      huart3

// ======================== 云台 ========================

inline constexpr uint16_t YAW_CHASSIS_ALIGN_ECD     = 2711;   // TODO: 实测标定
inline constexpr bool     YAW_ECD_GREATER_THAN_4096  = false;
inline constexpr uint16_t PITCH_HORIZON_ECD          = 3412;   // TODO: 实测标定
inline constexpr float    PITCH_MAX_ANGLE            = 25.0f;  // TODO: 实测
inline constexpr float    PITCH_MIN_ANGLE            = -20.0f;

// ======================== 底盘 ========================

inline constexpr float WHEEL_BASE              = 350.0f;  // mm, 前后轮距
inline constexpr float TRACK_WIDTH             = 300.0f;  // mm, 左右轮距
inline constexpr float CENTER_GIMBAL_OFFSET_X  = 0.0f;    // mm
inline constexpr float CENTER_GIMBAL_OFFSET_Y  = 0.0f;
inline constexpr float RADIUS_WHEEL            = 60.0f;   // mm
inline constexpr float REDUCTION_RATIO_WHEEL   = 19.0f;

// ======================== 发射 ========================

inline constexpr float ONE_BULLET_DELTA_ANGLE  = 36.0f;   // 度
inline constexpr float REDUCTION_RATIO_LOADER  = 36.0f;

// ======================== IMU 安装方向 ========================

inline constexpr int8_t GYRO2GIMBAL_DIR_YAW    = 1;
inline constexpr int8_t GYRO2GIMBAL_DIR_PITCH  = 1;
inline constexpr int8_t GYRO2GIMBAL_DIR_ROLL   = 1;
