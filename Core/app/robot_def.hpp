#pragma once

// ======================== 板型 (三选一) ========================

#define ONE_BOARD
// #define CHASSIS_BOARD
// #define GIMBAL_BOARD

#if !defined(ONE_BOARD) && !defined(CHASSIS_BOARD) && !defined(GIMBAL_BOARD)
#error "No board type defined! Define exactly one: ONE_BOARD, CHASSIS_BOARD, or GIMBAL_BOARD."
#endif

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

// ======================== 裁判系统 ========================

#define REFEREE_UART_HANDLE      huart6  // 常规链路 (底盘板, 115200)
#define VIDEO_LINK_UART_HANDLE   huart1  // 图传链路 (云台板, 921600)

// ======================== 视觉通信 ========================

// 传输方式 (二选一, 默认 UART)
#define VISION_USE_UART
// #define VISION_USE_VCP

#ifdef VISION_USE_UART
#define VISION_UART_HANDLE  huart1
#endif

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

// 摩擦轮默认转速 (deg/s, 需实测标定)
inline constexpr float FRICTION_DEFAULT_SPEED  = 5000.0f;
// 拨弹盘连发转速 (deg/s)
inline constexpr float LOADER_BURST_SPEED      = 3600.0f;

// ======================== 电机 CAN 句柄 ========================

#define CHASSIS_CAN_HANDLE  hcan2
#define YAW_CAN_HANDLE      hcan1
#define PITCH_CAN_HANDLE    hcan1
#define FRICTION_CAN_HANDLE hcan2
#define LOADER_CAN_HANDLE   hcan2

// ======================== 底盘电机 PID ========================

// M3508 速度环 (输出单位: A)
inline constexpr float CHASSIS_SPEED_KP          = 0.0122f;
inline constexpr float CHASSIS_SPEED_KI          = 0.00122f;
inline constexpr float CHASSIS_SPEED_KD          = 0.0f;
inline constexpr float CHASSIS_SPEED_MAX_OUT     = 14.65f;
inline constexpr float CHASSIS_SPEED_INTEGRAL_LIMIT = 3.66f;

// 功率模型: M3508 减速比 19:1, 输入已经是安培
inline constexpr float CHASSIS_OUTPUT_TO_TORQUE =
    0.3f * (3591.0f / 187.0f);

// ======================== 云台电机 PID ========================

// Yaw GM6020 — 角度环 (外环, 输出: deg/s 速度参考, 不受电压换算影响)
inline constexpr float YAW_ANGLE_KP              = 8.0f;
inline constexpr float YAW_ANGLE_KI              = 0.0f;
inline constexpr float YAW_ANGLE_KD              = 0.0f;
inline constexpr float YAW_ANGLE_MAX_OUT         = 500.0f;

// Yaw GM6020 — 速度环 (内环, 输出单位: V)
inline constexpr float YAW_SPEED_KP              = 0.04f;
inline constexpr float YAW_SPEED_KI              = 0.008f;
inline constexpr float YAW_SPEED_KD              = 0.0f;
inline constexpr float YAW_SPEED_MAX_OUT         = 24.0f;
inline constexpr float YAW_SPEED_INTEGRAL_LIMIT  = 8.0f;

// Pitch GM6020 — 角度环 (外环, 输出: deg/s 速度参考, 不受电压换算影响)
inline constexpr float PITCH_ANGLE_KP            = 6.0f;
inline constexpr float PITCH_ANGLE_KI            = 0.0f;
inline constexpr float PITCH_ANGLE_KD            = 0.0f;
inline constexpr float PITCH_ANGLE_MAX_OUT       = 500.0f;

// Pitch GM6020 — 速度环 (内环, 输出单位: V)
inline constexpr float PITCH_SPEED_KP            = 0.04f;
inline constexpr float PITCH_SPEED_KI            = 0.008f;
inline constexpr float PITCH_SPEED_KD            = 0.0f;
inline constexpr float PITCH_SPEED_MAX_OUT       = 24.0f;
inline constexpr float PITCH_SPEED_INTEGRAL_LIMIT = 8.0f;

// GM6020 motor_id (拨码开关)
inline constexpr uint8_t YAW_MOTOR_ID            = 1;
inline constexpr uint8_t PITCH_MOTOR_ID          = 2;

// ======================== 发射电机 PID ========================

// 摩擦轮 M3508 — 速度环 (输出单位: A)
inline constexpr float FRICTION_SPEED_KP         = 0.0122f;
inline constexpr float FRICTION_SPEED_KI         = 0.00122f;
inline constexpr float FRICTION_SPEED_KD         = 0.0f;
inline constexpr float FRICTION_SPEED_MAX_OUT    = 14.65f;
inline constexpr float FRICTION_SPEED_INTEGRAL_LIMIT = 3.66f;

// 拨弹盘 M2006 — 速度环 (输出单位: A)
inline constexpr float LOADER_SPEED_KP           = 0.01f;
inline constexpr float LOADER_SPEED_KI           = 0.001f;
inline constexpr float LOADER_SPEED_KD           = 0.0f;
inline constexpr float LOADER_SPEED_MAX_OUT      = 10.0f;
inline constexpr float LOADER_SPEED_INTEGRAL_LIMIT = 3.0f;

// 拨弹盘 M2006 — 角度环 (输出: deg/s 速度参考, 不受电流换算影响)
inline constexpr float LOADER_ANGLE_KP           = 8.0f;
inline constexpr float LOADER_ANGLE_KI           = 0.0f;
inline constexpr float LOADER_ANGLE_KD           = 0.0f;
inline constexpr float LOADER_ANGLE_MAX_OUT      = 500.0f;

// 摩擦轮 motor_id (CAN2, 避开底盘 ID 1-4)
inline constexpr uint8_t FRICTION_L_MOTOR_ID     = 5;
inline constexpr uint8_t FRICTION_R_MOTOR_ID     = 6;
// 拨弹盘 motor_id
inline constexpr uint8_t LOADER_MOTOR_ID         = 7;

// ======================== 双板通信 ========================

#define COMM_CAN_HANDLE  hcan1
// 物理 ID 段: 云台→底盘 0x100~, 底盘→云台 0x110~
inline constexpr uint16_t COMM_GIMBAL2CHASSIS_BASE_ID = 0x100;
inline constexpr uint16_t COMM_CHASSIS2GIMBAL_BASE_ID = 0x110;

// 板级自动切换 Tx/Rx ID, 改 define 即全切
#if defined(GIMBAL_BOARD)
inline constexpr uint16_t COMM_BASE_TX_ID = COMM_GIMBAL2CHASSIS_BASE_ID;
inline constexpr uint16_t COMM_BASE_RX_ID = COMM_CHASSIS2GIMBAL_BASE_ID;
#elif defined(CHASSIS_BOARD)
inline constexpr uint16_t COMM_BASE_TX_ID = COMM_CHASSIS2GIMBAL_BASE_ID;
inline constexpr uint16_t COMM_BASE_RX_ID = COMM_GIMBAL2CHASSIS_BASE_ID;
#else // ONE_BOARD — 不使用, 保留定义供编译通过
inline constexpr uint16_t COMM_BASE_TX_ID = COMM_GIMBAL2CHASSIS_BASE_ID;
inline constexpr uint16_t COMM_BASE_RX_ID = COMM_CHASSIS2GIMBAL_BASE_ID;
#endif

// ======================== VOFA 调试绘图 ========================
// 注意: VOFA 与视觉共用 huart1, 二选一
// #define VOFA_ENABLED
// #define VOFA_UART_HANDLE    huart1

// ======================== IMU 安装方向 ========================

inline constexpr int8_t GYRO2GIMBAL_DIR_YAW    = 1;
inline constexpr int8_t GYRO2GIMBAL_DIR_PITCH  = 1;
inline constexpr int8_t GYRO2GIMBAL_DIR_ROLL   = 1;
