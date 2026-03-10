# ins_task

IMU 姿态估计任务, 1 kHz FreeRTOS 任务。

## 工作流程

### 初始化

1. 根据 `robot_def.hpp` 配置创建 `Bmi088` 实例 (PRE_CALIBRATED 模式跳过在线校准)
2. 读取 100 个加速度计样本, 计算平均重力方向
3. 调用 `ahrs::GravityToQuaternion()` 得到初始四元数
4. 创建 `Ins` 实例, 用初始四元数初始化 EKF
5. 预热 DWT 计时器

### 循环 (1 kHz)

1. `DwtGetDeltaT()` 获取精确 dt
2. `imu->Acquire(bmi_data)` 读取 BMI088 原始数据
3. `ins->Update(gyro, accel, dt, temperature)` EKF 更新
4. `ins_topic.Publish(ins->Data())` 发布姿态数据
5. 每 2 次循环调用 `imu->HeaterCtrl()` 控制 IMU 温度 (500 Hz)

### 条件编译

- `ONE_BOARD` 或 `GIMBAL_BOARD`: 运行完整 IMU 任务
- `CHASSIS_BOARD`: `InsTaskStart()` 为空函数
