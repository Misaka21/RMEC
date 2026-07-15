#pragma once
// 假 CubeMX can.h: 仅提供 sal_can.h 在宿主机编译所需的最小类型
#include <stdint.h>

typedef struct CAN_HandleTypeDef {
    int id;  // 测试用标识, 区分总线
} CAN_HandleTypeDef;

// 真 HAL 中 CAN2 是外设基址宏; sal_can.h/dji_driver.cpp 用 #ifdef CAN2 判断
// 芯片是否有第二路 CAN。宿主机测试模拟双 CAN 板 (C 板), 必须定义。
#define CAN2 1

typedef struct {
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
} CAN_TxHeaderTypeDef;

#ifdef __cplusplus
extern "C" {
#endif

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#ifdef __cplusplus
}
#endif
