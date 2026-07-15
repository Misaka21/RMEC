#pragma once
// 假 CubeMX can.h: 仅提供 sal_can.h 在宿主机编译所需的最小类型
#include <stdint.h>

typedef struct CAN_HandleTypeDef {
    int id;  // 测试用标识, 区分总线
} CAN_HandleTypeDef;

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
