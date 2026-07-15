#pragma once
// 假 CubeMX usart.h: 仅提供 sal_usart.h / log.h 在宿主机编译所需的最小类型
#include <stdint.h>

typedef struct UART_HandleTypeDef {
    int id;  // 测试用标识, 区分不同串口
} UART_HandleTypeDef;

#define HAL_MAX_DELAY 0xFFFFFFFFU

// log.h 会把本头包进 extern "C", 其他 TU 直接 include; 统一成 C 链接性
#ifdef __cplusplus
extern "C" {
#endif

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

#ifdef __cplusplus
}
#endif
