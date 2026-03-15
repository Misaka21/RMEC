#ifndef _BSP_LOG_H
#define _BSP_LOG_H

/* 旧版 RTT 实现已移除, 见 git 历史 */

// ==================== UART 实现 (DAPLink / 串口助手) ====================

#include <cstdlib>

#ifdef __cplusplus
extern "C" {
#endif
#include "usart.h"
#ifdef __cplusplus
}
#endif

void BSPLogInit(UART_HandleTypeDef* handle);
int PrintLog(const char *fmt, ...);
void Float2Str(char *str, float va);

#define LOG_CLEAR()

#if DISABLE_LOG_SYSTEM
#define LOG(format, ...)
#define LOGINFO(format, ...)
#define LOGWARNING(format, ...)
#define LOGERROR(format, ...)
#else
#define LOG(format, ...)        PrintLog(format "\r\n", ##__VA_ARGS__)
#define LOGINFO(format, ...)    PrintLog("[I] " format "\r\n", ##__VA_ARGS__)
#define LOGWARNING(format, ...) PrintLog("[W] " format "\r\n", ##__VA_ARGS__)
#define LOGERROR(format, ...)   PrintLog("[E] " format "\r\n", ##__VA_ARGS__)
#endif

#endif
