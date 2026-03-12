/**
 * @brief FreeRTOS ucHeap 放置在 CCMRAM (0x10000000, 64KB)
 *
 * CCMRAM 只有 CPU 能访问, DMA 不可达.
 * 因此: 禁止从 FreeRTOS heap (new / pvPortMalloc) 分配 DMA buffer.
 * DMA buffer 必须用静态数组定义在普通 RAM 中.
 */
#include "FreeRTOS.h"

__attribute__((section(".ccmram"), used))
uint8_t ucHeap[configTOTAL_HEAP_SIZE];
