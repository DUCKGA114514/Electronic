#pragma once
#include "stm32f10x.h"
#include <stdint.h>
#include <stddef.h>

void UART1_DMA_Tx_Init(uint32_t baud);
size_t UART1_DMA_Tx_Write(const uint8_t *data, size_t len);
size_t UART1_DMA_Tx_Printf(const char *fmt, ...);
