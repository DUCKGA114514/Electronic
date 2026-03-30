#include "UART1_DMA_Tx.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

//这里除初始化外都是用的AI和以往的代码


/*
 * USART1 TX (PA9) + DMA1_Channel4 非阻塞发送（环形缓冲）
 * 适用于 STM32F103C8（中容量）标准外设库工程
 *
 * 注意：TX_BUF_SIZE 设为 2^n（如 1024/2048），环形取模使用 & (SIZE-1)
 */

#define TX_BUF_SIZE 2048
/*此部分代码使用AI生成 + 以往的代码*/
static uint8_t  tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;

static volatile uint8_t  dma_busy = 0;
static volatile uint16_t dma_last_len = 0;

static inline uint16_t _buf_used(void) {
    return (uint16_t)((tx_head - tx_tail) & (TX_BUF_SIZE - 1));
}
static inline uint16_t _buf_free(void) {
    return (uint16_t)(TX_BUF_SIZE - 1 - _buf_used());
}

static void _start_dma_if_idle(void) {
    if (dma_busy) return;
    if (tx_head == tx_tail) return;

    uint16_t tail = tx_tail;
    uint16_t head = tx_head;

    uint16_t len;
    if (head > tail) len = head - tail;
    else len = TX_BUF_SIZE - tail;

    dma_busy = 1;
    dma_last_len = len;

    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CNDTR = len;
    DMA1_Channel4->CMAR  = (uint32_t)&tx_buf[tail];
    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);

    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}





void UART1_DMA_Tx_Init(uint32_t baud) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin   = GPIO_Pin_9;           // PA9 = USART1_TX
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef us = {0};
    us.USART_BaudRate   = baud;
    us.USART_WordLength = USART_WordLength_8b;
    us.USART_StopBits   = USART_StopBits_1;
    us.USART_Parity     = USART_Parity_No;
    us.USART_Mode       = USART_Mode_Tx;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &us);
    USART_Cmd(USART1, ENABLE);

    DMA_InitTypeDef dma = {0};
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    dma.DMA_MemoryBaseAddr     = (uint32_t)tx_buf;
    dma.DMA_DIR                = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize         = 0;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;
    dma.DMA_Priority           = DMA_Priority_Medium;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &dma);

    NVIC_InitTypeDef nv = {0};
    nv.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 1;
    nv.NVIC_IRQChannelSubPriority        = 1;
    nv.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nv);
}








/*以下部分使用AI+ 以往的代码合成*/
size_t UART1_DMA_Tx_Write(const uint8_t *data, size_t len) {
    if (!data || len == 0) return 0;

    uint16_t free = _buf_free();
    if (len > free) len = free;
    if (len == 0) return 0;

    for (size_t i = 0; i < len; i++) {
        tx_buf[tx_head] = data[i];
        tx_head = (uint16_t)((tx_head + 1) & (TX_BUF_SIZE - 1));
    }

    _start_dma_if_idle();
    return len;
}

size_t UART1_DMA_Tx_Printf(const char *fmt, ...) {
    char tmp[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    if (n <= 0) return 0;
    if (n > (int)sizeof(tmp)) n = sizeof(tmp);

    return UART1_DMA_Tx_Write((const uint8_t*)tmp, (size_t)n);
}

void DMA1_Channel4_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC4) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_TC4);

        tx_tail = (uint16_t)((tx_tail + dma_last_len) & (TX_BUF_SIZE - 1));

        dma_busy = 0;
        _start_dma_if_idle();
    }
}
