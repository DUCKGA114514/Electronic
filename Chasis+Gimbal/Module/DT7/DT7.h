/* DT7/DR16 遥控器模块头文件。 */
#ifndef CHASIS_DT7_H
#define CHASIS_DT7_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * DT7/DR16 遥控器解析模块
 * UART5 + DMA + 空闲中断
 * 标准 18 字节 SBUS 帧
 * ========================= */

#define DT7_FRAME_LEN                18U
#define DT7_RX_BUFFER_LEN            32U
#define DT7_CHANNEL_OFFSET           1024
#define DT7_CHANNEL_MAX_ABS          660
#define DT7_UPDATE_TIMEOUT_MS        100U

#define DT7_SWITCH_UP                1U
#define DT7_SWITCH_MID               3U
#define DT7_SWITCH_DOWN              2U

typedef struct
{
    int16_t ch[5];          /* 遥控五个模拟通道，已减去 1024 中值 */
    uint8_t s1;             /* 右侧三段开关 */
    uint8_t s2;             /* 左侧三段开关 */

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_l;
    uint8_t mouse_r;
    uint16_t key;

    uint8_t valid;          /* 最近一次解包是否有效 */
    uint8_t online;         /* 是否在线 */
    uint32_t last_update_ms;/* 最近一次收到有效帧的时间 */
} DT7_State_t;

void DT7_Init(UART_HandleTypeDef *huart);
void DT7_StartReceive(void);
void DT7_RxEventHandler(UART_HandleTypeDef *huart, uint16_t size);
void DT7_Periodic1ms(void);
const DT7_State_t *DT7_GetState(void);
uint8_t DT7_IsOnline(void);

#ifdef __cplusplus
}
#endif

#endif // CHASIS_DT7_H
