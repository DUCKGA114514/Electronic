/* CAN 底层发送接口：对 HAL CAN 发送流程做轻量封装。 */
//
// Created by lisil on 2026/3/1.
//

#ifndef GIMBAL_BSP_CAN_H
#define GIMBAL_BSP_CAN_H

#include <stdint.h>
#include "can.h"
#define CAN_ID1     0x1FF
#define CAN_ID2     0x2FF

#define hcanx       &hcan1

typedef struct
{
    uint32_t can1_irq_cnt;
    uint32_t can2_irq_cnt;
    uint32_t can1_hal_rx_err_cnt;
    uint32_t can2_hal_rx_err_cnt;
    uint32_t can1_non_std_or_rtr_drop_cnt;
    uint32_t can2_non_std_or_rtr_drop_cnt;
    uint32_t can1_dispatch_cnt;
    uint32_t can2_dispatch_cnt;
    uint32_t can1_gm3508_cnt;
    uint32_t can2_gm6020_cnt;
    uint32_t can2_last_std_id;
    uint32_t can2_last_dlc;
    uint32_t can2_last_tick_ms;
} CAN_DebugStats_t;

uint8_t CAN_Send_Handle(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint8_t Length);
uint8_t CAN1_Send(uint16_t ID, uint8_t *Data, uint8_t Length);
const volatile CAN_DebugStats_t *CAN_GetDebugStats(void);

#endif //GIMBAL_BSP_CAN_H
