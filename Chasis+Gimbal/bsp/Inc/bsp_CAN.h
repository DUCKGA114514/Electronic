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

uint8_t CAN_Send_Handle(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint8_t Length);
uint8_t CAN_1Send(uint16_t ID,uint8_t *Data,uint8_t Length);

#endif //GIMBAL_BSP_CAN_H