/* 3508 底盘电机数据结构与接口声明。 */
#ifndef CHASIS_GM3508_H
#define CHASIS_GM3508_H

#include "main.h"
#include <stdint.h>
#include "reg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GM3508_MOTOR_NUM             4U
#define GM3508_CONTROL_STDID         0x200U
#define GM3508_FEEDBACK_STDID_BASE   0x201U
#define GM3508_OFFLINE_TIMEOUT_MS    100U

typedef struct
{
    uint16_t ecd;              /* 编码器原始角度 0~8191 */
    int16_t speed_rpm;         /* 实际转速 rpm */
    int16_t given_current;     /* 实际给定电流 */
    uint8_t temperature;       /* 温度 */
    uint8_t online;            /* 在线标志 */
    uint32_t last_update_ms;   /* 最近更新时间 */
} GM3508_Motor_t;

void GM3508_Init(void);
void GM3508_ProcessFeedback(const CanRxFrame_t *frame);
void GM3508_SendCurrentCAN1(const int16_t current_cmd[GM3508_MOTOR_NUM]);
void GM3508_Periodic1ms(void);
const GM3508_Motor_t *GM3508_GetMotors(void);
uint8_t GM3508_AllOnline(void);

#ifdef __cplusplus
}
#endif

#endif // CHASIS_GM3508_H
