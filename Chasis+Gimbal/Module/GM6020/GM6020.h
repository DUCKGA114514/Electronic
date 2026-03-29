/* 6020 云台电机数据结构与接口声明。 */
#ifndef CHASIS_GM6020_H
#define CHASIS_GM6020_H

#include "main.h"
#include <stdint.h>
#include "reg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GM6020_MOTOR_NUM                2  //电机个数
#define GM6020_YAW_INDEX                0  //电机编号
#define GM6020_PITCH_INDEX              1  //电机编号

#define GM6020_YAW_ID                   1   //这里需要我们更改
#define GM6020_PITCH_ID                 2

#define GM6020_FB_STDID_BASE            0x204  //接收
#define GM6020_CTRL_STDID_CURRENT_1_4   0x1FF  //使用电压控制
#define GM6020_CTRL_STDID_CURRENT_5_8   0x2FF  //使用电压控制
#define GM6020_CTRL_MAX_CMD             16000
#define GM6020_OFFLINE_TIMEOUT_MS       100
#define GM6020_ENCODER_MAX              8192.0f
#define GM6020_DEG_PER_ECD              (360.0f / GM6020_ENCODER_MAX)

typedef struct
{
    uint8_t id;  //电机ID
    uint8_t online;
    uint32_t last_update_ms;

    uint16_t ecd; // 当前编码器值
    uint16_t ecd_last; //上次编码器值
    int32_t round_cnt; //圈数计算
    float mech_angle_deg; //机械角度
    float total_angle_deg; //总角度
    float speed_rpm; //转速
    float current_feedback;//电流反馈
    uint8_t temperature;//温度
} GM6020_Motor_t;

void GM6020_Init(void);
void GM6020_ProcessFeedback(const CanRxFrame_t *frame);
void GM6020_Periodic1ms(void);
void GM6020_SendCurrentsCAN2(int16_t yaw_current, int16_t pitch_current);
const GM6020_Motor_t *GM6020_GetMotors(void);
const GM6020_Motor_t *GM6020_GetMotorByIndex(uint8_t index);
uint8_t GM6020_AllOnline(void);

#ifdef __cplusplus
}
#endif

#endif // CHASIS_GM6020_H
