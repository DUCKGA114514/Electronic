/* 3508 底盘电机反馈解析与电流下发模块。 */
#include "GM3508.h"
#include <string.h>
#include "bsp_CAN.h"
#include "can.h"

/* s_gm3508_motor：保存四个底盘 3508 电机的最近一次反馈数据。 */
//分为1234四个电机
static GM3508_Motor_t s_gm3508_motor[GM3508_MOTOR_NUM];

//初始化电机
void GM3508_Init(void)
{
    memset(s_gm3508_motor, 0, sizeof(s_gm3508_motor));
}

//处理CAN2通道接收到的数据
void GM3508_ProcessFeedback(const CanRxFrame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    if ((frame->std_id < GM3508_FEEDBACK_STDID_BASE) ||
        (frame->std_id >= (GM3508_FEEDBACK_STDID_BASE + GM3508_MOTOR_NUM)))
    {
        return;
    }

    uint8_t idx = (uint8_t)(frame->std_id - GM3508_FEEDBACK_STDID_BASE);
    s_gm3508_motor[idx].ecd = (uint16_t)((frame->data[0] << 8U) | frame->data[1]); //编码器原始角度
    s_gm3508_motor[idx].speed_rpm = (int16_t)((frame->data[2] << 8U) | frame->data[3]);  //转速RPM
    s_gm3508_motor[idx].given_current = (int16_t)((frame->data[4] << 8U) | frame->data[5]);//实际给定电流
    s_gm3508_motor[idx].temperature = frame->data[6];//温度
    s_gm3508_motor[idx].online = 1U; //是否在线
    s_gm3508_motor[idx].last_update_ms = HAL_GetTick(); //最后更新时间，用于判断还在不在线
}

//发送！
void GM3508_SendCurrentCAN1(const int16_t current_cmd[GM3508_MOTOR_NUM])
{
    /* 4 个 int16 电流值按大端方式打包成 8 字节 CAN 数据区。 */
    uint8_t tx_data[8];

    if (current_cmd == NULL)
    {
        return;
    }

    tx_data[0] = (uint8_t)((current_cmd[0] >> 8) & 0xFF);
    tx_data[1] = (uint8_t)(current_cmd[0] & 0xFF);
    tx_data[2] = (uint8_t)((current_cmd[1] >> 8) & 0xFF);
    tx_data[3] = (uint8_t)(current_cmd[1] & 0xFF);
    tx_data[4] = (uint8_t)((current_cmd[2] >> 8) & 0xFF);
    tx_data[5] = (uint8_t)(current_cmd[2] & 0xFF);
    tx_data[6] = (uint8_t)((current_cmd[3] >> 8) & 0xFF);
    tx_data[7] = (uint8_t)(current_cmd[3] & 0xFF);

    (void)CAN_Send_Handle(&hcan1, GM3508_CONTROL_STDID, tx_data, 8U);
}

//判断电机是否在线
void GM3508_Periodic1ms(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < GM3508_MOTOR_NUM; i++)
    {
        if ((now - s_gm3508_motor[i].last_update_ms) > GM3508_OFFLINE_TIMEOUT_MS)
        {
            s_gm3508_motor[i].online = 0U;
        }
    }
}

//外部接口，得到电机目前状态
const GM3508_Motor_t *GM3508_GetMotors(void)
{
    return s_gm3508_motor;
}

//全部在线为1 有一个不在线则为0
uint8_t GM3508_AllOnline(void)
{
    for (uint8_t i = 0; i < GM3508_MOTOR_NUM; i++)
    {
        if (s_gm3508_motor[i].online == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}
