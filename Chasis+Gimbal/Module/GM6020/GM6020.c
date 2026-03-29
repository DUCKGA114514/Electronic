/* 6020 云台电机反馈解析与电压下发模块。 */
#include "GM6020.h"

#include <string.h>

#include "bsp_CAN.h"
#include "can.h"

/* s_gm6020_motor：保存 yaw/pitch 两个 6020 电机的状态。 */
static GM6020_Motor_t s_gm6020_motor[GM6020_MOTOR_NUM];



//把ID 转化成数组的序号  如ID 1 -> [1]
static uint8_t GM6020_IdToIndex(uint8_t id)
{
    if (id == GM6020_YAW_ID)
    {
        return GM6020_YAW_INDEX;
    }
    if (id == GM6020_PITCH_ID)
    {
        return GM6020_PITCH_INDEX;
    }
    //不是已知ID 返回 OxFF表示无效
    return 0xFFU;
}



//6020初始化，把结构体内的ID变成 电机的 ID
void GM6020_Init(void)
{
    memset(s_gm6020_motor, 0, sizeof(s_gm6020_motor));
    s_gm6020_motor[GM6020_YAW_INDEX].id = GM6020_YAW_ID;
    s_gm6020_motor[GM6020_PITCH_INDEX].id = GM6020_PITCH_ID;
}



//解析收到电机反馈帧
void GM6020_ProcessFeedback(const CanRxFrame_t *frame)
{
    //让指针和总线空过渡，防止空指针，只关心CAN2传送过来的信息
    if ((frame == NULL) || (frame->bus != 2))
    {
        return;
    }

    //过滤标准帧 ID 范围  从0x205   到  0x208,如果不是的话就直接退出，它不是6020电机发来的数据
    if ((frame->std_id < (GM6020_FB_STDID_BASE + 1)) ||
        (frame->std_id > (GM6020_FB_STDID_BASE + 7)))
    {
        return;
    }

    //根据反馈ID算出电机编号   std_id  - 0x204 ==电机编号
    uint8_t id = (uint8_t)(frame->std_id - GM6020_FB_STDID_BASE);

    //通过GM6020)IdToIndex得到这是yaw轴电机还是pitch轴电机，方便计算
    uint8_t idx = GM6020_IdToIndex(id);

    if (idx >= GM6020_MOTOR_NUM)
    {
        return;
    }

    //取出目标电机结构体
    /*
    * data[0..1]：编码器角度 ecd
      data[2..3]：转速
      data[4..5]：电流
      data[6]：温度
     */
    GM6020_Motor_t *m = &s_gm6020_motor[idx];
    uint16_t new_ecd = (uint16_t)((frame->data[0] << 8) | frame->data[1]);


    if (m->last_update_ms != 0)
    {
        /* 编码器跨 0 点时会出现大跳变，这里通过半圈阈值判断“正反向过零”，
         * 把单圈角扩展成连续总角度，便于位置环直接使用。
         */
        //说人话就是防扣圈设计
        int16_t diff = (int16_t)new_ecd - (int16_t)m->ecd;
        if (diff > 4096)
        {
            m->round_cnt--;
        }
        else if (diff < -4096)
        {
            m->round_cnt++;
        }
    }
    //更新当前电机状态
    m->ecd_last = m->ecd;
    m->ecd = new_ecd;
    m->mech_angle_deg = (float)m->ecd * GM6020_DEG_PER_ECD;
    m->total_angle_deg = ((float)m->round_cnt * 360.0f) + m->mech_angle_deg;
    m->speed_rpm = (float)((int16_t)((frame->data[2] << 8U) | frame->data[3]));
    m->current_feedback = (float)((int16_t)((frame->data[4] << 8U) | frame->data[5]));
    m->temperature = frame->data[6];
    m->online = 1U; //收到反馈，电机在线
    m->last_update_ms = HAL_GetTick();
}



//周期性进行离线检测，如果last_update_ms大于设定的offline 离线设置值，就判断电机下线了
void GM6020_Periodic1ms(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < GM6020_MOTOR_NUM; i++)
    {
        if ((now - s_gm6020_motor[i].last_update_ms) > GM6020_OFFLINE_TIMEOUT_MS)
        {
            s_gm6020_motor[i].online = 0U;
        }
    }
}



//通过 CAN2 给 yaw 和 pitch 两个 6020 电机发送电流控制指令。
void GM6020_SendCurrentsCAN2(int16_t yaw_current, int16_t pitch_current)
{

    //CAN 数据区 8 字节，先全置零。
    uint8_t tx_data[8] = {0};

    //限幅
    yaw_current = (int16_t)LIMIT_MAX_MIN(yaw_current, GM6020_CTRL_MAX_CMD, -GM6020_CTRL_MAX_CMD);
    pitch_current = (int16_t)LIMIT_MAX_MIN(pitch_current, GM6020_CTRL_MAX_CMD, -GM6020_CTRL_MAX_CMD);

    //打包成 CAN 数据
    //复制，发送，视情况改=txdata的数组标号
    tx_data[0] = (uint8_t)((yaw_current >> 8) & 0xFF);
    tx_data[1] = (uint8_t)(yaw_current & 0xFF);
    tx_data[2] = (uint8_t)((pitch_current >> 8) & 0xFF);
    tx_data[3] = (uint8_t)(pitch_current & 0xFF);

    //根据需要修改STDID是1-4 还是 5-8
    (void)CAN_Send_Handle(&hcan2, GM6020_CTRL_STDID_CURRENT_1_4, tx_data, 8U);
}


//返回整个电机状态数组的首地址。
//让外部模块可以一次性访问所有电机状态。
const GM6020_Motor_t *GM6020_GetMotors(void)
{
    return s_gm6020_motor;
}


//按下标获取某一个电机状态。
const GM6020_Motor_t *GM6020_GetMotorByIndex(uint8_t index)
{
    if (index >= GM6020_MOTOR_NUM)
    {
        return NULL;
    }
    return &s_gm6020_motor[index];
}


//检查是否所有电机都在线。只要有一个不在线就返回0
uint8_t GM6020_AllOnline(void)
{
    for (uint8_t i = 0; i < GM6020_MOTOR_NUM; i++)
    {
        if (s_gm6020_motor[i].online == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}
