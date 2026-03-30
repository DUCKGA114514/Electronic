/*
 * 文件说明：
 * CAN 板级支持包。
 * 除发送封装外，本文件还在接收中断中按总线来源把反馈帧分发给 6020 云台电机或 3508 底盘电机解析模块。
 */
//
// Created by lisil on 2026/3/1.
//

#include "bsp_CAN.h"

#include "reg.h"
#include "bsp_CAN.h"

#include "can.h"
#include "main.h"
#include <string.h>
#include "GM3508.h"
#include "GM6020.h"

static void CAN_DispatchFrame(CAN_HandleTypeDef *hcan, uint32_t fifo)
{
     CAN_RxHeaderTypeDef RxHeader;
     uint8_t buf[8] = {0};
     CanRxFrame_t frame = {0};

     if (HAL_CAN_GetRxMessage(hcan, fifo, &RxHeader, buf) != HAL_OK)
     {
          return;
     }

     if (RxHeader.IDE != CAN_ID_STD || RxHeader.RTR != CAN_RTR_DATA)
     {
          return;
     }

     frame.std_id = (uint16_t)RxHeader.StdId;
     memcpy(frame.data, buf, (RxHeader.DLC <= 8U) ? RxHeader.DLC : 8U);
     frame.bus = (hcan->Instance == CAN1) ? 1U : 2U;

     if ((frame.bus == 1U) &&
         (frame.std_id >= GM3508_FEEDBACK_STDID_BASE) &&
         (frame.std_id < (GM3508_FEEDBACK_STDID_BASE + GM3508_MOTOR_NUM)))
     {
          GM3508_ProcessFeedback(&frame);
     }
     else if ((frame.bus == 2U) &&
              (frame.std_id >= GM6020_FB_STDID_BASE) &&
              (frame.std_id <= (GM6020_FB_STDID_BASE + 7U)))
     {
          GM6020_ProcessFeedback(&frame);
     }
}

/*
* 参数说明：
参数	作用
ID	CAN 标准帧ID（11位）  0x1FF  or 0x2FF
Data	要发送的数据指针
Length	数据长度（0~8字节）
返回值：

1 → 发送成功

0 → 发送失败
 */
uint8_t CAN_Send_Handle(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint8_t Length)
{
    if ((hcan == NULL) || (Data == NULL))
    {
        return 0;
    }

    if (Length > 8)
    {
        Length = 8;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        return 0;
    }

    CAN_TxHeaderTypeDef TxMessage;
    uint32_t TxMailbox;

    TxMessage.StdId = ID;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;
    TxMessage.DLC   = Length;
    TxMessage.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &TxMessage, Data, &TxMailbox) == HAL_OK)
    {
        return 1;
    }

    return 0;
}

//can发送函数
uint8_t CAN1_Send(uint16_t ID, uint8_t *Data, uint8_t Length)
{
    return CAN_Send_Handle(&hcan1, ID, Data, Length);
}

//CAN接收中断，待配置
/*
 启用后，只要 FIFO0 有数据：
 自动进入这个函数（中断上下文）

CAN收到数据
     ↓
进入中断回调
     ↓
读取FIFO0数据
     ↓
标记来自CAN1还是CAN2
     ↓
丢入RTOS消息队列
     ↓
退出中断


*/
/**
 *
 * @param hcan CAN句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
     CAN_DispatchFrame(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
     CAN_DispatchFrame(hcan, CAN_RX_FIFO1);
}
