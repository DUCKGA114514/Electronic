#include "stm32f10x.h"
#include "CAN.h"

//全局变量，主要是std id 和最大输出
#define GM6020_TX_STDID_1_4 0x1FF
#define GM6020_TX_STDID_5_7 0x2FF
#define GM6020_VCMD_MAX     10000

//限幅公式
static  int16_t clamp_i16(int16_t x, int16_t mn, int16_t mx)
{
    if (x < mn) return mn;
    if (x > mx) return mx;
    return x;
}

void MyCAN_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    // PA11 RX (IPU), PA12 TX (AF_PP)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    CAN_DeInit(CAN1);

    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CANfilter_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    CAN_StructInit(&CAN_InitStructure);

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = ENABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
    CAN_InitStructure.CAN_Prescaler = 3;
    CAN_Init(CAN1, &CAN_InitStructure);

    //单个电机，无需配置
    CANfilter_InitStructure.CAN_FilterNumber = 0;
    CANfilter_InitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CANfilter_InitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CANfilter_InitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CANfilter_InitStructure.CAN_FilterActivation = ENABLE;

    CANfilter_InitStructure.CAN_FilterIdHigh     = 0x0000;
    CANfilter_InitStructure.CAN_FilterIdLow      = 0x0000;
    CANfilter_InitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CANfilter_InitStructure.CAN_FilterMaskIdLow  = 0x0000;
    CAN_FilterInit(&CANfilter_InitStructure);

    // FIFO0 接收中断
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*以下由AI生成，是对自己写的废案的优化*/
static void GM6020_SendFrame(uint16_t stdid,
                             int16_t c1, int16_t c2, int16_t c3, int16_t c4)
{
	//这里id计算发送的，参考了我之前的作业，作业7,还有AI
    CanTxMsg tx;
    tx.StdId = stdid;
    tx.IDE   = CAN_Id_Standard;
    tx.RTR   = CAN_RTR_Data;
    tx.DLC   = 8;

    int16_t v[4] = {c1, c2, c3, c4};
    for (int i = 0; i < 4; i++)
    {
		//限幅，分别对应1~4个电机
        v[i] = clamp_i16(v[i], -GM6020_VCMD_MAX, GM6020_VCMD_MAX);
		
		//发送，用了一点点数学逻辑
        tx.Data[2*i]     = (uint8_t)(v[i] >> 8);
        tx.Data[2*i + 1] = (uint8_t)(v[i] & 0xFF);
    }
    CAN_Transmit(CAN1, &tx);
}

/* 只控制 ID=3 和 ID=7 同时发送，防止延迟） */
//void GM6020_SendVoltage_ID3_ID7(int16_t v_id3, int16_t v_id7)
//{
//    GM6020_SendFrame(GM6020_TX_STDID_1_4, 0, 0, v_id3, 0);

//    GM6020_SendFrame(GM6020_TX_STDID_1_4, 0, 0, v_id7, 0);
//}

void GM6020_SendVoltage_ID3_ID7(int16_t v_id7,int16_t v_id3)
{
    GM6020_SendFrame(GM6020_TX_STDID_5_7, 0, 0, v_id7, 0);

    GM6020_SendFrame(GM6020_TX_STDID_1_4, 0, 0, v_id3, 0);
}


//下面是两个废案，写复杂了
//void GM6020_SendVoltage(uint8_t motor_id, int16_t v_cmd)
//{
//    
//    if (motor_id < 1 || motor_id > 7) return;

//    v_cmd = clamp_i16(v_cmd, -GM6020_VCMD_MAX, GM6020_VCMD_MAX);

//    CanTxMsg tx;
//    tx.IDE = CAN_Id_Standard;
//    tx.RTR = CAN_RTR_Data;
//    tx.DLC = 8;

//    for (int i = 0; i < 8; i++) tx.Data[i] = 0;

//    uint8_t idx;
//    if (motor_id <= 4)
//    {
//        /* 0x1FF: ID1~ID4 */
//        tx.StdId = GM6020_TX_STDID_1_4;
//        idx = (motor_id - 1) * 2;       // 0,2,4,6
//    }
//    else
//    {
//        /* 0x2FF: ID5~ID7*/
//        tx.StdId = GM6020_TX_STDID_5_7;
//        idx = (motor_id - 5) * 2;  
//    }

//    tx.Data[idx]     = (uint8_t)(v_cmd >> 8);
//    tx.Data[idx + 1] = (uint8_t)(v_cmd & 0xFF);

//    CAN_Transmit(CAN1, &tx);
//}

//void GM6020_SendVoltage(uint8_t motor_id, int16_t v_cmd)
//{
//    if (motor_id < 1 || motor_id > 8) return;

//    v_cmd = clamp_i16(v_cmd, -GM6020_VCMD_MAX, GM6020_VCMD_MAX);

//    CanTxMsg tx;
//    tx.IDE = CAN_Id_Standard;
//    tx.RTR = CAN_RTR_Data;
//    tx.DLC = 8;

//    // 先清零
//    for (int i = 0; i < 8; i++) tx.Data[i] = 0;

//    // 选择发送帧 ID：1~4 用 0x1FF，5~8 用 0x2FF
//    if (motor_id <= 4) tx.StdId = GM6020_TX_STDID_1_4;
//    else              tx.StdId = GM6020_TX_STDID_5_8;

//    // 计算在帧内的通道（0~3），每个通道 2 字节，大端
//    uint8_t ch = (motor_id - 1) & 0x03;
//    uint8_t idx = ch * 2;

//    tx.Data[idx]     = (uint8_t)(v_cmd >> 8);
//    tx.Data[idx + 1] = (uint8_t)(v_cmd & 0xFF);

//    CAN_Transmit(CAN1, &tx);
//}



