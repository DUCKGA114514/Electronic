/*
 * 文件说明：此处键盘部分参考了文华学院的步兵开源代码
 * DT7/DR16 遥控器解析模块。
 * 使用 UART 空闲中断 + DMA 接收 SBUS 数据帧，并在本文件中完成解包、范围保护和在线状态维护。
 */
#include "DT7.h"
#include <string.h>
#include "stm32f4xx_hal_uart.h"

/* s_dt7_uart：当前绑定的遥控器串口句柄。 */
static UART_HandleTypeDef *s_dt7_uart = NULL; //看传入的是哪一个UART口，并保存
/* s_dt7_rx_buf：DMA 接收缓存，空闲中断回调时从这里取出最新 SBUS 数据。 */
static uint8_t s_dt7_rx_buf[DT7_RX_BUFFER_LEN]; //DMA接收缓冲区
/* s_dt7_state：解析后的遥控器状态，供底盘等模块直接读取。 */
static DT7_State_t s_dt7_state;

//对通道值做限幅。
static int16_t DT7_ClampChannel(int16_t val)
{
    if (val > DT7_CHANNEL_MAX_ABS)
    {
        return DT7_CHANNEL_MAX_ABS;
    }
    if (val < -DT7_CHANNEL_MAX_ABS)
    {
        return -DT7_CHANNEL_MAX_ABS;
    }
    return val;
}

/*
* 判断拨杆开关值是否合法。
一般 DR16/DT7 的拨杆只有 3 个状态：
上
中
下
如果解析出的值不是这三个之一，就说明这一帧数据有问题。
 */
static uint8_t DT7_SwitchValid(uint8_t sw)
{
    return (sw == DT7_SWITCH_UP) || (sw == DT7_SWITCH_MID) || (sw == DT7_SWITCH_DOWN);
}



/* SBUS 解包核心函数：把 18 字节原始帧解析为 5 个通道值、双拨杆、鼠标和键盘数据。 */
//此处键盘部分参考了文华学院的步兵开源代码
/*此处中科大电控教程中可提炼出来
* SBUS 的特点是：
每个通道通常占 11 位
多个通道是按位紧凑打包的，不是简单一个字节一个通道
所以这里必须通过：
位移 >> <<
按位或 |
掩码 & 0x07FF
把每个 11 位通道取出来。
0x07FFU 的意义
0x07FF = 2047 = 11位全1
也就是只保留低 11 位。
- DT7_CHANNEL_OFFSET
SBUS 原始值通常以某个中值为中心，比如 1024。
减去偏移后就得到以 0 为中心的摇杆值。
例如：
原始值 1024 → 变成 0
原始值更大 → 正方向
原始值更小 → 负方向
 */
static uint8_t DT7_ParseFrame(const uint8_t *buf, DT7_State_t *state)
{
    DT7_State_t temp = {0};

    if ((buf == NULL) || (state == NULL))
    {
        return 0U;
    }

    temp.ch[0] = (int16_t)(((buf[0] | (buf[1] << 8U)) & 0x07FFU) - DT7_CHANNEL_OFFSET);
    temp.ch[1] = (int16_t)((((buf[1] >> 3U) | (buf[2] << 5U)) & 0x07FFU) - DT7_CHANNEL_OFFSET);
    temp.ch[2] = (int16_t)((((buf[2] >> 6U) | (buf[3] << 2U) | (buf[4] << 10U)) & 0x07FFU) - DT7_CHANNEL_OFFSET);
    temp.ch[3] = (int16_t)((((buf[4] >> 1U) | (buf[5] << 7U)) & 0x07FFU) - DT7_CHANNEL_OFFSET);
    //拨杆开关
    temp.s1 = (uint8_t)((buf[5] >> 4U) & 0x0003U);
    temp.s2 = (uint8_t)(((buf[5] >> 4U) & 0x000CU) >> 2U);
    temp.ch[4] = (int16_t)(((buf[16] | (buf[17] << 8U)) & 0x07FFU) - DT7_CHANNEL_OFFSET);

    //鼠标解析
    ////此处键盘部分参考了文华学院的步兵开源代码
    temp.mouse_x = (int16_t)(buf[6] | (buf[7] << 8U));
    temp.mouse_y = (int16_t)(buf[8] | (buf[9] << 8U));
    temp.mouse_z = (int16_t)(buf[10] | (buf[11] << 8U));
    temp.mouse_l = buf[12];
    temp.mouse_r = buf[13];
    //解析键盘数据
    temp.key = (uint16_t)(buf[14] | (buf[15] << 8U));

    /* 通道保护：异常值通常意味着串口丢帧/错帧，直接判无效 */
    //检查 5 个通道值是不是在合理范围内。
    for (uint8_t i = 0; i < 5U; i++)
    {
        if ((temp.ch[i] > 700) || (temp.ch[i] < -700))
        {
            return 0U;
        }
        temp.ch[i] = DT7_ClampChannel(temp.ch[i]);
    }

    /* S1/S2 拨杆损坏：不再以拨杆值作为整帧有效性的判据。 */

    //成功解析，状态更新
    temp.valid = 1U;
    temp.online = 1U;
    temp.last_update_ms = HAL_GetTick();
    *state = temp;
    return 1U;
}

void DT7_Init(UART_HandleTypeDef *huart)
{
    //绑定串口
    s_dt7_uart = huart;
    //状态清零
    memset(&s_dt7_state, 0, sizeof(s_dt7_state));
    //启用DMA接收
    DT7_StartReceive();
}

//启动接收
void DT7_StartReceive(void)
{
    //如果还没初始化，就不启动接收。
    if (s_dt7_uart == NULL)
    {
        return;
    }

    //启动 UART 空闲中断 + DMA 接收
    (void)HAL_UARTEx_ReceiveToIdle_DMA(s_dt7_uart, s_dt7_rx_buf, DT7_RX_BUFFER_LEN);
    //关闭 DMA 半传输中断此处是为了减少中断
    if (s_dt7_uart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(s_dt7_uart->hdmarx, DMA_IT_HT);
    }
}

void DT7_RxEventHandler(UART_HandleTypeDef *huart, uint16_t size)
{
    //判断是不是目标串口，以及长度够不够
    if ((huart != s_dt7_uart) || (size < DT7_FRAME_LEN))
    {
        if (huart == s_dt7_uart)
        {
            DT7_StartReceive();
        }
        return;
    }
    //如果不是绑定的 UART，直接不处理
    //如果收到的数据长度还不到一帧，也不处理
    /* ReceiveToIdle 可能一次收到多帧，这里总是取最后 18 字节有效载荷 */
    uint16_t start = (uint16_t)(size - DT7_FRAME_LEN);
    DT7_State_t parsed = {0};
    if (DT7_ParseFrame(&s_dt7_rx_buf[start], &parsed) != 0U)
    {
        s_dt7_state = parsed;
    }
    else
    {
        s_dt7_state.valid = 0U;
    }

    DT7_StartReceive();
}

//离线检测
void DT7_Periodic1ms(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_dt7_state.last_update_ms) > DT7_UPDATE_TIMEOUT_MS)
    {
        s_dt7_state.online = 0U;
        s_dt7_state.valid = 0U;
        memset(s_dt7_state.ch, 0, sizeof(s_dt7_state.ch));
    }
}

//对外接口
const DT7_State_t *DT7_GetState(void)
{
    return &s_dt7_state;
}


uint8_t DT7_IsOnline(void)
{
    return s_dt7_state.online;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    DT7_RxEventHandler(huart, Size);
}
