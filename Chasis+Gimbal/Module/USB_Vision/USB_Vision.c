/* USB 视觉协议解析模块：维护视觉目标、在线状态以及给上位机回传的遥测缓存。 */
#include "USB_Vison.h"

#include <string.h>

#include "USB.h"

//视觉协议规定    需要修改
#define USB_VISION_RX_HEAD         0x00U  //接收包头
#define USB_VISION_RX_TAIL         0x01U  //接收包尾
//超时时间
#define USB_VISION_TIMEOUT_MS      80U
//视觉回传频率
#define USB_VISION_TX_PERIOD_MS    10U

/* s_usb_vision：当前最新视觉目标，供云台自瞄读取。 */
static USBVision_Target_t s_usb_vision;
/* s_usb_tx：回传给上位机的遥测缓存，通常包含当前姿态角。 */
static SENDPACKET s_usb_tx;
/* s_last_tx_ms：限速发送时间戳，避免 USB 回传过快占满带宽。 */
static uint32_t s_last_tx_ms = 0U;

/************************************此处参考了AI******************************/
/* 协议中角度数据以 IEEE754 float 形式发送，这里做字节流到 float 的转换。 */
static float USBVision_BytesToFloat(const uint8_t *buf)
{
    float value;
    memcpy(&value, buf, sizeof(float));
    return value;
}
/************************************此处参考了AI******************************/


void USBVision_Init(void)
{
    memset(&s_usb_vision, 0, sizeof(s_usb_vision));
    memset(&s_usb_tx, 0, sizeof(s_usb_tx));
    s_last_tx_ms = 0U;
}

//接收函数，将接收到的数据放到接受结构体里
void USBVision_ProcessRxFrame(const UsbRxFrame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    if ((frame->buf[0] != USB_VISION_RX_HEAD) || (frame->buf[USB_FRAME_LEN - 1U] != USB_VISION_RX_TAIL))
    {
        return;
    }

    //解析 pitch 和 yaw    需修改   此处是世界坐标系，也就是以大地为基础的坐标系
    s_usb_vision.pitch_world_deg = USBVision_BytesToFloat(&frame->buf[1]);
    s_usb_vision.yaw_world_deg = USBVision_BytesToFloat(&frame->buf[5]);

    s_usb_vision.aim_flag = 1U;
    s_usb_vision.online = 1U;
    s_usb_vision.last_update_ms = HAL_GetTick();
}

//视觉是否在线？
void USBVision_Periodic1ms(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_usb_vision.last_update_ms) > USB_VISION_TIMEOUT_MS)
    {
        s_usb_vision.online = 0U;
        s_usb_vision.aim_flag = 0U;
    }
}

//设置回传数据
void USBVision_SetTelemetry(float yaw_deg, float pitch_deg, float roll_deg)
{
    s_usb_tx.yaw = yaw_deg;
    s_usb_tx.pitch = pitch_deg;
    s_usb_tx.roll = roll_deg;
}

//回传
void USBVision_SendTelemetry(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_last_tx_ms) < USB_VISION_TX_PERIOD_MS)
    {
        return;
    }

    USB_Send(s_usb_tx);
    s_last_tx_ms = now;
}

//外部接口，用于得到视觉给的目标角度
const USBVision_Target_t *USBVision_GetTarget(void)
{
    return &s_usb_vision;
}
