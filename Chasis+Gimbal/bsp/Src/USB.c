/*
 * 文件说明：
 * USB CDC 通信实现。
 * USB_Send 负责把姿态遥测打包后发给上位机；USB_CDC_RxCallback 则把上位机下发的视觉帧按定长协议重组并入队。
 */
#include "USB.h"

#include <string.h>

#include "Vision.h"

void USB_Send(SENDPACKET Data)
{
    /* 发送格式：HEAD + pitch(float) + yaw(float) + roll(float) + TAIL */
    static uint8_t buf[3 * 4 + 2] = {0};
    buf[0] = USB_HEAD;
    buf[13] = USB_TAIL;

    memcpy(&buf[1], &Data.pitch, sizeof(float));
    memcpy(&buf[5], &Data.yaw, sizeof(float));
    memcpy(&buf[9], &Data.roll, sizeof(float));

    (void)CDC_Transmit_FS(buf, sizeof(buf));
}

void USB_CDC_RxCallback(uint8_t *Data, uint32_t Len)
{
    static uint8_t buf[USB_FRAME_LEN] = {0};
    static uint8_t rx_cnt = 0U;

    if ((Data == NULL) || (Len == 0U))
    {
        return;
    }

    for (uint32_t i = 0; i < Len; i++)
    {
        /* 逐字节重组固定长度协议，适合 CDC 收包长度不固定的情况。 */
        uint8_t byte = Data[i];
        if (rx_cnt == 0U)
        {
            if (byte == USB_HEAD)
            {
                buf[rx_cnt++] = byte;
            }
            continue;
        }

        buf[rx_cnt++] = byte;
        if (rx_cnt >= USB_FRAME_LEN)
        {
            if (buf[USB_FRAME_LEN - 1U] == USB_TAIL)
            {
                UsbRxFrame_t frame;
                memcpy(frame.buf, buf, USB_FRAME_LEN);
                (void)osMessageQueuePut(USBRXQueueHandle, &frame, 0U, 0U);
                Vision_OnUsbFrame(&frame);
            }
            rx_cnt = 0U;
            memset(buf, 0, sizeof(buf));
        }
    }
}
