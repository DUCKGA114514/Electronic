/* USB CDC 打包/拆包接口：负责与上位机视觉端进行定长帧通信。 */
//
// Created by lisil on 2026/3/3.
//

#ifndef GIMBAL_USB_H
#define GIMBAL_USB_H

#include "reg.h"
#include <string.h>
#include <stdlib.h>
#include "cmsis_os2.h"
#include "reg.h"
#include "usbd_cdc_if.h"

#define USB_HEAD    0x00
#define USB_TAIL    0x01
#define USB_RX_HEAD  0x02
#define USB_RX_TAIL  0x03

/* USB_FRAME_LEN 在 reg.h 中统一定义（默认10字节） */

void USB_Send(SENDPACKET Data);

/* ST USB CDC 回调会把收到的数据扔进这里做“拆帧/入队列” */
void USB_CDC_RxCallback(uint8_t* Data, uint32_t Len);

#endif //GIMBAL_USB_H