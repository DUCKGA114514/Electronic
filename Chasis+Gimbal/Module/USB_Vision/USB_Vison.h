/* USB 视觉模块头文件。 */
#ifndef CHASIS_USB_VISON_H
#define CHASIS_USB_VISON_H

#include "main.h"
#include <stdint.h>
#include "reg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float yaw_world_deg;      /* 上位机输出的世界坐标偏航角 */
    float pitch_world_deg;    /* 上位机输出的世界坐标俯仰角 */
    uint8_t aim_flag;         /* 是否检测到有效目标 */
    uint8_t online;           /* 视觉链路是否在线 */
    uint32_t last_update_ms;  /* 最近一次收到目标数据的时间戳 */
} USBVision_Target_t;

void USBVision_Init(void);
void USBVision_ProcessRxFrame(const UsbRxFrame_t *frame);
void USBVision_Periodic1ms(void);
void USBVision_SetTelemetry(float yaw_deg, float pitch_deg, float roll_deg);
void USBVision_SendTelemetry(void);
const USBVision_Target_t *USBVision_GetTarget(void);

#ifdef __cplusplus
}
#endif

#endif // CHASIS_USB_VISON_H
