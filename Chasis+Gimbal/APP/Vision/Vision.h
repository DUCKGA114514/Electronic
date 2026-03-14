/* 视觉任务与 USB 收包回调的外部接口。 */
#ifndef CHASIS_VISION_H
#define CHASIS_VISION_H

#include "main.h"
#include "reg.h"

#ifdef __cplusplus
extern "C" {
#endif

void Vision_OnUsbFrame(const UsbRxFrame_t *frame);
void StartVision_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // CHASIS_VISION_H
