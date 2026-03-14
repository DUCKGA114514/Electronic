/* 视觉任务：从 USB 队列中取出视觉帧并交给 USB 视觉模块解析，同时周期性回传遥测。 */
#include "Vision.h"

#include "USB_Vison.h"

void Vision_OnUsbFrame(const UsbRxFrame_t *frame)
{
    USBVision_ProcessRxFrame(frame);
}

void StartVision_Task(void *argument)
{
    (void)argument;

    USBVision_Init();

    for (;;)
    {
        UsbRxFrame_t frame;
        while (osMessageQueueGet(USBRXQueueHandle, &frame, NULL, 0U) == osOK)
        {
            Vision_OnUsbFrame(&frame);
        }

        USBVision_Periodic1ms();
        USBVision_SendTelemetry();
        osDelay(1);
    }
}
