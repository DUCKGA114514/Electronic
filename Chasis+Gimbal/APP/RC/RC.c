/* 遥控任务：完成 USB 视觉链路初始化，以及 DT7 遥控器的串口接收与在线检测。 */
#include "RC.h"
#include "usb_device.h"
#include "DT7.h"
#include "usart.h"

void StartRC_Task(void *argument)
{
    (void)argument;

    /* USB 视觉链路初始化仍然放在 RC 任务中，保证与现有工程兼容 */
    MX_USB_DEVICE_Init();

    /* DT7 遥控器使用 UART5 + DMA + 空闲中断接收 */
    DT7_Init(&huart5);

    for (;;)
    {
        /* 1ms 周期：只做轻量级在线性检测，保持任务职责单一 */
        DT7_Periodic1ms();
        osDelay(1);
    }
}
