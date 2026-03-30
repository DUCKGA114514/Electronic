#ifndef __CAN_H
#define __CAN_H

#include <stdint.h>

void MyCAN_Init(void);

/**
 * @brief GM6020 电压指令发送（v_cmd），按官方手册 CAN 映射
 *        - 0x1FF 控制 ID=1~4：ID1->Data0/1, ID2->Data2/3, ID3->Data4/5, ID4->Data6/7
 *        - 0x2FF 控制 ID=5~7：ID5->Data0/1, ID6->Data2/3, ID7->Data4/5, Data6/7 为 Null
 * @param motor_id  1~7
 * @param v_cmd     -25000~25000
 */
void GM6020_SendVoltage(uint8_t motor_id, int16_t v_cmd);
void GM6020_SendVoltage_ID3_ID7(int16_t v_id7,int16_t v_id3);

#endif
