/* INS 接口：向其他模块提供姿态数据读取和就绪状态查询。 */
#ifndef CHASIS_INS_H
#define CHASIS_INS_H

#include "main.h"
#include "reg.h"

#ifdef __cplusplus
extern "C" {
#endif

const Gyro_TypeDef *INS_GetData(void);
uint8_t INS_IsReady(void);
void StartINSTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // CHASIS_INS_H
