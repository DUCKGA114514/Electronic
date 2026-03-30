/* 云台控制模块头文件：定义云台状态机、目标量、反馈量以及控制频率等约束。 */
#ifndef CHASIS_GIMBAL_H
#define CHASIS_GIMBAL_H

#include "main.h"
#include <stdint.h>
#include "PID.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_CTRL_HZ                     1000.0f //控制频率
#define GIMBAL_DT_S                        (1.0f / GIMBAL_CTRL_HZ)
#define GIMBAL_VISION_TIMEOUT_MS           80U
#define GIMBAL_SENTRY_YAW_PERIOD_MS        3500U
#define GIMBAL_SENTRY_PITCH_PERIOD_MS      3000U

//此处是最大最小洗限制区域,软件限位在这里调试
#define GIMBAL_YAW_MIN_DEG                 (-40.0f)
#define GIMBAL_YAW_MAX_DEG                 (140.0f)
#define GIMBAL_PITCH_MIN_DEG               (-42.0f)
#define GIMBAL_PITCH_MAX_DEG               (42.0f)
#define GIMBAL_SOFT_LIMIT_MARGIN_DEG       (2.0f)


//云台状态枚举
typedef enum
{
    GIMBAL_MODE_INIT = 0,
    GIMBAL_MODE_SENTRY,
    GIMBAL_MODE_AUTO_AIM,
    GIMBAL_MODE_SAFE
} GimbalMode_e;


typedef struct
{
    float target_world_yaw_deg;      /* 世界坐标目标偏航角 */
    float target_world_pitch_deg;    /* 世界坐标目标俯仰角 */
    float target_joint_yaw_deg;      /* 解耦后关节应达到的偏航角 */
    float target_joint_pitch_deg;    /* 解耦后关节应达到的俯仰角 */

    float current_joint_yaw_deg;
    float current_joint_pitch_deg;
    float current_world_yaw_deg;
    float current_world_pitch_deg;

    float body_yaw_deg;
    float body_pitch_deg;
    float body_roll_deg;

    float yaw_rate_dps;
    float pitch_rate_dps;

    int16_t yaw_current_cmd;
    int16_t pitch_current_cmd;
    uint8_t vision_online;
    uint8_t imu_ready;
    uint8_t motor_ready;
    GimbalMode_e mode;
} GimbalControl_t;


void StartGimbal_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // CHASIS_GIMBAL_H
