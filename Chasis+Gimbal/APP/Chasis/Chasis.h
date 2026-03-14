/* 底盘控制模块头文件：集中定义模式、指令、轮速 PID 和总控状态。 */
#ifndef CHASIS_CHASIS_H
#define CHASIS_CHASIS_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * 全向轮步兵底盘控制模块
 * - FreeRTOS 1kHz 主控制
 * - DT7 遥控输入
 * - 四轮麦克纳姆/全向轮速度闭环
 * - 安全保护与模式机
 * ========================= */

#define CHASSIS_WHEEL_NUM                4U //电机个数
#define CHASSIS_WHEEL_RADIUS_M           0.0815f //轮半径
#define CHASSIS_HALF_LENGTH_M            0.1500f //底盘长度  需更改            //轮对角线距离 425mm = 0.425m
#define CHASSIS_HALF_WIDTH_M             0.1500f //底盘宽度  需更改
#define CHASSIS_HALF_SUM_M               (CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M)

#define CHASSIS_MOTOR_REDUCTION_RATIO    15.76f
#define CHASSIS_WHEEL_RPM_MAX            850.0f  //轮速最快
#define CHASSIS_CURRENT_MAX              12000.0f  //单电流最快
#define CHASSIS_TOTAL_CURRENT_MAX        24000.0f  //合电流最快

#define CHASSIS_CTRL_HZ                  1000.0f  //控制频率
#define CHASSIS_DT_S                     (1.0f / CHASSIS_CTRL_HZ)
#define CHASSIS_REMOTE_DEADBAND          12.0f
#define CHASSIS_TILT_LIMIT_DEG           45.0f

//状态机枚举
typedef enum
{
    CHASSIS_MODE_RELAX = 0,
    CHASSIS_MODE_REMOTE,
    CHASSIS_MODE_SPIN,
    CHASSIS_MODE_SAFE
} ChassisMode_e;

//车体目标方向
typedef struct
{
    float vx_mps;             /* 车体 x 方向速度，前进为正 */
    float vy_mps;             /* 车体 y 方向速度，左移为正 */
    float wz_radps;           /* 自旋角速度，逆时针为正 */
    uint8_t brake;            /* 是否强制刹车 */
} ChassisCommand_t;


//细化到每个轮子的目标速度
typedef struct
{
    float ref_rpm;      /* 目标轮速 */
    float fdb_rpm;      /* 反馈轮速 */
    float err;          /* 当前轮速误差 */
    float integral;     /* 误差积分项累计值 */
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float out_limit;    /* 电流输出限幅 */
    float i_limit;      /* 积分限幅，防止积分饱和 */
    float out;          /* 本轮最终输出电流 */
} ChassisWheelPid_t;


//控制结构体
typedef struct
{
    ChassisMode_e mode;
    ChassisMode_e last_mode;
    ChassisCommand_t cmd;
    float target_wheel_rpm[CHASSIS_WHEEL_NUM];
    int16_t motor_current[CHASSIS_WHEEL_NUM];
    uint8_t remote_online;
    uint8_t motor_online;
    uint8_t imu_valid;
    uint8_t emergency_stop;
    float pitch_deg;
    float roll_deg;
} ChassisControl_t;

void StartChassis_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // CHASIS_CHASIS_H
