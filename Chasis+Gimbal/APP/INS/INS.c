/*
 * 文件说明:主体还是使用了大疆官方给的MahonyAHRS文件，即四元数解算
 * 姿态解算任务。
 * 周期读取 BMI088 陀螺仪/加速度计数据，并使用 MahonyAHRS 进行四元数融合，
 * 最终输出 ROLL/PITCH/YAW 角度和原始陀螺仪、加速度计数据供云台/底盘使用。
 * 具体的计算思路在MahonyAHRS.c文件里面我有进行解释!
 * 具体的计算思路在MahonyAHRS.c文件里面我有进行解释!
 * 具体的计算思路在MahonyAHRS.c文件里面我有进行解释!
 */
#include "INS.h"

#include <math.h>
#include <string.h>

#include "BMI088Driver.h"
#include "MahonyAHRS.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



/* s_ins_data：对外发布的惯导数据快照。 */
static Gyro_TypeDef s_ins_data;
/* s_ins_ready：BMI088 初始化成功后置位，供上层判断姿态是否可信。 */
static uint8_t s_ins_ready = 0U;
/* s_q：Mahony 算法内部维护的四元数，初始值为单位四元数。 */
static float s_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};



/* 四元数转欧拉角：供上层模块直接读取更直观的 ROLL/PITCH/YAW。 */
static void INS_QuaternionToEuler(const float q[4], float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    float pitch;
    if (fabsf(sinp) >= 1.0f)
    {
        pitch = copysignf((float)M_PI / 2.0f, sinp);
    }
    else
    {
        pitch = asinf(sinp);
    }
    float yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));

    //弧度转角度
    *roll_deg = roll * 57.29578f;
    *pitch_deg = pitch * 57.29578f;
    *yaw_deg = yaw * 57.29578f;
}

//对外接口
const Gyro_TypeDef *INS_GetData(void)
{
    return &s_ins_data;
}

//告诉上次IMU准备好没有
uint8_t INS_IsReady(void)
{
    return s_ins_ready;
}

void StartINSTask(void *argument)
{
    (void)argument;
    uint32_t last_wake = osKernelGetTickCount();
    float gyro[3] = {0.0f};
    float accel[3] = {0.0f};
    float temp = 0.0f;

    memset(&s_ins_data, 0, sizeof(s_ins_data));
    twoKp = 2.0f * 0.35f;//比例增益  Kp 大：修正更快，但可能抖
    twoKi = 2.0f * 0.02f;//积分增益  Ki 大：能消除长期漂移，但太大可能积累误差过多

    if (BMI088_init() == BMI088_NO_ERROR)
    {
        s_ins_ready = 1U;
    }
    else
    {
        s_ins_ready = 0U;
    }

    for (;;)
    {
        if (s_ins_ready != 0U)
        {
            //读取数据
            BMI088_read(gyro, accel, &temp);

            //使用大疆官方给的姿态解算文件融合
            MahonyAHRSupdateIMU(s_q,
                                gyro[0], gyro[1], gyro[2],
                                accel[0], accel[1], accel[2]);

            //保存原始传感器数据   为计算不成功而兜底
            //其它需要用到的也可以直接拿，避免被新数据覆盖
            s_ins_data.GX = gyro[0];
            s_ins_data.GY = gyro[1];
            s_ins_data.GZ = gyro[2];
            s_ins_data.AX = accel[0];
            s_ins_data.AY = accel[1];
            s_ins_data.AZ = accel[2];
            INS_QuaternionToEuler(s_q, &s_ins_data.ROLL, &s_ins_data.PITCH, &s_ins_data.YAW);//四元数转欧拉角
        }

        osDelayUntil(last_wake + 1U);//1khz
        last_wake += 1U;
    }
}
