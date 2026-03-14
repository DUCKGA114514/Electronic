/*
 * 公共数据结构与通用宏定义。
 * 该文件像工程里的“公共协议中心”，底盘、云台、视觉、INS、USB、CAN 等模块都通过这里共享消息结构体和控制参数结构体。
 */
//
// Created by lisil on 2026/3/2.
//
#include "main.h"

#ifndef GIMBAL_REG_H
#define GIMBAL_REG_H

#ifndef ABS
#define ABS(x)                (( (x) >= 0 ) ? (x) : (-(x)))
#endif

#ifndef LIMIT_MAX_MIN
#define LIMIT_MAX_MIN(x,max,min) (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))
#endif

/* ==============================
 * RTOS 队列消息类型
 * 说明：
 * 1) CMSIS-RTOS2 的 MessageQueue 是“按元素拷贝”的，元素大小必须与真实消息一致。
 * 2) 这里统一定义“USB接收帧”“CAN接收帧”“云台目标命令”等消息，避免长度不匹配导致内存越界。
 * ============================== */

/* USB 固定帧长度：1(HEAD)+4(pitch)+4(yaw)+1(TAIL)=10 */
#ifndef USB_FRAME_LEN
#define USB_FRAME_LEN    10
#endif

typedef struct
{
    uint8_t buf[USB_FRAME_LEN];
} UsbRxFrame_t;

/* CAN 接收帧（标准帧 11bit） */
typedef struct
{
    uint16_t std_id;   // 标准帧ID
    uint8_t  data[8];  // 数据域
    uint8_t  bus;      // 1=CAN1, 2=CAN2（方便多CAN扩展）
} CanRxFrame_t;

/* 视觉/哨兵下发给 PID 位置环的目标角（单位：deg） */
typedef struct
{
    float yaw_deg;         // 视觉/上位机给出的目标偏航角（单位：deg）
    float pitch_deg;       // 视觉/上位机给出的目标俯仰角（单位：deg）
    uint8_t aim_flag;      // 1=视觉识别到装甲板/需要自瞄；0=未识别
    uint32_t stamp_ms;     // 接收时间戳，用于超时判断
} GimbalTargetCmd_t;

/*6020电机数据接收结构体*/
typedef struct
{
    uint16_t Angle;        //转子机械角度
    int16_t RealSpeed;    //转子转速
    int16_t Current;      //实际转矩电流
    uint8_t Temperature;  //电机温度
}DJI6020MotorReceive_t ;



/*视觉数据接收结构体*/
typedef struct
{
    int16_t Yaw_CameraToEnemy_ErrorAngle;
    int16_t Pitch_CameraToEnemy_ErrorAngle;
    uint8_t PC_Aiming_Flag;
}PCReceive_t;


/*陀螺仪数据接收结构体*/
typedef struct
{
    //加速度计速度
    float AX;
    float AY;
    float AZ;

    //陀螺仪速度
    float GX;
    float GY;
    float GZ;

    //陀螺仪角度
    float PITCH;
    float YAW;
    float ROLL;
}Gyro_TypeDef;

/*前馈PID结构体*/
typedef struct
{
    float K1; //增益系数K1
    float K2; //增益系数K2
    float Last_DeltIn; //上一次输入值
    float Pre_DeltIn; //当前输入值
    float Out;   //当前输出值
    float OutMax; //输出最大值
}FeedForward_t;

/*PID结构体*/
typedef struct
{
    float DeadZone;//死区：误差足够小时直接按 0 处理，减少抖动

    float SetPoint;//设定目标值
    float ActualValue;//实际值(反馈值)

    float P;//比例常数
    float I;//积分常数
    float D;//微分常数

    float Pout;//比例输出
    float Iout;//积分输出
    float Dout;//微分输出

    float LastError;//上一次误差
    float PreError;//当前误差(Present 当前)
    float SumError;//积分误差

    float Last_DOut;//上一次微分输出

    float Out;//总输出
    float OutMax;//最大输出限制
    float Last_Out;//上一次总输出

    float ErrorMax;//偏差上限，超过偏差则不计算积分作用
    float IMax;//积分限制
    float  I_U;//变速积分上限
    float  I_L;//变速积分下限

    float RC_DF;//不完全微分滤波系数

    char I_Flag;//是否能进入积分项(适用于底盘功率限制)

}PID_t;

/*USB待发送数据结构体*/
typedef struct
{
    float yaw;
    float pitch;
    float roll;
}SENDPACKET;



#endif //GIMBAL_REG_H