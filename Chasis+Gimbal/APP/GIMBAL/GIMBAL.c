/*
 * 文件说明：
 * 云台 1ms 控制任务。
 * 主流程：读取电机反馈/IMU/视觉目标 -> 选择模式（初始化/巡逻/自瞄/安全） -> 计算目标角度 ->
 * 执行角度环 + 速度环串级 PID -> 通过 CAN1 向 GM6020 发送电压指令。
 */
#include "GIMBAL.h"

#include <math.h>
#include <string.h>

#include "GM6020.h"
#include "INS.h"
#include "USB_Vison.h"
#include "can.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct
{
    PID_t angle_pid;   /* 外环角度环：位置误差 -> 角速度目标 */
    PID_t rate_pid;    /* 内环速度环：角速度误差 -> 电机电流指令 */
} GimbalAxisPid_t;

/* s_gimbal：云台控制总状态，汇总目标、反馈、模式与最终电流命令。 */
static GimbalControl_t s_gimbal;


/* yaw/pitch 两个轴各自采用“角度环 + 速度环”的串级控制。 */
static GimbalAxisPid_t s_yaw_pid;   //yaw轴的两个环-->angle  rate
static GimbalAxisPid_t s_pitch_pid; //pitch轴的两个环 -->angle rate


/* 仅做一次 CAN1 初始化，避免任务重入或异常恢复时重复开启。 */
static uint8_t s_can_started = 0U;


/* 零位用于把电机累计角换算成“相对安装中位”的关节角。 */
static float s_yaw_zero_deg = 0.0f;    //yaw 电机上电时的累计角，作为零位
static float s_pitch_zero_deg = 0.0f;  //pitch 电机上电时的累计角，作为零位
static uint8_t s_zero_inited = 0U;     //零位是否已经初始化完成



static void Gimbal_CANInitOnce(void);
static void Gimbal_Init(void);
static void Gimbal_UpdateStateEstimate(void);
static void Gimbal_UpdateMode(void);
static void Gimbal_UpdateTarget(void);
static void Gimbal_ApplySoftLimit(void);
static void Gimbal_RunController(void);
static void Gimbal_SendOutput(void);
static float Gimbal_Clamp(float value, float min_value, float max_value);
static float Gimbal_Wrap180(float deg);
static float Gimbal_Sawtooth(uint32_t period_ms, float min_deg, float max_deg);


/*初始化CAN1并启动接收中断*/
static void Gimbal_CANInitOnce(void)
{
    //如果已经初始化过就返回，保证不重复初始化
    if (s_can_started != 0U)
    {
        return;
    }

    //先不过滤，全部接收先这里后续需要调
    CAN_FilterTypeDef can1_filter = {0};
    can1_filter.FilterActivation = ENABLE;
    can1_filter.FilterBank = 0;
    can1_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can1_filter.FilterIdHigh = 0x0000;
    can1_filter.FilterIdLow = 0x0000;
    can1_filter.FilterMaskIdHigh = 0x0000;
    can1_filter.FilterMaskIdLow = 0x0000;
    can1_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can1_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can1_filter.SlaveStartFilterBank = 14;

    (void)HAL_CAN_ConfigFilter(&hcan1, &can1_filter);
    (void)HAL_CAN_Start(&hcan1);
    (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    //把started调成1U保证已经初始化完毕
    s_can_started = 1U;
}

/*云台初始化入口*/
static void Gimbal_Init(void)
{
    //全部清零，保证没有数据残留
    memset(&s_gimbal, 0, sizeof(s_gimbal));
    memset(&s_yaw_pid, 0, sizeof(s_yaw_pid));
    memset(&s_pitch_pid, 0, sizeof(s_pitch_pid));

    GM6020_Init();
    Gimbal_CANInitOnce();

    /***************************************************************************此处是PID调试地区**********************************************************************************/
    s_yaw_pid.angle_pid.P = 15.0f;
    s_yaw_pid.angle_pid.I = 0.00f;
    s_yaw_pid.angle_pid.D = 0.0f;
    s_yaw_pid.angle_pid.OutMax = 250.0f;
    s_yaw_pid.angle_pid.DeadZone = 0.05f;

    s_yaw_pid.rate_pid.P = 85.0f;
    s_yaw_pid.rate_pid.I = 4.0f;
    s_yaw_pid.rate_pid.D = 0.0f;
    s_yaw_pid.rate_pid.IMax = 2500.0f;
    s_yaw_pid.rate_pid.OutMax = 15000.0f;
    s_yaw_pid.rate_pid.I_L = 1.0f;
    s_yaw_pid.rate_pid.I_U = 15.0f;

    s_pitch_pid.angle_pid.P = 18.0f;
    s_pitch_pid.angle_pid.I = 0.00f;
    s_pitch_pid.angle_pid.D = 0.0f;
    s_pitch_pid.angle_pid.OutMax = 220.0f;
    s_pitch_pid.angle_pid.DeadZone = 0.05f;

    s_pitch_pid.rate_pid.P = 90.0f;
    s_pitch_pid.rate_pid.I = 4.5f;
    s_pitch_pid.rate_pid.D = 0.0f;
    s_pitch_pid.rate_pid.IMax = 2500.0f;
    s_pitch_pid.rate_pid.OutMax = 15000.0f;
    s_pitch_pid.rate_pid.I_L = 1.0f;
    s_pitch_pid.rate_pid.I_U = 15.0f;

/***************************************************************************此处是PID调试地区**********************************************************************************/


    //进入已初始化模式
    s_gimbal.mode = GIMBAL_MODE_INIT;
}


//限幅函数
static float Gimbal_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}


//角度返回是0~360°，我在这里把它映射到-180~180
static float Gimbal_Wrap180(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}



//此处是云台自动化扫描函数
/*
HAL_GetTick() % period_ms：得到当前周期内时间
phase：归一化到 0~1
前半周期从 min_deg 线性走到 max_deg
后半周期再从 max_deg 线性走回 min_deg
*/

static float Gimbal_Sawtooth(uint32_t period_ms, float min_deg, float max_deg)
{
    float span = max_deg - min_deg;
    uint32_t t = HAL_GetTick() % period_ms;
    float phase = (float)t / (float)period_ms;
    if (phase < 0.5f)
    {
        return min_deg + span * (phase * 2.0f);
    }
    return max_deg - span * ((phase - 0.5f) * 2.0f);
}



/**********************************************************************此处为核心******************************************************************/
/*更新云台的状态估计（电机角度 + IMU姿态），计算出当前云台在世界坐标系下的姿态。*/
static void Gimbal_UpdateStateEstimate(void)
{

    /*获取传感器数据指针*/
    const GM6020_Motor_t *yaw_motor = GM6020_GetMotorByIndex(GM6020_YAW_INDEX);        //yaw电机反馈
    const GM6020_Motor_t *pitch_motor = GM6020_GetMotorByIndex(GM6020_PITCH_INDEX);    //PITCH电机反馈
    const Gyro_TypeDef *imu = INS_GetData();   //IMU数据

    //更新电机状态
    GM6020_Periodic1ms();  //可在6020.C文件中找到，这个是周期性检查6020电机是否处于离线状态
    s_gimbal.motor_ready = GM6020_AllOnline();  //判断电机是不是都在线
    s_gimbal.imu_ready = INS_IsReady();  //判断IMU是否可用

    //上电当前位置当作相对零点，不明白实验室的要求，是使用绝对零位还是相对零位
    //首次记录零位
    if ((yaw_motor != NULL) && (pitch_motor != NULL) && (s_zero_inited == 0U) && s_gimbal.motor_ready)
    {
        s_yaw_zero_deg = yaw_motor->total_angle_deg;
        s_pitch_zero_deg = pitch_motor->total_angle_deg;
        s_zero_inited = 1U;
    }

    //计算关节角度把累计角在转化成相对零位角
    if (yaw_motor != NULL)
    {
        s_gimbal.current_joint_yaw_deg = yaw_motor->total_angle_deg - s_yaw_zero_deg;
    }
    if (pitch_motor != NULL)
    {
        s_gimbal.current_joint_pitch_deg = pitch_motor->total_angle_deg - s_pitch_zero_deg;
    }

    //如果IMU正常
    if (s_gimbal.imu_ready != 0U)
    {
        s_gimbal.body_yaw_deg = imu->YAW;
        s_gimbal.body_pitch_deg = imu->PITCH;
        s_gimbal.body_roll_deg = imu->ROLL;
        s_gimbal.yaw_rate_dps = imu->GZ * 57.29578f;  //此处单位为rad/s
        s_gimbal.pitch_rate_dps = imu->GY * 57.29578f; // 此处单位也是rad/s
    }
    else //如果IMU不正常
    {
        s_gimbal.body_yaw_deg = 0.0f; //机体姿态角
        s_gimbal.body_pitch_deg = 0.0f;
        s_gimbal.body_roll_deg = 0.0f;
        /*此处估算参考了AI*/
        s_gimbal.yaw_rate_dps = (yaw_motor != NULL) ? (yaw_motor->speed_rpm * 6.0f) : 0.0f; //角速度由电机估算
        s_gimbal.pitch_rate_dps = (pitch_motor != NULL) ? (pitch_motor->speed_rpm * 6.0f) : 0.0f;
        /*此处参考了AI*/
    }


    /*此处参考了AI*/
    //计算世界坐标姿态
    //世界 yaw = 车体 yaw + 云台相对 yaw
    //世界 pitch = 车体 pitch + 云台相对 pitch
    s_gimbal.current_world_yaw_deg = Gimbal_Wrap180(s_gimbal.body_yaw_deg + s_gimbal.current_joint_yaw_deg);
    s_gimbal.current_world_pitch_deg = s_gimbal.body_pitch_deg + s_gimbal.current_joint_pitch_deg;
    /*此处参考了AI*/

    //发送给视觉
    USBVision_SetTelemetry(s_gimbal.current_world_yaw_deg,
                           s_gimbal.current_world_pitch_deg,
                           s_gimbal.body_roll_deg);
}



//模式实时更新以及管理
//决定当前云台应该工作在哪种模式。
static void Gimbal_UpdateMode(void)
{
    //先得到视觉目标
    const USBVision_Target_t *vision = USBVision_GetTarget();

    //视觉在线并且当前确实有目标可瞄准
    s_gimbal.vision_online = (vision->online != 0U) && (vision->aim_flag != 0U);

    //电机不在线进入安全模式
    if (s_gimbal.motor_ready == 0U)
    {
        s_gimbal.mode = GIMBAL_MODE_SAFE;
    }
    else if (s_zero_inited == 0U)//零位没有初始化，进入初始化模式
    {
        s_gimbal.mode = GIMBAL_MODE_INIT;
    }
    else if (s_gimbal.vision_online != 0U)//视觉有目标，进入自瞄模式
    {
        s_gimbal.mode = GIMBAL_MODE_AUTO_AIM;
    }
    else//视觉没目标，进入SENTRY即哨兵模式，进行扫描
    {
        s_gimbal.mode = GIMBAL_MODE_SENTRY;
    }
}


//目标更新
//负责计算云台应该转到哪里
static void Gimbal_UpdateTarget(void)
{
    const USBVision_Target_t *vision = USBVision_GetTarget();

    //模式划分


    //自瞄模式
    if (s_gimbal.mode == GIMBAL_MODE_AUTO_AIM)
    {
        //转到视觉传过来的目标欧拉角
        s_gimbal.target_world_yaw_deg = vision->yaw_world_deg;
        s_gimbal.target_world_pitch_deg = vision->pitch_world_deg;
    }
    else if (s_gimbal.mode == GIMBAL_MODE_SENTRY)//哨兵模式
    {
        //在限位区域内进行扫描  角度在后面进行调试  周期在.h文件里面调试
        s_gimbal.target_world_yaw_deg = Gimbal_Sawtooth(GIMBAL_SENTRY_YAW_PERIOD_MS, -60.0f, 60.0f);
        s_gimbal.target_world_pitch_deg = Gimbal_Sawtooth(GIMBAL_SENTRY_PITCH_PERIOD_MS, -8.0f, 10.0f);
    }
    else//其它模式  保持当前位置不动
    {
        s_gimbal.target_world_yaw_deg = s_gimbal.current_world_yaw_deg;
        s_gimbal.target_world_pitch_deg = s_gimbal.current_world_pitch_deg;
    }

    /*此处参考了中国科学技术大学robowalker战队的开源*/


    /* 视觉给的是世界坐标系欧拉角，这里用机体姿态做解耦补偿。
     * 这样底盘侧发生俯仰/横滚时，云台目标仍然保持在世界坐标目标上。
     */
    s_gimbal.target_joint_yaw_deg = Gimbal_Wrap180(s_gimbal.target_world_yaw_deg - s_gimbal.body_yaw_deg);
    s_gimbal.target_joint_pitch_deg = s_gimbal.target_world_pitch_deg - s_gimbal.body_pitch_deg;


    /*此处参考了中国科学技术大学robowalker战队的开源*/

    //下方的软件限位
    Gimbal_ApplySoftLimit();
}

//是要软限位的目的是永远不要到达机械极限
//就像要给彼此留一些距离qaq
static void Gimbal_ApplySoftLimit(void)
{
    float yaw_min = GIMBAL_YAW_MIN_DEG + GIMBAL_SOFT_LIMIT_MARGIN_DEG;
    float yaw_max = GIMBAL_YAW_MAX_DEG - GIMBAL_SOFT_LIMIT_MARGIN_DEG;
    float pitch_min = GIMBAL_PITCH_MIN_DEG + GIMBAL_SOFT_LIMIT_MARGIN_DEG;
    float pitch_max = GIMBAL_PITCH_MAX_DEG - GIMBAL_SOFT_LIMIT_MARGIN_DEG;

    s_gimbal.target_joint_yaw_deg = Gimbal_Clamp(s_gimbal.target_joint_yaw_deg, yaw_min, yaw_max);
    s_gimbal.target_joint_pitch_deg = Gimbal_Clamp(s_gimbal.target_joint_pitch_deg, pitch_min, pitch_max);
}


/*****************************************************************PID控制算法核心内容****************************************************************************************************/
static void Gimbal_RunController(void)
{
    /* 串级控制：外环先把角度误差转成目标角速度，
     * 内环再根据角速度误差输出最终电流。
     */
    float yaw_rate_ref;
    float pitch_rate_ref;

    s_yaw_pid.angle_pid.SetPoint = s_gimbal.target_joint_yaw_deg; //目标角度 - 当前角度 → 目标角速度
    s_yaw_pid.angle_pid.ActualValue = s_gimbal.current_joint_yaw_deg;//目标角速度 - 当前角速度 → 电机电压命令
    yaw_rate_ref = PID_Calc(&s_yaw_pid.angle_pid); //最终发送量

    s_pitch_pid.angle_pid.SetPoint = s_gimbal.target_joint_pitch_deg;//目标角度 - 当前角度 → 目标角速度
    s_pitch_pid.angle_pid.ActualValue = s_gimbal.current_joint_pitch_deg;//目标角速度 - 当前角速度 → 电机电压命令
    pitch_rate_ref = PID_Calc(&s_pitch_pid.angle_pid);//最终发送量

    //解释同上
    s_yaw_pid.rate_pid.SetPoint = yaw_rate_ref;
    s_yaw_pid.rate_pid.ActualValue = s_gimbal.yaw_rate_dps;
    s_gimbal.yaw_current_cmd = (int16_t)PID_Calc(&s_yaw_pid.rate_pid);

    s_pitch_pid.rate_pid.SetPoint = pitch_rate_ref;
    s_pitch_pid.rate_pid.ActualValue = s_gimbal.pitch_rate_dps;
    s_gimbal.pitch_current_cmd = (int16_t)PID_Calc(&s_pitch_pid.rate_pid);
}
/*****************************************************************PID控制算法核心内容****************************************************************************************************/



//真正发 CAN 之前，再做一层安全检查。
static void Gimbal_SendOutput(void)
{
    //如果电机不在线就不给电压了
    if (s_gimbal.mode == GIMBAL_MODE_SAFE)
    {
        s_gimbal.yaw_current_cmd = 0;
        s_gimbal.pitch_current_cmd = 0;
    }

    /**********************************************************************此处参考了AI****************************************************************/
    /* 靠近限位时进一步衰减输出，避免硬顶把线束拧死。 */
    if ((s_gimbal.current_joint_yaw_deg < (GIMBAL_YAW_MIN_DEG + 1.0f) && s_gimbal.yaw_current_cmd < 0) ||
        (s_gimbal.current_joint_yaw_deg > (GIMBAL_YAW_MAX_DEG - 1.0f) && s_gimbal.yaw_current_cmd > 0))
    {
        s_gimbal.yaw_current_cmd = 0;
    }

    if ((s_gimbal.current_joint_pitch_deg < (GIMBAL_PITCH_MIN_DEG + 1.0f) && s_gimbal.pitch_current_cmd < 0) ||
        (s_gimbal.current_joint_pitch_deg > (GIMBAL_PITCH_MAX_DEG - 1.0f) && s_gimbal.pitch_current_cmd > 0))
    {
        s_gimbal.pitch_current_cmd = 0;
    }
    /**********************************************************************此处参考了AI****************************************************************/

    //发送！！！！！！！！
    GM6020_SendCurrentsCAN1(s_gimbal.yaw_current_cmd, s_gimbal.pitch_current_cmd);
}


//1khz控制
void StartGimbal_Task(void *argument)
{
    (void)argument;
    uint32_t last_wake = osKernelGetTickCount();

    Gimbal_Init();

    for (;;)
    {
        Gimbal_UpdateStateEstimate();
        Gimbal_UpdateMode();
        Gimbal_UpdateTarget();
        Gimbal_RunController();
        Gimbal_SendOutput();
        osDelayUntil(last_wake + 1U);
        last_wake += 1U;
    }
}
