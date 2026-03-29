/*
 * 文件说明：
 * 底盘 1ms 控制任务。
 * 主要流程：读取遥控器/姿态状态 -> 更新模式机 -> 逆解四轮目标转速 -> 速度 PI 输出电流 -> 通过 CAN 下发给 3508 电机。
 */
#include "Chasis.h"

#include <math.h>
#include <string.h>

#include "can.h"
#include "DT7.h"
#include "GM3508.h"
#include "INS.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 兼容文件 */
#ifndef CLAMP
#define CLAMP(x, low, high)  (((x) < (low)) ? (low) : (((x) > (high)) ? (high) : (x)))
#endif

/* s_chassis：底盘总控状态，保存模式、目标、保护状态和最终输出。 */
static ChassisControl_t s_chassis;
/* s_wheel_pid：四个轮子的独立速度 PI 控制器参数和中间量。 */
static ChassisWheelPid_t s_wheel_pid[CHASSIS_WHEEL_NUM];

/* s_can_started：CAN 初始化只执行一次的标志位,思路与云台的相同，避免重复配置滤波器。 */
static uint8_t s_can_started = 0U;

/* ========================= 私有函数声明 ========================= */
static void Chassis_Init(void);
static void Chassis_CANInitOnce(void);
static void Chassis_ProcessCanQueue(void);
static void Chassis_UpdateMode(void);
static void Chassis_UpdateCommandFromRemote(void);
static void Chassis_UpdateSafetyState(void);
static void Chassis_CalcInverseKinematics(void);
static void Chassis_SolveWheelCurrent(void);
static void Chassis_SendOutput(void);
static void Chassis_StopAll(void);
static float Chassis_ApplyDeadband(float value, float deadband);
static float Chassis_ApplyRamp(float current, float target, float step);
static float Chassis_RemoteToSpeed(int16_t channel, float max_speed);
static float Chassis_RemoteToYawRate(int16_t channel, float max_rate);
static float Chassis_WheelPidCalc(ChassisWheelPid_t *pid, float ref_rpm, float fdb_rpm);


//底盘初始化
static void Chassis_Init(void)
{
    //初始为0
    memset(&s_chassis, 0, sizeof(s_chassis));
    memset(s_wheel_pid, 0, sizeof(s_wheel_pid));

    //模式设置，初始化时调到安全模式
    s_chassis.mode = CHASSIS_MODE_SAFE;
    s_chassis.last_mode = CHASSIS_MODE_SAFE;

    /* 四个轮子参数先定为一致，后期若装配差异明显，可单独整定 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        s_wheel_pid[i].kp = 11.0f;
        s_wheel_pid[i].ki = 0.12f;
        s_wheel_pid[i].out_limit = CHASSIS_CURRENT_MAX;
        s_wheel_pid[i].i_limit = 5000.0f;
    }

    //3508初始化
    GM3508_Init();
    Chassis_CANInitOnce();
}

//CAN初始化配置
static void Chassis_CANInitOnce(void)
{
    if (s_can_started != 0U)
    {
        return;
    }

    CAN_FilterTypeDef can1_filter = {0};

    /* CAN2：底盘 3508 反馈接收 */
    //此处先全开放，后续需按实际情况调试
    can1_filter.FilterActivation = ENABLE;
    can1_filter.FilterBank = 1;
    can1_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can1_filter.FilterIdHigh = 0x0000;
    can1_filter.FilterIdLow = 0x0000;
    can1_filter.FilterMaskIdHigh = 0x0000;
    can1_filter.FilterMaskIdLow = 0x0000;
    can1_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can1_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can1_filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_READY)
    {
        (void)HAL_CAN_ConfigFilter(&hcan1, &can1_filter);
        (void)HAL_CAN_Start(&hcan1);
        (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

    s_can_started = 1U;
}

static void Chassis_ProcessCanQueue(void)
{
    /*
     * 底盘任务仅保留接口，后续此处可以再补充优化
     */
}

//死区函数，不知道用不用得到，因为我常用手柄，知道手柄不设置死区会导致漂移
//所以认为遥控器可能也有这个风险
static float Chassis_ApplyDeadband(float value, float deadband)
{
    if (fabsf(value) < deadband)
    {
        return 0.0f;
    }
    return value;
}


/*************************************************************此处参考了rm给的官方文件例程*******************************************************/
/* 斜坡函数：限制命令每周期的变化量，避免遥控器突变导致底盘瞬间冲击。 */
//我的理解是  比如目标速度从 0 突然变到 2m/s，斜坡不会一下跳过去，而是每 1ms 只增加一点点。
static float Chassis_ApplyRamp(float current, float target, float step)
{
    if (current < target)
    {
        current += step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= step;
        if (current < target)
        {
            current = target;
        }
    }
    return current;
}
/*************************************************************此处参考了rm给的官方文件例程*******************************************************/

//遥控器参数  --->   实际速度
static float Chassis_RemoteToSpeed(int16_t channel, float max_speed)
{
    float value = Chassis_ApplyDeadband((float)channel, CHASSIS_REMOTE_DEADBAND);
    return value / (float)DT7_CHANNEL_MAX_ABS * max_speed;
}

//遥控器参数 ---> 角速度
static float Chassis_RemoteToYawRate(int16_t channel, float max_rate)
{
    float value = Chassis_ApplyDeadband((float)channel, CHASSIS_REMOTE_DEADBAND);
    return value / (float)DT7_CHANNEL_MAX_ABS * max_rate;
}

//更新底盘状态，由S1拨杆决定
static void Chassis_UpdateMode(void)
{
    const DT7_State_t *rc = DT7_GetState();

    s_chassis.remote_online = DT7_IsOnline();
    s_chassis.motor_online = GM3508_AllOnline();

    if ((s_chassis.remote_online == 0U) || (rc->valid == 0U))
    {
        s_chassis.mode = CHASSIS_MODE_SAFE;
        return;
    }

    /* S1：底盘工作模式
     * 上：安全停机
     * 中：常规遥控
     * 下：小陀螺
     */
    if (rc->s1 == DT7_SWITCH_UP)
    {
        s_chassis.mode = CHASSIS_MODE_SAFE;
    }
    else if (rc->s1 == DT7_SWITCH_MID)
    {
        s_chassis.mode = CHASSIS_MODE_REMOTE;
    }
    else
    {
        s_chassis.mode = CHASSIS_MODE_SPIN;
    }
    /* S1 拨杆损坏：强制固定为遥控模式。 */
    s_chassis.mode = CHASSIS_MODE_REMOTE;
}

//安全保护
static void Chassis_UpdateSafetyState(void)
{
    s_chassis.emergency_stop = 0U;

    if (s_chassis.remote_online == 0U)
    {
        s_chassis.emergency_stop = 1U;
    }

    /* 底盘首次上电阶段，电机尚未回报时不允许输出，避免误驱动 */
    if (s_chassis.motor_online == 0U)
    {
        s_chassis.emergency_stop = 1U;
    }
/**************************************此处参考AI********************************************************************/
    /* 倾覆保护接口：目前工程未完整接入 INS 时，该逻辑默认不触发。
     * 一旦后续 INS 任务写入 pitch/roll 和 imu_valid，即可直接生效。
     * 这里采用“极限保护”而不是“轻微倾斜即停机”，满足赛场斜坡/碰撞后继续可跑的要求。
     */
    if ((s_chassis.imu_valid != 0U) &&
        ((fabsf(s_chassis.pitch_deg) > CHASSIS_TILT_LIMIT_DEG) ||
         (fabsf(s_chassis.roll_deg) > CHASSIS_TILT_LIMIT_DEG)))
    {
        s_chassis.emergency_stop = 1U;
    }
/**************************************此处参考AI********************************************************************/
    if (s_chassis.mode == CHASSIS_MODE_SAFE)
    {
        s_chassis.emergency_stop = 1U;
    }
}

//速度挡位切换
static void Chassis_UpdateCommandFromRemote(void)
{
    const DT7_State_t *rc = DT7_GetState();
    float max_vx = 1.2f;
    float max_vy = 1.2f;
    float max_wz = 4.5f;

    /* S2：速度挡位
     * 上：低速精细控车
     * 中：常规速度
     * 下：高速
     */
    //其实就是调整遥控器的最大速度最低速度限制
    if (rc->s2 == DT7_SWITCH_UP)
    {
        max_vx = 0.9f;
        max_vy = 0.9f;
        max_wz = 3.0f;
    }
    else if (rc->s2 == DT7_SWITCH_MID)
    {
        max_vx = 1.5f;
        max_vy = 1.5f;
        max_wz = 5.0f;
    }
    else
    {
        max_vx = 2.2f;
        max_vy = 2.0f;
        max_wz = 8.0f;
    }
    /* S2 拨杆损坏：强制固定为中挡。 */
    max_vx = 1.5f;
    max_vy = 1.5f;
    max_wz = 5.0f;


    //小车目标速度
    float target_vx = Chassis_RemoteToSpeed(rc->ch[1], max_vx);
    float target_vy = Chassis_RemoteToSpeed(rc->ch[0], max_vy);
    float target_wz = Chassis_RemoteToYawRate(rc->ch[2], max_wz);

    //ch[1] 前后  CH[0]左右 ch[2] 旋转




    /* 每毫秒最大变化量，可兼顾响应与稳定 */
    //把目标输入变成平滑命令，而不是直接瞬间跳变。
    //仿照大疆官方给的斜坡函数
    s_chassis.cmd.vx_mps = Chassis_ApplyRamp(s_chassis.cmd.vx_mps, target_vx, 0.0060f);
    s_chassis.cmd.vy_mps = Chassis_ApplyRamp(s_chassis.cmd.vy_mps, target_vy, 0.0060f);
    s_chassis.cmd.wz_radps = Chassis_ApplyRamp(s_chassis.cmd.wz_radps, target_wz, 0.0200f);
    s_chassis.cmd.brake = 0U;

/****************************************************此处参考了华南虎战队、robowalker战队步兵开源********************************************/
    /* 实战功能 2：小陀螺模式
     * 保留平移输入，同时给定基础自旋角速度，让步兵近战更难被锁。
     */
    if (s_chassis.mode == CHASSIS_MODE_SPIN)
    {
        float spin_base = (rc->s2 == DT7_SWITCH_DOWN) ? 10.0f : 7.0f;
        /* S2 拨杆损坏：小陀螺基础角速度固定。 */
        spin_base = 7.0f;
        if (rc->ch[2] >= 0)
        {
            target_wz = spin_base + Chassis_RemoteToYawRate(rc->ch[2], 2.0f);
        }
        else
        {
            target_wz = -spin_base + Chassis_RemoteToYawRate(rc->ch[2], 2.0f);
        }
        s_chassis.cmd.wz_radps = Chassis_ApplyRamp(s_chassis.cmd.wz_radps, target_wz, 0.0300f);
    }
/****************************************************此处参考了华南虎战队、robowalker战队步兵开源********************************************/

/**************************************此处参考AI********************************************************************/
    /* 实战功能 3：回中即柔性刹车，防止松杆后继续飘车 */
    if ((fabsf(target_vx) < 0.02f) && (fabsf(target_vy) < 0.02f) && (fabsf(target_wz) < 0.05f))
    {
        s_chassis.cmd.brake = 1U;
    }
}
/**************************************此处参考AI********************************************************************/



static void Chassis_CalcInverseKinematics(void)
{
    /* 电机轴 rpm = 轮组线速度 / 轮周长 * 60 * 减速比 */
    const float wheel_circ = 2.0f * (float)M_PI * CHASSIS_WHEEL_RADIUS_M;
    const float meter_per_sec_to_rpm = 60.0f / wheel_circ * CHASSIS_MOTOR_REDUCTION_RATIO;
    const float vx = s_chassis.cmd.vx_mps;
    const float vy = s_chassis.cmd.vy_mps;
    const float proj = CHASSIS_OMNI_PROJECTION_GAIN;
    const float wz_term = CHASSIS_OMNI_ROTATION_RADIUS_M * s_chassis.cmd.wz_radps;

    /* X 型四全向轮逆解
     * 已知机器人的运动状态和位姿，求机器人各个机构的运动状态
     * 就是我们已经知道了机器人想要的 vx、vy 和 wz，求每个轮子的目标转速。
     * 轮序：0 前左，1 前右，2 后左，3 后右  需要按照实际电机的ID进行调试
     * 轮速由车体平移速度在各轮驱动方向上的投影和自旋项共同组成。
     * cos(45°) = sin(45°) = 0.7071067812，投影系数为 1/sqrt(2)，即 CHASSIS_OMNI_PROJECTION_GAIN。
     * 自旋项由车体绕 Z 轴转动速度决定的，wz_term 为车体绕 Z 轴转动速度乘以转动半径。
     * meter_per_sec_to_rpm 单位换算，60/轮周长 * 减速比。
     */
    s_chassis.target_wheel_rpm[0] = (proj * (vx - vy) - wz_term) * meter_per_sec_to_rpm;
    s_chassis.target_wheel_rpm[1] = (proj * (vx + vy) + wz_term) * meter_per_sec_to_rpm;
    s_chassis.target_wheel_rpm[2] = (proj * (vx + vy) - wz_term) * meter_per_sec_to_rpm;
    s_chassis.target_wheel_rpm[3] = (proj * (vx - vy) + wz_term) * meter_per_sec_to_rpm;

    /* 防止任意单轮目标越界，保持方向不变整体缩放 */
    /*直白来说就是如果某一个轮子的目标转速超了上限，那就整体按比例缩小四轮速度，保持运动方向不变*/
    /*可以避免一个轮子拖着整车跑的情况，节省电力*/
    float max_abs = 1.0f;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        float abs_val = fabsf(s_chassis.target_wheel_rpm[i]);
        if (abs_val > max_abs)
        {
            max_abs = abs_val;
        }
    }

    if (max_abs > CHASSIS_WHEEL_RPM_MAX)
    {
        float scale = CHASSIS_WHEEL_RPM_MAX / max_abs;
        for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
        {
            s_chassis.target_wheel_rpm[i] *= scale;
        }
    }
}


//底盘PID控制   单个轮子的速度环
//这个环由P主导
static float Chassis_WheelPidCalc(ChassisWheelPid_t *pid, float ref_rpm, float fdb_rpm)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->ref_rpm = ref_rpm;
    pid->fdb_rpm = fdb_rpm;
    pid->err = pid->ref_rpm - pid->fdb_rpm;

    pid->integral += pid->err * CHASSIS_DT_S;
    pid->integral = CLAMP(pid->integral, -pid->i_limit, pid->i_limit);

    pid->out = pid->kp * pid->err + pid->ki * pid->integral;
    pid->out = CLAMP(pid->out, -pid->out_limit, pid->out_limit);
    return pid->out;
}

/******************************************************此处参考华南虎、文华战队步兵开源代码******************************************************/
//根据目标转速和反馈转速，用 PI 求每个轮子的输出电流。
static void Chassis_SolveWheelCurrent(void)
{
    const GM3508_Motor_t *motors = GM3508_GetMotors();
    float total_abs_current = 0.0f;

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        float ref = s_chassis.target_wheel_rpm[i];
        float fdb = (float)motors[i].speed_rpm;

        /* 柔性刹车策略：松杆时将参考速度快速收回 0，提高站停稳定性 */
        if (s_chassis.cmd.brake != 0U)
        {
            ref *= 0.25f;
        }

        float out = Chassis_WheelPidCalc(&s_wheel_pid[i], ref, fdb);
        s_chassis.motor_current[i] = (int16_t)CLAMP(out, -CHASSIS_CURRENT_MAX, CHASSIS_CURRENT_MAX);
        total_abs_current += fabsf((float)s_chassis.motor_current[i]);
    }

    /**************************************此处参考AI********************************************************************/
    /* 实战功能 4：总电流限幅，降低急变向/撞击恢复时的总线冲击 */
    if (total_abs_current > CHASSIS_TOTAL_CURRENT_MAX)
    {
        float scale = CHASSIS_TOTAL_CURRENT_MAX / total_abs_current;
        for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
        {
            s_chassis.motor_current[i] = (int16_t)((float)s_chassis.motor_current[i] * scale);
        }
    }
    /**************************************此处参考AI********************************************************************/
}
/******************************************************此处参考华南虎、文华战队步兵开源代码******************************************************/


//急停与发送
static void Chassis_SendOutput(void)
{
    //如果急停标志位要急停
    if (s_chassis.emergency_stop != 0U)
    {
        Chassis_StopAll();
        return;
    }
//否则发送
    GM3508_SendCurrentCAN1(s_chassis.motor_current);
}


//急停，用于保护
static void Chassis_StopAll(void)
{
    memset(s_chassis.target_wheel_rpm, 0, sizeof(s_chassis.target_wheel_rpm));
    memset(s_chassis.motor_current, 0, sizeof(s_chassis.motor_current));
    s_chassis.cmd.vx_mps = 0.0f;
    s_chassis.cmd.vy_mps = 0.0f;
    s_chassis.cmd.wz_radps = 0.0f;
    GM3508_SendCurrentCAN1(s_chassis.motor_current);
}


//1khz控制
void StartChassis_Task(void *argument)
{
    (void)argument;

    Chassis_Init();
    uint32_t tick = osKernelGetTickCount();

    for (;;)
    {
        /* 1kHz 主控制循环：
         *  先收消息
         *  再更新模式/安全状态
         *  最后求解和下发
         */
        Chassis_ProcessCanQueue();
        GM3508_Periodic1ms();

        /* 读取 INS 姿态接口：当前版本若 INS 未就绪，则安全逻辑自动忽略 */
        s_chassis.imu_valid = INS_IsReady();
        if (s_chassis.imu_valid != 0U)
        {
            const Gyro_TypeDef *imu = INS_GetData();
            s_chassis.pitch_deg = imu->PITCH;
            s_chassis.roll_deg = imu->ROLL;
        }

        Chassis_UpdateMode();
        Chassis_UpdateSafetyState();

        if (s_chassis.emergency_stop == 0U)
        {
            Chassis_UpdateCommandFromRemote();
            Chassis_CalcInverseKinematics();
            Chassis_SolveWheelCurrent();
            Chassis_SendOutput();
        }
        else
        {
            Chassis_StopAll();
        }

        s_chassis.last_mode = s_chassis.mode;
        tick += 1U;
        osDelayUntil(tick);
    }
}
