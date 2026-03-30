#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "CAN.h"
#include "PID.h"
#include "UART1_DMA_Tx.h"



/* 两台 GM6020：ID=3 和 7 */
#define MOTOR_NUM 2 // 区分电机

volatile int16_t  gm6020_speed_rpm[MOTOR_NUM] = {0};
volatile uint16_t gm6020_angle_raw[MOTOR_NUM] = {0};


//结构体+数组，将两个电机的双环分割开来，避免数据冲突

static PID_t pid_pos[MOTOR_NUM];   // 角度环：输出=速度目标
static PID_t pid_spd[MOTOR_NUM];  // 速度环：输出=电压指令

/* 内外环变量 */
volatile float   spd_target_rpm[MOTOR_NUM] = {0.0f};   // 内环速度目标（由角度环计算得出）
volatile int16_t v_cmd_out[MOTOR_NUM]      = {0};      // 最终输出（发送到 GM6020）

/* 角度目标：0 <-> 90deg，每 1s 切换 */
static volatile float target_angle_deg = 0.0f;


#define GM6020_ANGLE_MAX   8192.0f // 角度范围0~8192

//单位转换 raw - > reg   其实就是把 0~8192映射到0~360,第七次作业有训练过
static  float raw_to_deg(uint16_t raw)
{
    return ((float)raw) * 360.0f / GM6020_ANGLE_MAX;
}
static  float deg_to_raw(float deg)
{
    return deg * GM6020_ANGLE_MAX / 360.0f;
}


/* 防扣圈思路在培训时候有讲过，防止目标和实际误差过大 ,但限制目标值其实也并不需要这个来做，但我还是写了一下  */
static  float angle_err_wrap(float target_raw, float fdb_raw)
{
    float err = target_raw - fdb_raw;
    if (err >  (GM6020_ANGLE_MAX / 2.0f)) err -= GM6020_ANGLE_MAX;
    if (err < -(GM6020_ANGLE_MAX / 2.0f)) err += GM6020_ANGLE_MAX;
    return err;
}

int main(void)
{
    Timer_Init();
    MyCAN_Init();

    UART1_DMA_Tx_Init(115200);

    //此处可以调参
    /* 速度环：1kHz；角度环：100Hz */
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        PID_Init(&pid_spd[i],
                 22.0f, 0.0f, 0.00f,          // kp ki kd
                 -7500.0f, 7500.0f,           // out_min out_max（电压限幅）
                 0.001f, 0.01f);              // dt=1ms, d_tau=10ms

        PID_Init(&pid_pos[i],
		1.0f, 0.0f, 0.0f,            // kp ki kd（角度环一般不用给I和D吧？反正我给了是有点难调）
                 -200.0f, 200.0f,             // out_min out_max（速度限幅）
                 0.01f, 0.0f);                // dt=10ms, d_tau=0
    }

    while (1)
    {


    }
}


void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx;

    if (CAN_MessagePending(CAN1, CAN_FIFO0) != 0)
    {
        CAN_Receive(CAN1, CAN_FIFO0, &rx);

        if (rx.DLC == 8)
        {
            if (rx.StdId == 0x207)   // ID=3
            {
                gm6020_angle_raw[0] = (uint16_t)((rx.Data[0] << 8) | rx.Data[1]);
                gm6020_speed_rpm[0] = (int16_t)((rx.Data[2] << 8) | rx.Data[3]);
            }
			//这里之前犯蠢了，写成了0x211，然后突然发现是16进制，要写成20B
            else if (rx.StdId == 0x20B) // ID=7
            {
                gm6020_angle_raw[1] = (uint16_t)((rx.Data[0] << 8) | rx.Data[1]);
                gm6020_speed_rpm[1] = (int16_t)((rx.Data[2] << 8) | rx.Data[3]);
            }
        }
}	
}

//1S,切换一次角度目标值，你也可以切换目标，限位是-60~60,我这里贪方便改成了0~120，也可以修改，会麻烦一点
//切换速度可以更改TIM4数值来得到
//如果不一样，希望可以指正我，我可以修改

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

        target_angle_deg = (target_angle_deg < 1.0f) ? 90.0f : 0.0f;
    }
}

//PID角度环计算，100hz计算
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        /* 外环：角度 -> 速度目标（两台电机分别计算） */
        float target_raw = deg_to_raw(target_angle_deg);

        for (int i = 0; i < MOTOR_NUM; i++)
        {
            float fdb_raw = (float)gm6020_angle_raw[i];

            float err = angle_err_wrap(target_raw, fdb_raw);
            float fake_measure = target_raw - err;

            spd_target_rpm[i] = PID_Update(&pid_pos[i], target_raw, fake_measure);
        }

        /* 串口输出：目标角度、两台电机角度/速度目标/实际速度 ,两个速度主要是用来调速度环用，不看也行*/
        UART1_DMA_Tx_Printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                            target_angle_deg,
                            raw_to_deg(gm6020_angle_raw[0]),
                            spd_target_rpm[0],
                            (float)gm6020_speed_rpm[0],
                            raw_to_deg(gm6020_angle_raw[1]),
                            spd_target_rpm[1],
                            (float)gm6020_speed_rpm[1]);
    }
}

/* TIM2：1kHz 内环速度 PID */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        /* 内环：速度 PID（两台电机分别计算并发送电压指令） */
        for (int i = 0; i < MOTOR_NUM; i++)
        {
            float out = PID_Update(&pid_spd[i], spd_target_rpm[i], (float)gm6020_speed_rpm[i]);
            v_cmd_out[i] = (int16_t)out;

            GM6020_SendVoltage_ID3_ID7(v_cmd_out[1],v_cmd_out[0]);
        }
    }
}

