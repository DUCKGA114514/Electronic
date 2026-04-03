#include "PID.h"

//限幅公式
static float clamp_f(float x, float mn, float mx)
{
    if (x < mn) return mn;
    if (x > mx) return mx;
    return x;
}

//static inline float clamp_f(float x, float mn, float mx)
//{
//    if (x < mn) return mn;
//    if (x > mx) return mx;
//    return x;
//}


//dt是采样周期1e-6f 是 安全保护：防止除以 0 防止数值爆炸

//d_tau —— 微分低通滤波时间常数

//PID计算思维参考了中科大的代码
void PID_Init(PID_t *pid,float kp, float ki, float kd,float out_min, float out_max,float dt, float d_tau)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->dt    = (dt    > 1e-6f) ? dt    : 1e-6f;
    pid->d_tau = (d_tau > 1e-6f) ? d_tau : 1e-6f;

    pid->integral = 0.0f;
    pid->prev_measure = 0.0f;
    pid->d_lpf = 0.0f;

    // integral_max = 0.7*out_max/ki（ki=0 不积分）,此公式参考网络资料
	//1e_6f是安全保护，参考了中科大24哨兵开源的保护，但在这道题目上加不加都可以
    if (pid->ki > 1e-6f)
    {
        float out_abs = (out_max >= 0.0f) ? out_max : -out_max;
        pid->integral_max = 0.7f * out_abs / pid->ki;
    }
    else
    {
        pid->integral_max = 0.0f;
    }
}

//PID重置，重初始化
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_measure = 0.0f;
    pid->d_lpf = 0.0f;
}

//PID计算更新
float PID_Update(PID_t *pid, float target, float measure)
{
    const float err = target - measure;

    // 微分：对测量值求导
    const float d_meas = (measure - pid->prev_measure) / pid->dt;
    pid->prev_measure = measure;

	
    // 一阶低通滤波：d_lpf = d_lpf + alpha*(d_meas - d_lpf)
	//此处参考了网络上一阶低通滤波的资料，以及MPU6050姿态解算的互补滤波
    const float alpha = pid->dt / (pid->d_tau + pid->dt);
    pid->d_lpf = pid->d_lpf + alpha * (d_meas - pid->d_lpf);

    const float p = pid->kp * err;
    const float d = -pid->kd * pid->d_lpf; 

    // 不含积分的输出
    float u_no_i = p + d;

    // 先看看加上积分会不会饱和
    float u = u_no_i + pid->ki * pid->integral;


    // 抗积分饱和,此处参考中科大教程代码逻辑
    int allow_i = 1;
    if (u > pid->out_max && err > 0) allow_i = 0;
    if (u < pid->out_min && err < 0) allow_i = 0;

    if (allow_i && pid->ki > 1e-6f)
    {
        pid->integral += err * pid->dt;

        if (pid->integral_max > 0.0f)
            pid->integral = clamp_f(pid->integral, -pid->integral_max, pid->integral_max);
    }
	

    // 最终输出(要加上限幅喵)
    u = u_no_i + pid->ki * pid->integral;
    u = clamp_f(u, pid->out_min, pid->out_max);
	
    return u;
}
