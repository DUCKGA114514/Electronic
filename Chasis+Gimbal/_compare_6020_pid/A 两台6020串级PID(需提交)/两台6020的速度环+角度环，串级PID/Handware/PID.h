#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "stm32f10x.h"

/*
 * 速度环 PID：输出 = 电压指令（v_cmd）
 * 改动点：
 * 1) 引入 dt（1kHz => 0.001f）
 * 2) 微分对测量值（避免目标突变导致 D 冲击）
 * 3) 微分一阶低通滤波（d_tau）
 * 4) 抗积分饱和（输出饱和且误差继续推向饱和时，停止积分）
 * 5) 合理积分限幅：integral_max = 0.7*out_max/ki（ki=0 则不积分）
 */

typedef struct
{
    float kp;
    float ki;
    float kd;

    float dt;            // 控制周期(s)，例如 0.001
    float d_tau;         // 微分低通时间常数(s)，建议 0.01

    float integral;
    float integral_max;

    float prev_measure;  // 上一次测量值
    float d_lpf;         // 低通后的测量微分

    float out_min;
    float out_max;
} PID_t;

void  PID_Init(PID_t *pid,
               float kp, float ki, float kd,
               float out_min, float out_max,
               float dt, float d_tau);

void  PID_Reset(PID_t *pid);

// 返回：限幅后的输出（电压指令）
float PID_Update(PID_t *pid, float target, float measure);

#endif
