/*
 * 文件说明：
 * 该文件实现本工程的 PID 与前馈控制器。
 * 其中 PID_Calc 同时兼容死区、梯形积分、误差窗积分、变速积分、不完全微分等策略，
 * 用于云台角度环/速度环、底盘速度环等闭环控制场景。
 */
//
// Created by lisil on 2026/3/1.
//


#include "main.h"
#include "PID.h"
#include <math.h>

char GimbalModule=1;

/*
*怎么用这版（

想要云台（变速积分 + 不完全微分）

设 ErrorMax = 0（关闭误差窗积分）

设 I_L / I_U 为你\原来的阈值

设 RC_DF 为 0~1 之间（比如 0.7）


想要 普通（误差窗积分 + 普通微分）

设 ErrorMax 为你原来的阈值

设 I_L=I_U=0（关闭变速积分）

设 RC_DF=0（关闭不完全微分
 */


/******************************
 * 函数名:PID_Calc
 * 功能说明:微分先行pid(暂未实现)+变积分pid+不完全微分pid+梯形积分pid+积分分离pid
 * 形参:PID_TypeDef结构体
 * 返回值:PID反馈计算的输出值
 * ************************** */
float PID_Calc(PID_t *P)
{
    /* 保存历史 */
    P->LastError = P->PreError;
    P->Last_DOut = P->Dout;

    /* 计算误差 */
    P->PreError = P->SetPoint - P->ActualValue;

    /* 死区 */
    if (ABS(P->PreError) <= P->DeadZone)
        P->PreError = 0.0f;

    /********************************************************************************
     * 比例项
     ********************************************************************************/
    P->Pout = P->P * P->PreError;

    /********************************************************************************
     * 积分项（通用：按配置选择策略）
     * 规则优先级：
     *  如果配置了 ErrorMax（>0），则做“误差窗积分”
     *  否则如果配置了 I_L/I_U（I_U>I_L>0），则做“变速积分/积分分离”
     *  否则默认正常积分
     ********************************************************************************/
    float i_inc = 0.0f;  /* 本次积分增量（梯形积分） */

    /* 1) 误差窗积分（类似 else 分支里的逻辑） */
    if (P->ErrorMax > 0.0f)
    {
        if (P->PreError > -P->ErrorMax && P->PreError < P->ErrorMax)
        {
            i_inc = (P->PreError + P->LastError) * 0.5f;
            P->I_Flag = 1;
        }
        else
        {
            /* 不积分 */
            P->I_Flag = 0;
        }
    }
    /* 2) 变速积分（类似 GimbalModule 分支里的逻辑） */
    else if (P->I_U > P->I_L && P->I_L > 0.0f)
    {
        float ae = ABS(P->PreError);

        if (ae < P->I_L)
        {
            i_inc = (P->PreError + P->LastError) * 0.5f;
        }
        else if (ae < P->I_U)
        {
            /* 权重从 1 线性减小到 0（注意用 |e| 做权重更合理） */
            float w = (P->I_U - ae) / (P->I_U - P->I_L);
            i_inc = (P->PreError + P->LastError) * 0.5f * w;
        }
        else
        {
            /* ae >= I_U 不积分 */
            i_inc = 0.0f;
        }
    }
    /* 3) 默认正常积分 */
    else
    {
        i_inc = (P->PreError + P->LastError) * 0.5f;
    }

    /* 积分累计 + 限幅 */
    if (i_inc != 0.0f)
    {
        P->SumError += i_inc;
        P->SumError = LIMIT_MAX_MIN(P->SumError, P->IMax, -P->IMax);
    }

    P->Iout = P->I * P->SumError;

    /********************************************************************************
     * 微分项（通用：按 RC_DF 决定是否做“不完全微分”）
     * - RC_DF 取值建议 0~1：
     *   RC_DF=0    纯当前微分（不过滤）
     *   RC_DF≈0.7  有一定平滑
     *   RC_DF=1    完全沿用上次（等于没微分，通常不要这样）
     ********************************************************************************/
    float d_raw = P->D * (P->Out - P->Last_Out);

    if (P->RC_DF > 0.0f && P->RC_DF < 1.0f)
    {
        P->Dout = d_raw * (1.0f - P->RC_DF) + P->Last_DOut * P->RC_DF;
    }
    else
    {
        P->Dout = d_raw;
    }

    /* 注意：Last_Out 更新顺序不要动（你原注释的点） */
    P->Last_Out = P->Out;

    /* 输出限幅 */
    P->Out = LIMIT_MAX_MIN(P->Pout + P->Iout + P->Dout, P->OutMax, -P->OutMax);

    return P->Out;
}


/******************************
 * 函数名:FeedForward_Calc
 * 功能说明:前馈PID
 * 形参:FeedForward_TypeDef结构体
 * 返回值:前馈PID反馈计算的输出值
 * ************************** */
float FeedForward_Calc(FeedForward_t *FF)
{
    FF->Out=FF->Pre_DeltIn*FF->K1+(FF->Pre_DeltIn-FF->Last_DeltIn)*FF->K2;
    FF->Last_DeltIn=FF->Pre_DeltIn;
    return LIMIT_MAX_MIN(FF->Out,FF->OutMax,-FF->OutMax);
}


/*
 (1) 计算 PID（误差闭环）
float u_pid = PID_Calc(&pid);

 (2) 计算前馈（基于指令变化）
ff.Pre_DeltIn = cmd - ff.LastCmd;    // Δin：指令变化量（建议用“目标值”）
float u_ff = FeedForward_Calc(&ff);
ff.LastCmd = cmd;                    // 你需要自己保存上一周期 cmd（结构体里加个 LastCmd 更方便）

 (3) 合成输出
float u = u_pid + u_ff;

 (4) 总限幅（建议总限幅做一次即可）
u = LIMIT_MAX_MIN(u, pid.OutMax, -pid.OutMax);
 好处：前馈负责“跟随性/响应速度”，PID 负责“稳态误差/扰动抑制”。
 */