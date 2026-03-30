#include "stm32f10x.h"

/*
 * TIM2 : 1kHz 进行内环速度控制
 * TIM3 : 100Hz 外环角度控制 + VOFA打印
 * TIM4 : 1Hz   角度目标 0/90deg
 */
void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;

    /* ---------------- TIM2 1kHz ---------------- */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 72 - 1;       
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1000 - 1;       
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);

    /* ---------------- TIM3 100Hz ---------------- */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 720 - 1;      
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1000 - 1;        
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &tim);

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM3, ENABLE);

    /* ---------------- TIM4 1Hz ---------------- */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = 7200 - 1;      
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 10000 - 1;        
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &tim);

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM4, ENABLE);
}
