/* 低通滤波接口声明：用于对高频抖动信号进行平滑处理。 */
//
// Created by lisil on 2026/3/1.
//

#ifndef GIMBAL_FIR_H
#define GIMBAL_FIR_H

float LowPass(float Sample_Pre,float OutPut_Last,int Mode);

#endif //GIMBAL_FIR_H