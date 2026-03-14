/* PID/前馈控制对外接口。 */
//
// Created by lisil on 2026/3/1.
//

#ifndef GIMBAL_PID_H
#define GIMBAL_PID_H

/**
 * 增大K1能使控制器输出迅速地响应当前输入的变化，若设置过大，会导致系统震荡或不稳定
 * 增大K2能使控制器对当前输入变化更加敏感(即输入的变化快慢),若增益过大，会引起输出的不必要剧烈波动，导致系统不稳定
 *
 */

#include "reg.h"

float PID_Calc(PID_t *P);
float FeedForward_Calc(FeedForward_t *FF);

#endif //GIMBAL_PID_H