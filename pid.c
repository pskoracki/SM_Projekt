/*
 * pid.c
 *
 *  Created on: Jan 21, 2024
 *      Author: pskor
 */
#include "pid.h"

void PID_Init(PIDController* pid, float kp, float ki, float kd, float y_ref) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->y_ref = y_ref;
    pid->error_prev = 0;
    pid->error_i = 0;
}

float PID_Update(PIDController* pid, float y_ref2, float enkoder) {

    float error = y_ref2 - enkoder;

    float pid_p = pid->kp * error;
    float pid_i = pid->ki * pid->error_i;
    float pid_d = pid->kd * (error - (pid->error_prev));

    float output = pid_p + pid_i + pid_d;

    pid->error_prev = error;
    pid->error_i += error;

    return output;
}
