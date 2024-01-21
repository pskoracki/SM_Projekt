/*
 * pid.h
 *
 *  Created on: Jan 21, 2024
 *      Author: pskor
 */

#ifndef INC_PID_H_
#define INC_PID_H_
#include "main.h"

typedef struct {
    float kp;
    float ki;
    float kd;

    float y_ref;
    float error_prev;
    float error_i;
} PIDController;

void PID_Init(PIDController* pid, float kp, float ki, float kd, float y_ref);
float PID_Update(PIDController* pid, float y_ref2, float enkoder);


#endif /* INC_PID_H_ */
