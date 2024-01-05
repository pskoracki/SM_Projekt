/*
 * motor.h
 *
 *  Created on: Jan 5, 2024
 *      Author: pskor
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include "main.h"

extern int predkosc;

#define TRIG_PIN GPIO_PIN_11
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_10
#define ECHO_PORT GPIOB

//void microDelay (uint16_t delay);
void changeVel(int v);
void stepCV(int steps, int delay);
void stepCCV(int steps, int delay);


#endif /* INC_MOTOR_H_ */
