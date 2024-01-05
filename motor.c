/*
 * motor.c
 *
 *  Created on: Jan 5, 2024
 *      Author: pskor
 */
#include "motor.h"
int predkosc=1050;

void changeVel(int v){
if(HAL_GPIO_ReadPin(GPIOC, MOTOR_L_Pin)==GPIO_PIN_SET){v+=50;HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);}
if(HAL_GPIO_ReadPin(GPIOC, MOTOR_R_Pin)==GPIO_PIN_SET){v-=50;}
HAL_GPIO_TogglePin(GPIOB, LD2_Pin);}

void stepCV(int steps, int delay)
{
//	delay=delay/1000;
	for(int x=0; x<steps; x++)
//	while(HAL_GPIO_ReadPin(GPIOC,  MOTOR_L_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOC,  MOTOR_R_Pin) != GPIO_PIN_SET)
		    {
//		  	  changeVel(delay);
		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET);   // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET); // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET); // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET);   // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET);   // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET); // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET);   // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET); // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET); // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET);   // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET);   // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET); // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET); // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET);   // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET); // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET); // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET);   // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);   // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET); // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET); // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);   // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET);   // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET); // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);   // IN4
		      microDelay(delay);
		    }
}

void stepCCV(int steps, int delay)
{
//	delay=delay/1000;
	for(int x=0; x<steps; x++)
//	while(HAL_GPIO_ReadPin(GPIOC,  MOTOR_R_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOC,  MOTOR_L_Pin) != GPIO_PIN_SET)
		    {
//		  	  changeVel(delay);
		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET);   // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET); // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET); // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET);   // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET);   // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET); // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET);   // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET); // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET);   // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET);   // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET); // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET); // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET);   // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET); // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET); // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET);   // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET);   // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET); // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET);   // IN4
		      microDelay(delay);

		      HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET);   // IN1
		      HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET); // IN2
		      HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
		      HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET);   // IN4
		      microDelay(delay);
		    }
}

