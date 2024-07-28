/*

 * DC_motor.c

 *

 *  Created on: Nov 22, 2023

 *      Author: giuby

 */
#include "DC_motor.h"
#include "main.h"
#include "stdint.h"

float DegreeSec2RPM(float speed_degsec){
	float speed_rpm = (speed_degsec * 60)/360;
	return speed_rpm;
}

float Voltage2Duty(float u){
	if(u <= 0){
		u = -u;
	}
	float duty = 100 * u/V_MAX;

	if (duty > 100){
		duty = 100;
	}else if(duty < 0){
		duty = 0;
	}
	return duty;
}

uint8_t Ref2Direction(float y_ref){
	uint8_t direction;
	if(y_ref >= 0){
		direction = 0;
	} else {
		direction = 1;
	}
	return direction;
}

void set_PWM_and_dir_back_wheel(float duty, uint8_t direction){
	TIM1 ->CCR1 = (duty/100.0)*TIM1->ARR;

	//ARR:valore massimo che il contatore può raggiungere


	//Qui non ho capito a che serve
	if(direction == 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}else if(direction == 1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}
}

void set_PWM_and_dir_front_wheel (float duty, uint8_t dir){
	TIM3 -> CCR1 = (duty/100)*TIM3->ARR;

	if (dir == 0){
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}else if (dir == 1){
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
}
