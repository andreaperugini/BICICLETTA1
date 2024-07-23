/*

 * DC_motor.h

 *

 *  Created on: Nov 22, 2023

 *      Author: giuby

 */

#ifndef INC_DC_MOTOR_H_
#define INC_DC_MOTOR_H_

#include <main.h>
#include <PID.h>

#define PI 3.14
#define V_MAX 10

float DegreeSec2RPM(float);
int Voltage2Direction(float);

float Voltage2Duty(float);
uint8_t Ref2Direction(float);
void set_PWM_and_dir(float, uint8_t);

#endif /* INC_DC_MOTOR_H_ */