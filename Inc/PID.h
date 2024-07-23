/*

 * PID.h

 *

 *  Created on: Nov 22, 2023

 *      Author: giuby

 */



#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct PID{
	float Kp;
	float Ki;
	float Kd;
	float Tc;
	float u_max;
	float u_min;
}PID;

void init_PID(PID*, float, float, float);
void tune_PID(PID*, float, float, float);
float PID_controller(PID*, float, float);

#endif /* INC_PID_H_ */
