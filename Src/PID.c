/*

 * PID.c

 *

 *  Created on: Nov 22, 2023

 *      Author: giuby

 */
#include <PID.h>


void init_PID(PID* p, float Tc, float u_max, float u_min){
	p->Tc = Tc;
	p->u_max = u_max;
	p->u_min = u_min;
}

void tune_PID(PID* p, float Kp, float Ki, float Kd){
	p->Kp = Kp;
	p->Ki = Ki;
	p->Kd = Kd;
}

float PID_controller(PID* p, float measure, float y_ref){
	static float e_old = 0, Iterm = 0;
	float u;
	float newIterm;
	float e = measure - y_ref;
	float Pterm = p->Kp * e;

	newIterm = Iterm + (p->Ki)*p->Tc*e_old;
	float Dterm = (p->Kd/p->Tc)*(e - e_old);
	e_old = e;

	u = Pterm + newIterm + Dterm;

	if( u > p->u_max){
		u = p->u_max;
	} else if(u < p->u_min){
		u = p->u_min;
	} else{
		Iterm = newIterm;
	}

	return u;
}
