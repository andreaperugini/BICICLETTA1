
/*
 * PID.c
 *
 *  Created on: Nov 20, 2023
 *      Author: andre
 */

#include <PID.h>

void init_PID (PID* p, float Tc, float u_max, float u_min){

	p->Tc = Tc;
	p->u_max = u_max;
	p->u_min = u_min;
	p->e_old=0;
	p->Iterm=0;

}


void tune_PID (PID* p, float Kp, float Ki, float Kd){

	p->Kp = Kp;
	p->Kd = Kd;
	p->Ki = Ki;
}

float PID_controller (PID* p, float y, float r){

	float u;
	float newIterm;
	float e = r-y;
	float Pterm = p-> Kp * e;

	newIterm = p->Iterm + (p->Ki)* p->Tc * p->e_old;
	p->e_old = e;
	u = Pterm + newIterm;




	if (u > p->u_max){
		u = p->u_max;
	}else if(u<p->u_min){
		u = p->u_min;
	}else
	{
		p->Iterm = newIterm;
	}

	return u;

}

