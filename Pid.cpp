/*
 * pid.cpp
 *
 *      Author: shikama
 */

#include "Pid.h"

Pid::Pid(void) {
	delta_t = DELTA_T;
	integral = 0;
	diff[1] = 0;
}

float Pid::input(int error)
{
	float ret;

	diff[0] = diff[1];
	diff[1] = error;
	integral += ((diff[1] + diff[0])/ 2.0) * delta_t;

	p = kp * diff[1]; 						//比例計算
	i = ki * integral;						//積分
	d = kd * (diff[1] - diff[0]) / delta_t;	//微分

	ret = p + i + d;
	/* 上下限値チェック */

	if (ret > 100.) {
		ret = 100.;
	} else if (ret < -100.) {
		ret = -100.;
	}

	return ret;
}

void Pid::set_params(float p, float d, float i) {
	kp = p;
	kd = d;
	ki = i;
}

void Pid::reset() {
	diff[0] = 0;
	diff[1] = 0;
	integral = 0;
}
