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

	p = kp * diff[1]; 						//���v�Z
	i = ki * integral;						//�ϕ�
	d = kd * (diff[1] - diff[0]) / delta_t;	//����

	ret = p + i + d;
	/* �㉺���l�`�F�b�N */

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
