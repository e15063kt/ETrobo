/*
 * Evlog.h
 *
 *  Created on: 2015/06/07
 *      Author: shikama
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef EVLOG_H_
#define EVLOG_H_

#include "ev3api.h"
#include "Clock.h"
#include "ColorSensor.h"
#include "SonarSensor.h"
#include "GyroSensor.h"
#include "Motor.h"

#define MAX 100		/* 最大ログデータ数 */

struct log_data {
	int time;
	int light;
	int sonar;
	int gyro;
	int tail;
	int lmtr;
	int rmtr;
	int x[4];
};

class EvLog {
private:
	struct log_data data[MAX];
	int wcount;			/* total number of stored bytes */
	int rcount;
	int total_count;
	int full;
	int iiflag;
	int ioflag;
	int eflag;
	int start_time;
	FILE* fp;
	char fname[32];
	ev3api::Clock*			mClock;
	ev3api::ColorSensor* 	mColorSensor;
	ev3api::SonarSensor*	mSonarSensor;
	ev3api::GyroSensor*		mGyroSensor;
	ev3api::Motor*			mTail;
	ev3api::Motor* 			mLeftWheel;
	ev3api::Motor* 			mRightWheel;
	int mMaxRecord;
public:
	EvLog(ev3api::Clock*, ev3api::ColorSensor*, ev3api::SonarSensor*,
			ev3api::GyroSensor*, ev3api::Motor*, ev3api::Motor*, ev3api::Motor*);
	void input(int x1, int x2, int x3, int x4);
	void input(int cflag, int x1, int x2, int x3, int x4);
	void output(int eflag);
	int log_queue(void) { return (wcount - rcount); }
	int get_time(void) { return((int) mClock->now() - start_time); }
	void set_start_time(void) { start_time = (int) mClock->now(); }
};

#endif /* EVLOG_H_ */

#ifdef __cplusplus
}
#endif
