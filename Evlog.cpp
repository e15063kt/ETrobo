/*
 * Evlog.cpp
 *
 *  Created on: 2015/06/07
 *      Author: shikama
 */
#include "Evlog.h"
#include <stdio.h>
#include <string.h>

EvLog::EvLog(ev3api::Clock* clock,
					ev3api::ColorSensor* colorSensor,
					ev3api::SonarSensor* sonarSensor,
					ev3api::GyroSensor*	gyroSensor,
					ev3api::Motor* tail,
					ev3api::Motor* leftWheel,
					ev3api::Motor* rightWheel)
	: mClock(clock),
	  mColorSensor(colorSensor),
	  mSonarSensor(sonarSensor),
	  mGyroSensor(gyroSensor),
	  mTail(tail),
	  mLeftWheel(leftWheel),
	  mRightWheel(rightWheel){
	wcount = 0;
	rcount = 0;
	full = 0;
	iiflag = 1;
	ioflag = 1;
	total_count = 0;
	mMaxRecord = 100000;

	char str[32] = "";
	char str1[] = "evlog_";
	char str2[] = ".csv";
	char s[16];
	int n = 0;
	FILE* ffp;

	if ((ffp = fopen("File_no.txt", "r")) == NULL){}
	else {
		fscanf(ffp, "%d", &n);
		fclose(ffp);
	}
	printf("n = %d\n", n);
	if ((ffp = fopen("File_no.txt", "w")) == NULL){
		fprintf(stderr, "Cant Open File!!\n");
	}
	n++;
	fprintf(ffp, "%d", n);
	sprintf(s, "%d", n);

	strcat(str, str1);
	strcat(str, s);
	strcat(str, str2);
	strcpy(fname, str);
	fclose(ffp);
}

void EvLog::input(int x1, int x2, int x3, int x4)
{
	if (iiflag) {
		start_time = (int) mClock->now();
		iiflag = 0;
	}
	if (full == 1) return;
	if (wcount - rcount > MAX) {
		full = 1;
		return;
	}
	int n = wcount % MAX;
	data[n].time = (int) mClock->now() - start_time;
	data[n].light = (int) mColorSensor->getBrightness();
//	data[n].light = (int) mColorSensor->getColorNumber();
//	data[n].light = mColorLight->get_value();
	data[n].sonar = (int) mSonarSensor->getDistance();
	data[n].gyro = (int) mGyroSensor->getAnglerVelocity();
	data[n].tail = (int) mTail->getCount();
	data[n].lmtr = (int) mLeftWheel->getCount();
	data[n].rmtr = (int) mRightWheel->getCount();
	data[n].x[0] = x1;
	data[n].x[1] = x2;
	data[n].x[2] = x3;
	data[n].x[3] = x4;
	++wcount;
}

void EvLog::input(int cflag, int x1, int x2, int x3, int x4)
{
	if (iiflag) {
		start_time = (int) mClock->now();
		iiflag = 0;
	}
	if (full == 1) return;
	if (wcount - rcount > MAX) {
		full = 1;
		return;
	}
	int n = wcount % MAX;
	data[n].time = (int) mClock->now() - start_time;
	data[n].light = (int) mColorSensor->getBrightness();
	data[n].sonar = (int) mSonarSensor->getDistance();
	data[n].gyro = (int) mGyroSensor->getAnglerVelocity();
	data[n].tail = (int) mTail->getCount();
	data[n].lmtr = (int) mLeftWheel->getCount();
	data[n].rmtr = (int) mRightWheel->getCount();
	data[n].x[0] = x1;
	data[n].x[1] = x2;
	data[n].x[2] = x3;
	data[n].x[3] = x4;
	++wcount;
}

void EvLog::output(int eflag)
{
	int i;
	if (total_count == mMaxRecord) { return; }
	if (ioflag == 1) {
		fp = fopen(fname, "w");
		fprintf(fp, "time, light, sonar, gyro, tail, lmtr, rmtr, d1, d2, d3, d4\n");
		ioflag = 0;
	}
	while (wcount != rcount) {
		i = rcount % MAX;
		fprintf(fp, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", 
			data[i].time, data[i].light, data[i].sonar, data[i].gyro, data[i].tail, data[i].lmtr, data[i].rmtr,
				data[i].x[0], data[i].x[1], data[i].x[2], data[i].x[3]);
		++rcount;
		++total_count;
	}
	if (eflag == 1) fclose(fp);
}
