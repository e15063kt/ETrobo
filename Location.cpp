/*
 * Location.cpp
 *
 *  Created on: 2012/03/05
 *      Author: shikama
 */
#include "Location.h"
#include <math.h>

RoboLoc::RoboLoc(float dia, float width, float gratio)
{
	n = 0;
	flag = 0;
	R = dia;
	D = width;
	GR = gratio;
	x = 0;
	y = 0;
	tl = 0;;
	distance = 0;
	omega = 0;
}

void RoboLoc::input(int n6, int n7)
{
	if (n == 0) {
		pthr = (float) n6;
		pthl = (float) n7;
	}
	if (n >= 1) {
        thr = ((float) pthr - n6) * GR;
        thl = ((float) pthl - n7) * GR;
        pthr = (float) n6;
        pthl = (float) n7;
        tr = 3.14159265 * R * thr / 180;
        tl = 3.14159265 * R * thl / 180;
        d = -(tr + tl) / 2;
        w = (tl - tr) / (2 * D);
        x = x + d * cos(omega + (w / 2));
        y = y + d * sin(omega + (w / 2));
        omega = omega + w;
        distance += d;
        omega_d = omega * 180 / 3.14159265;	// ラディアンを度に変換
	}
	++n;
}

