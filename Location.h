#ifndef LOCATION_H_
#define LOCATION_H_

#include <math.h>
#define M 100 // 1区間のログデータ数

class RoboLoc {
private:
	float pthr;
	float pthl;
	float thr;
	float thl;
	float tr;
	float tl;
	float w;
	float d;
	float R;
	int n;
public:
	RoboLoc(float rad, float width, float gratio);	// dia: 車輪の半径、width: 車輪間隔の1/2、ギア比
	void input(int n6, int n7);
	unsigned char flag;
	float D;
	float GR;
	float x;			//　走行体のx座標
	float y;			//　走行体のy座標
	float omega;		//　走行体の角度(rad)
	float omega_d;		//　走行体の角度（度）
	float distance;		//　走行体の総走行距離
	float raw_w(void) { return w; }
};

#endif /* LOCATION_H_ */
