#ifdef __cplusplus
extern "C" {
#endif

#ifndef RUN_METHOD_H_
#define RUN_METHOD_H_


//#define LIGHT_WHITE          40  /* 白色の光センサ値 */
//#define LIGHT_BLACK          0  /* 黒色の光センサ値 */
#include "ev3api.h"
#include "Motor.h"
#include "Pid.h"
#include "Location.h"
#include "GyroSensor.h"
#include "ColorSensor.h"
#include "BalancerCpp.h"        // <1>
#include "balancer.h"

class RunMethod{
private:
	ev3api::Motor*				mTailMotor;
	ev3api::Motor*				mLeftWheel;
	ev3api::Motor*				mRightWheel;
	RoboLoc*					mRloc;
	Balancer*					mBalancer;
	ev3api::GyroSensor*			mGyroSensor;
	ev3api::ColorSensor*		mColorSensor;
	int forward;
	int turn;
	int pwm_L;
	int pwm_R;
	int start_dist;
	int Current_taile;
	//int pwm_Tail;
	Pid mPid;
	int pre;
	int Light;
	int hit;
	int plus;
	//int ccount;
	//int angle;
public:
	RunMethod(ev3api::Motor*, ev3api::Motor*, ev3api::Motor*, RoboLoc*, Balancer*, ev3api::GyroSensor*, ev3api::ColorSensor*);
	int RunOnOff(int, signed char, int);
	int RunPid(int, signed char, int, int, int);
	int RunPid_Kai(int, signed char, int, int, int, int);
	void Test(void);
	void SetPid(float, float, float);
	int TailStopHere(int);
	int StopHere(int, int);
	int TailRun(int, int, int, int);
	void TailControl(int, int32_t);
	int Tail(int, int, int, int);
	int StarGazer(int, signed int, int);
	// int StarGazer_dansa(int, signed int, int);
	int SpinTop(int);
	int Rocket_S(int);
	void ThresholdTransfer(int, int);
	int Step(int, signed char);
	int Spin(int, int, int);
	int LIGHT_WHITE;
	int LIGHT_BLACK;
	void BlackReturn(int, int);
	int Detection(int, int);
	int Curve_Def(void);
	void Plan(int, int, int);
	int Line_(void);
	int ED;
	int BlackDetect(int);
	int tail_up(int);
	int tail_down(int);
	int tail_onoff(int, signed int, int, int, int, float);
	int Run_dansa(int, signed char, int);
	int RunPidBack(int, signed char, int, int, int);
	int StarGazer_v2(int, signed int, int);
	int TailRun_Kai(signed char, int,int);
};

#endif

#ifdef __cplusplus
}
#endif
