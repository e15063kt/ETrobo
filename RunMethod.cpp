#include "RunMethod.h"
#include <stdlib.h>

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B,
    tail_motor      = EV3_PORT_A;

#define TAIL_ANGLE_DRIVE      0 
//#define TAIL_ANGLE_stargazer     80
extern int wait_event(void);
extern void tail_control(signed int angle);
extern int sonar_alert(void);
extern int angle(int a);
int Current_tail;

RunMethod::RunMethod(ev3api::Motor* tailMotor,
					ev3api::Motor* leftWheel,
					ev3api::Motor* rightWheel,
					RoboLoc* rLoc,
					Balancer* balancer,
					ev3api::GyroSensor* gyroSensor,
					ev3api::ColorSensor* colorSensor)
	: mTailMotor(tailMotor),
	  mLeftWheel(leftWheel),
	  mRightWheel(rightWheel),
	  mRloc(rLoc),
	  mBalancer(balancer),
	  mGyroSensor(gyroSensor),
	  mColorSensor(colorSensor) {
	  forward = 0;
	  turn = 0;
	  pwm_R = 0;
	  pwm_L = 0;
	  start_dist = 0;
      pre = 0;
      ED = 0;
      hit = 0;
      plus = 0;
	  mPid.set_params(0.5, 0.1, 0.05);
}

//*********************************************************************************
//
//  関数名　　： RunOnOff
//  引数　　　：　edge(エッジ), forward(走行速度), dist(距離)
//  返り値　　：　なし
//  説明　　　：　on・off走行用プログラム
//
//*********************************************************************************


int RunMethod::RunOnOff(int edge, signed char forward, int dist){
    start_dist = mRloc->distance;
        int32_t motor_ang_l, motor_ang_r;
        int gyro, volt;
        int VX = 0;
    while((mRloc->distance - start_dist) < dist){
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE);

        // if (sonar_alert() == 1) {
        //     forward = turn = 0;
        // } else {
          if (mColorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK) * 0.5){
                 turn =  5; 
             } else {
                 turn = -5; 
             }
         



        if (edge == 0) {
        	turn = -turn;
		}

        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        VX = Detection(50, gyro);

        if(VX == 1) break;

        mBalancer->setCommand(forward, turn);   // <1>
        mBalancer->update(gyro, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmRight();       // <3>
        pwm_R = mBalancer->getPwmLeft();        // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);


        if (wait_event()) {
            ext_tsk();
        }
    }
    return 0;
}

//*********************************************************************************
//
//  関数名　　：　RunPid
//  引数　　　：　edge(エッジ), forward(走行速度), dist(距離)
//  返り値　　：　なし
//  説明　　　：　PID走行用(ライントレース)プログラム
//
//*********************************************************************************


int RunMethod::RunPid(int edge, signed char forward, int dist, int Black_Plus, int Gyro_Plus){
    start_dist = mRloc->distance;
    int32_t     motor_ang_l, motor_ang_r;
    int         gyro, volt;
    int count = 0;
    while ((mRloc->distance - start_dist) < dist) {
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); 

        int err = mColorSensor->getBrightness() - (LIGHT_WHITE + LIGHT_BLACK + Black_Plus) * 0.5;
        turn = mPid.input(err);

        if (edge == 0) {
        	turn = -turn;
		}

        count++;

        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        mBalancer->setCommand(forward, turn);   // <1>
        mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmRight();       // <3>
        pwm_R = mBalancer->getPwmLeft();        // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if(count % 200 == 0) plus += Gyro_Plus;



        if (wait_event()) {
            ext_tsk();
        }
    }
    return 0;
}


int RunMethod::RunPid_Kai(int edge, signed char forward, int dist, int Black_Plus, int Gyro_Plus, int Angle){
    start_dist = mRloc->distance;
    int32_t     motor_ang_l, motor_ang_r;
    int         gyro, volt;
    while ((mRloc->distance - start_dist) < dist) {
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(Angle); 

        int err = mColorSensor->getBrightness() - (LIGHT_WHITE + LIGHT_BLACK + Black_Plus) * 0.5;
        turn = mPid.input(err);

        if (edge == 0) {
            turn = -turn;
        }

        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        mBalancer->setCommand(forward, turn);   // <1>
        mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmRight();       // <3>
        pwm_R = mBalancer->getPwmLeft();        // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();
        }
    }
    return 0;
}


void RunMethod::Test(){
	while(1){
	mLeftWheel->setPWM(50);
	mRightWheel->setPWM(50);
	if (wait_event()) {
		ext_tsk();
	}
	slp_tsk();
	}
}

void RunMethod::SetPid(float p, float i, float d) {
	mPid.set_params(p, d, i);
}

//*********************************************************************************
//
//  関数名　　：　TailStopHere
//  引数　　　：　edge(エッジ), forward(走行速度), second_ms(待機時間), ang(尻尾角度)
//  返り値　　：　なし
//  説明　　　：　尻尾を出して走行停止(待機)するプログラム
//
//*********************************************************************************

int RunMethod::TailStopHere(int second_ms){
	start_dist = mRloc->distance;
    int         count = 0;
    int16_t     Angle = ev3_motor_get_counts(tail_motor);
    while (++count < second_ms) {    	
		if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(Angle); 

       	turn = 0;

        pwm_L = 0; 
        pwm_R = 0; 
        mLeftWheel->setPWM(0);
        mRightWheel->setPWM(0);

        if (wait_event()) {
            ext_tsk();
        }
    }
 
    mLeftWheel->reset();
    mRightWheel->reset();
    mGyroSensor->reset();
    mBalancer->init(0);

    return 0;
}

//*********************************************************************************
//
//  関数名　　： StopHere
//  引数　　　：　edge(エッジ), forward(走行速度), second_ms(待機時間), ang(尻尾角度)
//  返り値　　：　なし
//  説明　　　：　倒立(走行停止または待機)するプログラム
//
//*********************************************************************************

int RunMethod::StopHere(int Gyro_Plus, int second_ms){
	start_dist = mRloc->distance;
    int         count = 0;
    int32_t     motor_ang_l, motor_ang_r;
    int         gyro, volt;
    signed char pwm_L, pwm_R; /* 左右モータPWM出力 */     
    while (++count < second_ms) {    	
		if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); 

        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        mBalancer->setCommand(0, 0);   // <1>
        // mBalancer->update(gyro + Gyro_Plus, motor_ang_r, motor_ang_l, volt); // <2>
        mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmLeft();    // <3>
        pwm_R = mBalancer->getPwmRight();    // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();
        }
    }
    return 0;
}

//*********************************************************************************
//
//  関数名　　：　TailRun
//  引数　　　：　edge(エッジ), second_ms(待機時間)
//  返り値　　：　なし
//  説明　　　：　尻尾走行用プログラム
//
//*********************************************************************************

int RunMethod::TailRun(int second_ms, int pwmL, int pwmR, int Angle){
    start_dist = mRloc->distance;
    int         count = 0;
    while (++count < second_ms) {
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(Angle); 

        pwm_L = pwmL;
        pwm_R = pwmR;
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();

        }
    }
    return 0;
}

//*********************************************************************************
//
//  関数名　　：　TailRun
//  引数　　　：　edge(エッジ), pwmL(左エンコーダ値), pwmR(右エンコーダ値), count_ang(最大尻尾角度)
//  返り値　　：　なし
//  説明　　　：　尻尾走行用プログラム
//
//*********************************************************************************

void RunMethod::TailControl(int pwm_arm, int32_t angle){
    int startAngle = mTailMotor->getCount();
    if (angle > 0) {
        while ((mTailMotor->getCount() - startAngle) < angle){
            mTailMotor->setPWM(pwm_arm);
            if (wait_event())   ext_tsk();
        }
    } else {
        while ((mTailMotor->getCount() - startAngle) > angle){
            mTailMotor->setPWM(-pwm_arm);
            if (wait_event())   ext_tsk();
        }
    }
    mTailMotor->stop();
}




int RunMethod::Tail(int edge, int pwmL, int pwmR, int count_ang){
    int         count_ms = 0;
    while (++count_ms < 10000000) {
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        int16_t Angle = ev3_motor_get_counts(tail_motor);
        
        if (++count_ms % 16 == 0){
            ev3_motor_set_power(tail_motor, 100);
            tail_control(Angle + 1);
        }

        if (Angle == count_ang) break;
        
        int err = mColorSensor->getBrightness() - (LIGHT_WHITE + LIGHT_BLACK)/2;
        turn = mPid.input(err);

        if (edge == 0) {
            turn = -turn;
        }

        pwm_L = pwmL; 
        pwm_R = pwmR;
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();
        }
    }
   
    return 0;
}

//*********************************************************************************
//
//  関数名　　：　StarGazer
//  引数　　　：　edge(エッジ), angle(尻尾角度)
//  返り値　　：　なし
//  説明　　　：　尻尾出しプログラム
//
//*********************************************************************************

int RunMethod::StarGazer(int edge, signed int angle, int TAIL_ANGLE_stargazer){
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    int forward = 1;
    int count = 0;

    while(1){

    if (ev3_button_is_pressed(BACK_BUTTON)) break;

         // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 

        while(count++ < 240){//200

            if(TAIL_ANGLE_stargazer >= count){
                tail_control(count);
            }else{
                tail_control(TAIL_ANGLE_stargazer);
            }

            // “|—§UŽq§ŒäAPI ‚É“n‚·ƒpƒ‰ƒ[ƒ^‚ðŽæ“¾‚·‚é 
            motor_ang_l = mLeftWheel->getCount();
            motor_ang_r = mRightWheel->getCount();
            gyro = mGyroSensor->getAnglerVelocity();
            volt = ev3_battery_voltage_mV();

            // “|—§UŽq§ŒäAPI‚ðŒÄ‚Ño‚µA“|—§‘–s‚·‚é‚½‚ß‚Ì 
            // ¶‰Eƒ‚[ƒ^o—Í’l‚ð“¾‚é 
            if(count <= TAIL_ANGLE_stargazer + 20){
                if (mColorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK) * 0.5){
                    turn = 0; // ¶ù‰ñ–½—ß 
                } else {
                    turn = 0; // ‰Eù‰ñ–½—ß 
                }

                if (edge == 0) turn = -turn;

                mBalancer->setCommand(forward, turn);   // <1>
                mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
                pwm_L = mBalancer->getPwmLeft();    // <3>
                pwm_R = mBalancer->getPwmRight();    // <3>
                mLeftWheel->setPWM(pwm_L);
                mRightWheel->setPWM(pwm_R);
            }
            else if(TAIL_ANGLE_stargazer  + 20 < count && count <= TAIL_ANGLE_stargazer + 37){
                mBalancer->setCommand(forward, 0);   // <1>
                mBalancer->update(19 - plus, motor_ang_r, motor_ang_l, volt); // <2>
//            pwm_L = 60;
//            pwm_R = 60;
                pwm_L = mBalancer->getPwmRight();       // <3>
                pwm_R = mBalancer->getPwmLeft();        // <3>
                mLeftWheel->setPWM(pwm_L);
                mRightWheel->setPWM(pwm_R);
            }else{
                pwm_L = 0;
                pwm_R = 0;
                mLeftWheel->setPWM(pwm_L);
                mRightWheel->setPWM(pwm_R);
            }

            if (wait_event()) {
            ext_tsk();
            }
        }

        count = 0;

        int x = (TAIL_ANGLE_stargazer - angle) * 20;
        Current_taile = TAIL_ANGLE_stargazer;

        while(count++ < x){

            if(count % 20 == 0)  {
                Current_taile -= 1;
            }

            tail_control(Current_taile);

            pwm_L = 7;
            pwm_R = 7;

            mLeftWheel->setPWM(pwm_L);
            mRightWheel->setPWM(pwm_R);

            if (wait_event()) {
            ext_tsk();
        }
        }
        Current_taile = angle;
        plus = 0;
        break;
    }
    return 0;
}

//*********************************************************************************
//
//  関数名　　：　SpinTop
//  引数　　　：　wait(回転時間)
//  返り値　　：　なし
//  説明　　　：　段差の上で回転するプログラム
//
//*********************************************************************************

int RunMethod::SpinTop(int deg){
    float start_omega = mRloc->omega_d;
    int16_t     Angle = ev3_motor_get_counts(tail_motor);
    while (mRloc->omega_d - start_omega <= deg){
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(Angle);

        forward = 0;

        ev3_motor_set_power(left_motor, 40);
        
        ev3_motor_set_power(right_motor, -40);
                
        if (wait_event()) {
            ext_tsk();
        }
    }
    
    return 0;
}



int RunMethod::Rocket_S(int ang){
    int count = 0;
    while(count++ < 16){
        tail_control(ang); //99
        if(wait_event()){
            ext_tsk();
        }
    }
    return 0;
}



void RunMethod::ThresholdTransfer(int W, int B){
    LIGHT_WHITE = W;
    LIGHT_BLACK = B;   
}



int RunMethod::Step(int edge, signed char forward){
    start_dist = mRloc->distance;
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    int count = 0;
    while(mGyroSensor->getAnglerVelocity() > 60){

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); 

        int err = mColorSensor->getBrightness() - (LIGHT_WHITE + LIGHT_BLACK) * 0.5;
        turn = mPid.input(err);

        if (edge == 0) {
            turn = -turn;
        }

        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        mBalancer->setCommand(forward, turn);   // <1>
        mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmLeft();    // <3>
        pwm_R = mBalancer->getPwmRight();    // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);
    
        if (wait_event()) {
            ext_tsk();
        }

    }

    while(++count < 300){
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); 

        int err = mColorSensor->getBrightness() - (LIGHT_WHITE + LIGHT_BLACK) * 0.5;
        turn = mPid.input(err);

        if (edge == 0) {
            turn = -turn;
        }

        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        mBalancer->setCommand(10, turn);   // <1>
        mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmRight();    // <3>
        pwm_R = mBalancer->getPwmLeft();    // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

    }

    mRloc->distance = 0;

    return 0;
}

/////////////////////////////////////////////////////////
int RunMethod::Spin(int deg, int G, int flag){
    float start_omega = mRloc->omega_d;
    
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    int y = 0;

    int count = 0;
    while(count++ <= 500 && flag == 1){
        tail_control(TAIL_ANGLE_DRIVE); // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 

        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        mBalancer->setCommand(0, 0);   // <1>

        mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmRight();        // <3>
        pwm_R = mBalancer->getPwmLeft();       // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();
        }
    }

    while (abs(mRloc->omega_d - start_omega) <= deg){
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 

        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        // int err = mColorSensor->getBrightness() - (LIGHT_WHITE + LIGHT_BLACK) * 0.5;

        // if(el <= 1000 && on == 0){
        //     if(err >= 0){
        //         el += err * 0.6;
        //     }
        // }else {
        //     on = 1;
        // }

        // if(on == 1){
        //     if(err < 10){
        //         if(err < 0){
        //             el += err * 30;
        //         }else{
        //             el -= err * 30;
        //         }
        //         y = 0;
        //         x = 3;
        //     }else{
        //         y = 8;
        //         x = 15;
        //     }
        //     if((start_omega - mRloc->omega_d >= -90 && start_omega - mRloc->omega_d < 0) || (start_omega - mRloc->omega_d <= 90 && start_omega - mRloc->omega_d > 0)){
        //         mBalancer->setCommand(-x, 0);   // <1>
        //     }else if((start_omega - mRloc->omega_d >= -180 && start_omega - mRloc->omega_d < -90) || (start_omega - mRloc->omega_d <= 180 && start_omega - mRloc->omega_d > 90)){
        //         if(start_omega - mRloc->omega_d > 0) y = - y;
        //         mBalancer->setCommand(-x - 2, y);   // <1>
        //     }else if((start_omega - mRloc->omega_d > 180 && start_omega - mRloc->omega_d <= 270)  || (start_omega - mRloc->omega_d < -180 && start_omega - mRloc->omega_d >= -270)){
        //         if(start_omega - mRloc->omega_d > 0) y = - y;
        //         mBalancer->setCommand(x, y);   // <1>
        //     }else if((start_omega - mRloc->omega_d > 270)  || (start_omega - mRloc->omega_d < -270)){
        //         mBalancer->setCommand(x, 0);   // <1>
        //     }
        //     if(el < 100) {
        //         on = 0;
        //         el = 0;
        //     }
            
        // }
        if(G == 0){
            y = -80;
        }else{
            y = 80;
        }

        mBalancer->setCommand(0, y);

        if(flag == 1){
        if(abs(mRloc->omega_d - start_omega) >= deg * 0.3){
            gyro = gyro + 4;
        }else if(abs(mRloc->omega_d - start_omega) >= deg * 0.7){
            gyro = gyro + 5;
        }else{
            gyro = gyro + 2;
        }
        }else{
            gyro = gyro + 2;
        }

        // if(flag == 1){
        // if(abs(mRloc->omega_d - start_omega) >= deg * 0.3){
        //     gyro = gyro + 3;
        // }else if(abs(mRloc->omega_d - start_omega) >= deg * 0.5){
        //     gyro = gyro + 5;
        // }else if(abs(mRloc->omega_d - start_omega) >= deg * 0.7){
        //     gyro = gyro + 7;
        // }else if(abs(mRloc->omega_d - start_omega) >= deg * 0.9){
        //     gyro = gyro + 8;
        // }else{
        //     gyro = gyro + 2;
        // }
        // }else{
        //     gyro = gyro + 1;
        // }



        mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmRight();        // <3>
        pwm_R = mBalancer->getPwmLeft();        // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        // ev3_motor_steer(EV3_PORT_C, EV3_PORT_B, 50, -100); 


        if (wait_event()) {
            ext_tsk();
        }
    }
    return 0;
}

// void RunMethod::BlackReturn(int edge){
//     int32_t motor_ang_l, motor_ang_r;
//     int gyro, volt;
//     int el = 0;
//     int y = 0;

//     int edge = Curve_Def();

//     // while(1){
//         tail_control(TAIL_ANGLE_DRIVE); // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 
//         motor_ang_l = mLeftWheel->getCount();
//         motor_ang_r = mRightWheel->getCount();
//         gyro = mGyroSensor->getAnglerVelocity();
//         volt = ev3_battery_voltage_mV();

//         // int err = mColorSensor->getBrightness() - (LIGHT_WHITE + LIGHT_BLACK) * 0.5;
//         //     if(err < 10){
//         //         if(err < 0){
//         //             el += err;
//         //         }else{
//         //             el -= err;
//         //         }
//         //     }

//         // if(edge == 0){
//         //     y = -3;
//         // }else{
//         //     y = 3;
//         // }

//         // mBalancer->setCommand(5, y);   // <1>

//         // if(el <= -20) {
//         RunOnOff(edge, 50);//, 50);   // <1>
//         int start_dist = mRloc->distance;
//         while(abs(mRloc->distance - start_dist) < 135){
//             tail_control(TAIL_ANGLE_DRIVE); // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 
//             motor_ang_l = mLeftWheel->getCount();
//             motor_ang_r = mRightWheel->getCount();
//             gyro = mGyroSensor->getAnglerVelocity();
//             volt = ev3_battery_voltage_mV();

//             mBalancer->setCommand(-10, 0);

//             mBalancer->update(gyro, motor_ang_r, motor_ang_l, volt); // <2>
//             pwm_L = mBalancer->getPwmRight();        // <3>
//             pwm_R = mBalancer->getPwmLeft();       // <3>
//             mLeftWheel->setPWM(pwm_L);
//             mRightWheel->setPWM(pwm_R);

//             if (wait_event()) {
//                 ext_tsk();
//             }
//         }
//         break;
//         // }

//         mBalancer->update(gyro, motor_ang_r, motor_ang_l, volt); // <2>
//         pwm_L = mBalancer->getPwmRight();        // <3>
//         pwm_R = mBalancer->getPwmLeft();       // <3>
//         mLeftWheel->setPWM(pwm_L);
//         mRightWheel->setPWM(pwm_R);

        

//         if (wait_event()) {
//             ext_tsk();
//         }
//     }
// // }

void RunMethod::BlackReturn(int des_dist, int flag){
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

    int edge = Curve_Def();

    ev3_speaker_play_tone(NOTE_C5, 300);

        tail_control(TAIL_ANGLE_DRIVE); // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 
        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        RunOnOff(edge, 30, 50);   // <1>
        ED = edge;
        int start_dist = mRloc->distance;
        while(abs(mRloc->distance - start_dist) < des_dist && flag == 1){
            tail_control(TAIL_ANGLE_DRIVE); // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 
            motor_ang_l = mLeftWheel->getCount();
            motor_ang_r = mRightWheel->getCount();
            gyro = mGyroSensor->getAnglerVelocity();
            volt = ev3_battery_voltage_mV();

            mBalancer->setCommand(-10, 0);

            mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
            pwm_L = mBalancer->getPwmRight();        // <3>
            pwm_R = mBalancer->getPwmLeft();       // <3>
            mLeftWheel->setPWM(pwm_L);
            mRightWheel->setPWM(pwm_R);

            if (wait_event()) {
                ext_tsk();
            }
        }
    }

int RunMethod::Detection(int dif, int gyro){
    int x = 0;
    int ans = abs(gyro - pre);
    if(dif <= ans) x = 1;
    pre = gyro;
    return x;
}

int RunMethod::Curve_Def(){
    int UI = 0;
    int num = 1;
    int x = 0;
    int y = 0;
    int ang = 0;
    while(1){
    tail_control(TAIL_ANGLE_DRIVE); // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 
    if(num == 1) {
        x = 2;
        y = 10;
        ang = 10;
    }else if(num == 2){
        x = 2;
        y = -10;
        ang = 10;
    }    
    Plan(x, y, ang);
    UI = Line_();

    if(UI == 1){
        if(num == 1) UI = 1;
        else UI = 0;
        return UI;
        break;
    }

    Plan(x, y, ang * 3);
    UI = Line_();

    if(UI == 1){
        if(num == 1) UI = 1;
        else UI = 0;
        return UI;
        break;
    }

    Plan(-x * 2, -y * 2, ang * 4);

    if(num == 1) num = 2;
    else num = 1;
}
return 0;
}

void RunMethod::Plan(int x, int y, int ang){
    float start_omega = mRloc->omega_d;
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    int plus = -1;
    if(x < 0) plus = 1;
    while(abs(mRloc->omega_d - start_omega) <= ang){
        tail_control(TAIL_ANGLE_DRIVE); // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 
        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        mBalancer->setCommand(x, y);

        mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmRight();        // <3>
        pwm_R = mBalancer->getPwmLeft();       // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);
        if (wait_event()) {
            ext_tsk();
        }
    }
}

int RunMethod::Line_(){
    int flag = 0;
    int count = 0;
    int el = 0;
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    while(count <= 200){
        tail_control(TAIL_ANGLE_DRIVE); // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 
        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        int err = mColorSensor->getBrightness() - (LIGHT_WHITE + LIGHT_BLACK) * 0.5;
            if(err <= 6){
                if(err < 0){
                    el += err;
                }else{
                    el -= err;
                }
            }

        mBalancer->setCommand(0, 0);

        mBalancer->update(gyro + 2 - plus, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmRight();        // <3>
        pwm_R = mBalancer->getPwmLeft();       // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if(el <= -10) flag = 1;
        count++;

        if (wait_event()) {
            ext_tsk();
        }
    }
    return flag;
}


    
int RunMethod::BlackDetect(int deg){
    float start_omega = mRloc->omega_d;
    int16_t Angle = ev3_motor_get_counts(tail_motor);
    forward = 0;
    while(abs(mRloc->omega_d - start_omega) < deg){

        tail_control(Angle);

        ev3_motor_set_power(left_motor, 0);
        
        ev3_motor_set_power(right_motor, 20);

        if(mColorSensor->getBrightness() < 5){
            ev3_speaker_play_tone(NOTE_C6, 200);
            break;

        }else if(abs(mRloc->omega_d - start_omega) < deg && mColorSensor->getBrightness() > 5){
            while(abs(mRloc->omega_d - start_omega) < -deg){

                tail_control(Angle);

                ev3_motor_set_power(left_motor, 0);
        
                ev3_motor_set_power(right_motor, -20);

            }

        } 

        if (wait_event()) {
            ext_tsk();
        }

    }

    while(abs(mRloc->omega_d - start_omega) < -deg){

        tail_control(Angle);

        ev3_motor_set_power(left_motor, 20);
        
        ev3_motor_set_power(right_motor, 0);

        if(mColorSensor->getBrightness() < 5){
            ev3_speaker_play_tone(NOTE_C6, 200);
            break;
        
        }else if(abs(mRloc->omega_d - start_omega) < -deg && mColorSensor->getBrightness() > 5){
            while(abs(mRloc->omega_d - start_omega) < -deg){

                tail_control(Angle);

                ev3_motor_set_power(left_motor, 0);
        
                ev3_motor_set_power(right_motor, -20);

            }

        }

        if (wait_event()) {
            ext_tsk();
        }

    }

    return 0;
}


// int RunMethod::Tail_up(int angle){
//     int         j = 0, k = 0;
//     int         i = (angle - Current_taile);
//     while(j < i){
//         k += 1;
//             if(k % 16 == 0){//20
//                 Current_taile += 1;
//                 j += 1;
//                 Light /= 0.954;
//             }

//         tail_control(Current_taile);


//         pwm_L = -8;
//         pwm_R = -8;

//         if(k < 20){
//             pwm_L = 0;
//             pwm_R = 0;
//         }
        
//         mLeftWheel->setPWM(pwm_L);
//         mRightWheel->setPWM(pwm_R);

//         if (wait_event()) {
//             ext_tsk();
//         }
//     }

//     j = 0;

//     while(j++ < 20){//20
//         tail_control(Current_taile);

//         pwm_L = 0;
//         pwm_R = 0;
//         mLeftWheel->setPWM(pwm_L);
//         mRightWheel->setPWM(pwm_R);

//         if (wait_event()) {
//             ext_tsk();
//         }
//     }
//     return 0;
// }

// int RunMethod::tail_onoff(int edge, signed int angle, int dist, int forward, int plus, int inf, float y){               //距離制御tail_onoff
//     start_dist = mRloc->distance;
//     int up = 0;
//     int i = Current_taile - angle;
//     int gyro;
//     int x = 0;

//     if(i < 0){
//         up = 1;
//         //i = abs(i);
//     }

//         if(up == 0){
//             tail_down(angle);
//             start_dist = mRloc->distance;
//         }else if(up == 1){
//             tail_up(angle);
//             start_dist = mRloc->distance;
//         }

//     while(abs(mRloc->distance - start_dist) < dist){
        

//         tail_control(angle);

//         gyro = mGyroSensor->getAnglerVelocity();
        
//         if (mColorSensor->getBrightness() >= Light + plus){
//             turn =  1; // ¶ù‰ñ–½—ß 
//         } else {
//             turn = -1; // ‰Eù‰ñ–½—ß 
//         }

//         if(edge == 0) turn = -turn;

//          // if(flag == 2){
//          //    err = mRloc->omega_d - SUM;
//          //    if (err > 0) turn = 1;//15
//          //    else turn = -1;//15
//          // }

//         pwm_R = forward;
//         pwm_L = forward;

//         if (turn > 0){ //左旋回
//             pwm_L *= y;
//         } else {
//             pwm_R *= y;
//         }

//         if(forward == 0) pwm_R = pwm_L = 0;

//         x = Detection(30, gyro);

//         if(inf == 1){
//             if(x == 1) {
//                 tail_onoff(1, angle, 200, 9, 0, 0, 0.8);
//                 break;
//             }
//         }

//         mLeftWheel->setPWM(pwm_L);
//         mRightWheel->setPWM(pwm_R);

//         if (wait_event()) {
//             ext_tsk();
//         }
//         }
//         Current_taile = angle;
//         return 0;
// }

int RunMethod::tail_up(int angle){
    int         j = 0, k = 0;
    int         i = (angle - Current_taile);
    int s;
    if(hit == 0) {
        s = (90 - Current_taile);
        Light = ((LIGHT_WHITE + LIGHT_BLACK) * 0.5) * pow(0.9945, s);//0.965//0.954//0.956//0.947
        hit = 1;
    }


    while(j < i){
        k += 1;
            if(k % 16 == 0){//20
                Current_taile += 1;
                j += 1;
                Light /= 0.9945;
            }

        tail_control(Current_taile);


        pwm_L = -5;
        pwm_R = -5;

        if(k < 20){
            pwm_L = 0;
            pwm_R = 0;
        }
        
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();
        }
    }

    j = 0;

    while(j++ < 20){//20
        tail_control(Current_taile);

        pwm_L = 0;
        pwm_R = 0;
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();
        }
    }
    return 0;
}

int RunMethod::tail_down(int angle){
    int         j = 0, k = 0;
    int         i = (Current_taile - angle);
    int s;
     if(hit == 0) {
        s = (90 - Current_taile);
        Light = ((LIGHT_WHITE + LIGHT_BLACK) * 0.5) * pow(0.9945, s);//12
        hit = 1;
    }

    while(j < i){
        k += 1;
            if(k % 15 == 0){
                Current_taile -= 1;
                j += 1;
                Light *= 0.9945;
            }

        tail_control(Current_taile);

        pwm_L = 0;
        pwm_R = 0;
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();
        }
    }

    j = 0;

    while(j++ < 20){
        tail_control(Current_taile);

        pwm_L = 0;
        pwm_R = 0;
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();
        }
    }
    return 0;
}

int RunMethod::tail_onoff(int edge, signed int angle, int dist, int forward, int plus_L, float y){               //距離制御tail_onoff
    start_dist = mRloc->distance;
    int up = 0;
    int i = Current_taile - angle;
    // int x = 0;

    if(i < 0){
        up = 1;
    }

        if(up == 0){
            tail_down(angle);
            start_dist = mRloc->distance;
        }else if(up == 1){
            tail_up(angle);
            start_dist = mRloc->distance;
        }

    while((mRloc->distance - start_dist) < dist){
        

        tail_control(angle);

        
        if (mColorSensor->getBrightness() >= Light + plus_L){
            turn =  1; // ¶ù‰ñ–½—ß 
        } else {
            turn = -1; // ‰Eù‰ñ–½—ß 
        }

        if(edge == 0) turn = -turn;

        pwm_R = forward;
        pwm_L = forward;

        if (turn > 0){ //左旋回
            pwm_L *= y;
        } else {
            pwm_R *= y;
        }

        if(forward == 0) pwm_R = pwm_L = 0;

        // x = Detection(70, gyro);

        // if(inf == 1){
        //     if(x == 1) {
        //         tail_onoff(1, angle, 200, 0, 0, 0, 0.8);
        //         break;
        //     }
        // }

        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();
        }
        }
        Current_taile = angle;
        return 0;
}

int RunMethod::Run_dansa(int edge, signed char forward, int dist){
    start_dist = mRloc->distance;
    int32_t     motor_ang_l, motor_ang_r;
    int         gyro, volt;
    while ((mRloc->distance - start_dist) < dist) {
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(70); 

        // int err = mColorSensor->getBrightness() - (LIGHT_WHITE + LIGHT_BLACK) * 0.5;
        // turn = mPid.input(err);

        if (edge == 0) {
            turn = -turn;
        }


        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        mBalancer->setCommand(forward, 0);   // <1>
        mBalancer->update(gyro - plus - 50, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmRight();       // <3>
        pwm_R = mBalancer->getPwmLeft();        // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if (wait_event()) {
            ext_tsk();
        }
    }
    return 0;
}

// int RunMethod::StarGazer_dansa(int edge, signed int angle, int TAIL_ANGLE_stargazer){
//     int32_t motor_ang_l, motor_ang_r;
//     int gyro, volt;
//     int forward = 40;
//     int count = 0;

//     while(1){

//     if (ev3_button_is_pressed(BACK_BUTTON)) break;

//          // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 

//         while(count++ < 240){//200

//             if(TAIL_ANGLE_stargazer >= count){
//                 tail_control(count);
//             }else{
//                 tail_control(TAIL_ANGLE_stargazer);
//             }

//             // “|—§UŽq§ŒäAPI ‚É“n‚·ƒpƒ‰ƒ[ƒ^‚ðŽæ“¾‚·‚é 
//             motor_ang_l = mLeftWheel->getCount();
//             motor_ang_r = mRightWheel->getCount();
//             gyro = mGyroSensor->getAnglerVelocity();
//             volt = ev3_battery_voltage_mV();

//             // “|—§UŽq§ŒäAPI‚ðŒÄ‚Ño‚µA“|—§‘–s‚·‚é‚½‚ß‚Ì 
//             // ¶‰Eƒ‚[ƒ^o—Í’l‚ð“¾‚é 
//             if(count <= TAIL_ANGLE_stargazer + 20){
//                 if (mColorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK) * 0.5){
//                     turn =  1; // ¶ù‰ñ–½—ß 
//                 } else {
//                     turn = -1; // ‰Eù‰ñ–½—ß 
//                 }

//                 if (edge == 0) turn = -turn;

//                 mBalancer->setCommand(forward, turn);   // <1>
//                 mBalancer->update(gyro - plus - 30, motor_ang_r, motor_ang_l, volt); // <2>
//                 pwm_L = mBalancer->getPwmLeft();    // <3>
//                 pwm_R = mBalancer->getPwmRight();    // <3>
//                 mLeftWheel->setPWM(pwm_L);
//                 mRightWheel->setPWM(pwm_R);
//             }
//             else if(TAIL_ANGLE_stargazer  + 20 < count && count <= TAIL_ANGLE_stargazer + 37){
//                 mBalancer->setCommand(forward, 0);   // <1>
//                 mBalancer->update(19 - plus, motor_ang_r, motor_ang_l, volt); // <2>
// //            pwm_L = 60;
// //            pwm_R = 60;
//                 pwm_L = mBalancer->getPwmRight();       // <3>
//                 pwm_R = mBalancer->getPwmLeft();        // <3>
//                 mLeftWheel->setPWM(pwm_L);
//                 mRightWheel->setPWM(pwm_R);
//             }else{
//                 pwm_L = 0;
//                 pwm_R = 0;
//                 mLeftWheel->setPWM(pwm_L);
//                 mRightWheel->setPWM(pwm_R);
//             }

//             if (wait_event()) {
//             ext_tsk();
//             }
//         }

//         count = 0;

//         int x = (TAIL_ANGLE_stargazer - angle) * 20;
//         Current_taile = TAIL_ANGLE_stargazer;

//         while(count++ < x){

//             if(count % 20 == 0)  {
//                 Current_taile -= 1;
//             }

//             tail_control(Current_taile);

//             pwm_L = 5;
//             pwm_R = 5;

//             mLeftWheel->setPWM(pwm_L);
//             mRightWheel->setPWM(pwm_R);

//             if (wait_event()) {
//             ext_tsk();
//         }
//         }
//         Current_taile = angle;
//         plus = 0;
//         break;
//     }
//     return 0;
// }
int RunMethod::RunPidBack(int edge, signed char forward, int dist, int Black_Plus, int Gyro_Plus){
    start_dist = mRloc->distance;
    int32_t     motor_ang_l, motor_ang_r;
    int         gyro, volt;
    int count = 0;
    while ((mRloc->distance - start_dist) > dist) {
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); 

        int err = mColorSensor->getBrightness() - (LIGHT_WHITE + LIGHT_BLACK + Black_Plus) * 0.5;
        turn = mPid.input(err);

        if (edge == 0) {
            turn = -turn;
        }

        count++;

        motor_ang_l = mLeftWheel->getCount();
        motor_ang_r = mRightWheel->getCount();
        gyro = mGyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        mBalancer->setCommand(forward, turn);   // <1>
        mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = mBalancer->getPwmRight();       // <3>
        pwm_R = mBalancer->getPwmLeft();        // <3>
        mLeftWheel->setPWM(pwm_L);
        mRightWheel->setPWM(pwm_R);

        if(count % 200 == 0) plus += Gyro_Plus;



        if (wait_event()) {
            ext_tsk();
        }
    }
    return 0;
}
int RunMethod::StarGazer_v2(int edge, signed int angle, int TAIL_ANGLE_stargazer){
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;
    int forward = 10;
    int count = 0;

    while(1){

    if (ev3_button_is_pressed(BACK_BUTTON)) break;

         // ƒoƒ‰ƒ“ƒX‘–s—pŠp“x‚É§Œä 

        while(count++ < 240){//200

            if(TAIL_ANGLE_stargazer >= count){
                tail_control(count);
            }else{
                tail_control(TAIL_ANGLE_stargazer);
            }

            // “|—§UŽq§ŒäAPI ‚É“n‚·ƒpƒ‰ƒ[ƒ^‚ðŽæ“¾‚·‚é 
            motor_ang_l = mLeftWheel->getCount();
            motor_ang_r = mRightWheel->getCount();
            gyro = mGyroSensor->getAnglerVelocity();
            volt = ev3_battery_voltage_mV();

            // “|—§UŽq§ŒäAPI‚ðŒÄ‚Ño‚µA“|—§‘–s‚·‚é‚½‚ß‚Ì 
            // ¶‰Eƒ‚[ƒ^o—Í’l‚ð“¾‚é 
            if(count <= TAIL_ANGLE_stargazer + 20){
                if (mColorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK) * 0.5){
                    turn = 0; // ¶ù‰ñ–½—ß 
                } else {
                    turn = 0; // ‰Eù‰ñ–½—ß 
                }

                if (edge == 0) turn = -turn;

                mBalancer->setCommand(forward, turn);   // <1>
                mBalancer->update(gyro - plus, motor_ang_r, motor_ang_l, volt); // <2>
                pwm_L = mBalancer->getPwmLeft();    // <3>
                pwm_R = mBalancer->getPwmRight();    // <3>
                mLeftWheel->setPWM(pwm_L);
                mRightWheel->setPWM(pwm_R);
            }
            else if(TAIL_ANGLE_stargazer  + 20 < count && count <= TAIL_ANGLE_stargazer + 37){
                mBalancer->setCommand(forward, 0);   // <1>
                mBalancer->update(19 - plus, motor_ang_r, motor_ang_l, volt); // <2>
//            pwm_L = 60;
//            pwm_R = 60;
                pwm_L = mBalancer->getPwmRight();       // <3>
                pwm_R = mBalancer->getPwmLeft();        // <3>
                mLeftWheel->setPWM(pwm_L);
                mRightWheel->setPWM(pwm_R);
            }else{
                pwm_L = 10;
                pwm_R = 10;
                mLeftWheel->setPWM(pwm_L);
                mRightWheel->setPWM(pwm_R);
            }

            if (wait_event()) {
            ext_tsk();
            }
        }

        count = 0;

        int x = (TAIL_ANGLE_stargazer - angle) * 20;
        Current_taile = TAIL_ANGLE_stargazer;

        while(count++ < x){

            if(count % 20 == 0)  {
                Current_taile -= 1;
            }

            tail_control(Current_taile);

            pwm_L = 10;
            pwm_R = 10;

            mLeftWheel->setPWM(pwm_L);
            mRightWheel->setPWM(pwm_R);

            if (wait_event()) {
            ext_tsk();
        }
        }
        Current_taile = angle;
        plus = 0;
        break;
    }
    return 0;
}
int RunMethod::TailRun_Kai(signed char forward, int dist, int starangle){
    start_dist = mRloc->distance;
    float start_omega = mRloc->omega_d;
    int speed;      
    while ( (mRloc->distance - start_dist) < dist) {
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(starangle); 

        if((mRloc->omega_d - start_omega)>5){
            speed = 0;
            StopHere(0,5);
            SpinTop(-5);
        } else if((mRloc->omega_d - start_omega)<-5){
            speed = 0;
            StopHere(0,5);
            SpinTop(5);
        } else{
            speed = forward;
        }
        mLeftWheel->setPWM(speed);
        mRightWheel->setPWM(speed);
        if (wait_event()) {
            ext_tsk();
        }
    }
    return 0;
}



