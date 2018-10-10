/**
 ******************************************************************************
 ** ファイル名 : app.c
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用Cサンプルプログラム
 **
 ** 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include <string.h>
#include <syssvc/serial.h>
#include "BalancerCpp.h"        // <1>
#include "balancer.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "Evlog.h"
#include "Location.h"
#include "Pid.h"
#include "RunMethod.h"

using namespace ev3api;

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    sonar_sensor    = EV3_PORT_2,
    color_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B,
    tail_motor      = EV3_PORT_A;

static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define GYRO_OFFSET  -1          /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
//#define LIGHT_WHITE  40         /* 白色の光センサ値 */
//#define LIGHT_BLACK  0         /* 黒色の光センサ値 */
/* sample_c2マクロ */
#define SONAR_ALERT_DISTANCE 10 /* 超音波センサによる障害物検知距離[cm] */
/* sample_c3マクロ */
#define TAIL_ANGLE_STAND_UP 84 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      0 /* バランス走行時の角度[度] 3 */
#define P_GAIN             2.5F /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60 /* 完全停止用モータ制御PWM絶対最大値 */
/* sample_c4マクロ */
//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START         '1'    /* リモートスタートコマンド */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* 関数プロトタイプ宣言 */
int sonar_alert(void);
void tail_control(signed int angle);
int wait_event(void);
//static int runonoff(int a, signed char b, int c);

TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;
EvLog*          elog;
Balancer*       balancer;              // <1>
RoboLoc*        gRloc;
Pid*            pid;
RunMethod*      gRunMethod;

void ev3_cyc_tracer(intptr_t exinf){
//    wup_tsk(DRIVE_TASK);
}

int eflag = 0;
int LIGHT_WHITE;
int LIGHT_BLACK;
int sflag = 0;
/* メインタスク */
void main_task(intptr_t unused)
{
    balancer = new Balancer();
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_2);
    colorSensor = new ColorSensor(PORT_3);
    gyroSensor = new GyroSensor(PORT_4);
    leftMotor = new Motor(PORT_C);
    rightMotor = new Motor(PORT_B);
    tailMotor = new Motor(PORT_A);
    clock = new Clock();
    gRloc = new RoboLoc(50., 90., 1.0);
    pid = new Pid();
    gRunMethod = new RunMethod(tailMotor, leftMotor, rightMotor, gRloc, balancer, gyroSensor, colorSensor);
    elog = new EvLog(clock, colorSensor, sonarSensor,
        gyroSensor, tailMotor, leftMotor, rightMotor);

    /* LCD画面表示 */
    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

    //尻尾リセット
    tailMotor->reset();

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */


    // 走行モーターエンコーダーリセット
    leftMotor->reset();
    rightMotor->reset();

    // スタート待機
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

        if(sflag == 0){
            int num = 0;
            int WHITE = 0, WHITE2 = 0;
            int BLACK = 0, BLACK2 = 0;
            char mess2[100];
        while(num <= 5){
            
            char mess[100] = {0};
            tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */
            int LIGHT = colorSensor->getBrightness();
            sprintf(mess, "ColorReflect = %2d", LIGHT);
            ev3_lcd_draw_string(mess, 4, 15);

            if(touchSensor->isPressed() == true || num == 5){
                if(num < 4) num++;
                
                if(num == 1){
                    WHITE = colorSensor->getBrightness();
                    ev3_speaker_play_tone(NOTE_C5, 300);
                    clock->sleep(500);
                }else if(num == 2){
                    BLACK = colorSensor->getBrightness();
                    ev3_speaker_play_tone(NOTE_D5, 300); 
                    clock->sleep(500);
                }else if(num == 3){
                    WHITE2 = colorSensor->getBrightness();
                    ev3_speaker_play_tone(NOTE_E5, 300);
                    clock->sleep(500);
                }else if(num == 4){
                    BLACK2 = colorSensor->getBrightness();
                    ev3_speaker_play_tone(NOTE_F5, 300);
                    clock->sleep(500);
                	num ++;
                }else{
                    LIGHT_WHITE = (WHITE + WHITE2) / 2;
                    LIGHT_BLACK = (BLACK + BLACK2) / 2;
                    num++;
                }
            }
            clock->sleep(10);
        }
            sprintf(mess2, "WHITE = %d", LIGHT_WHITE);
            ev3_lcd_draw_string(mess2, 38, 35);
            sprintf(mess2, "BLACK = %d", LIGHT_BLACK);
            ev3_lcd_draw_string(mess2, 38, 55);
            ev3_speaker_play_tone(NOTE_G5, 300);
            sflag = 1;

            gRunMethod->ThresholdTransfer(LIGHT_WHITE, LIGHT_BLACK);
        }

        if (bt_cmd == 1)  {
            ev3_speaker_play_tone(NOTE_C6, 500);
            break;
        }

         /* リモートスタート */
        ev3_led_set_color(LED_OFF); /* 初期化完了通知 */

        if(ev3_button_is_pressed(RIGHT_BUTTON)) {
        	sflag = 0;
        	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
        	ev3_speaker_play_tone(NOTE_A5, 300);
        	clock->sleep(500);
        	ev3_speaker_play_tone(NOTE_B5, 300);
        	clock->sleep(500);
        	ev3_speaker_play_tone(NOTE_C6, 300);
        	clock->sleep(500);
        	
        	
        }


        if (sonar_alert() == 1){
            ev3_speaker_play_tone(NOTE_C6, 500);
            break;
        } 
/*
        if (touchSensor->isPressed() == true){

        }*/
//        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)    break; /* タッチセンサが押された */

        //tslp_tsk(10); /* 10msecウェイト */
        clock->sleep(10);
   
    }


    /* ジャイロセンサーリセット */
    gyroSensor->reset();
    balancer->init(GYRO_OFFSET);

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    //周期ハンドラ起動
//    ev3_sta_cyc(EV3_CYC_TRACER);
    act_tsk(WUP_TASK);
    ev3_led_set_color(LED_OFF);
//    wup_tsk(DRIVE_TASK);

//    slp_tsk();

    while (eflag == 0){
        elog->output(0);
        clock->sleep(50);
    }

    ter_tsk(WUP_TASK);
//    ev3_stp_cyc(EV3_CYC_TRACER);
    ev3_led_set_color(LED_GREEN);

    elog->output(1);
    leftMotor->reset();
    rightMotor->reset();
    //bt_transfer();
    ter_tsk(BT_TASK);
    fclose(bt);
    ext_tsk();
}

void wup_task(intptr_t exinf){
    while(1){
        if (ev3_button_is_pressed(BACK_BUTTON)){
            eflag = 1;
            break;
        }
        wup_tsk(DRIVE_TASK);
        clock->sleep(4);
    }
}

void drive_task(intptr_t exinf){
    slp_tsk();
    while(1){
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        //シナリオスタート
        gRunMethod->Rocket_S(100);
        //gRunMethod->RunOnOff(0,100,100);
        //直１
        gRunMethod->SetPid(0.65,0.0015,0.1);
        gRunMethod->RunPid(0,100,50,0,3);//(2.2)
ev3_speaker_play_tone(NOTE_C6, 300);//音１
        gRunMethod->SetPid(0.55,0.0015,0.1);
        gRunMethod->RunPid(0,100,1800,0,3.5);//(2.2)
        
        gRunMethod->SetPid(0.65,0.0015,0.1);
        gRunMethod->RunPid(0,90,200,0,3);//(2.2)
ev3_speaker_play_tone(NOTE_C6, 300);//音2
         //カーブ1
        gRunMethod->SetPid(0.7,0.0015,0.11);//p0.8  i0.0015  d0.11
        gRunMethod->RunPid(0,90,750,0,2);
        gRunMethod->RunPid(0,90,1000,0,3);

ev3_speaker_play_tone(NOTE_C6, 300);//音3
        //直２
        gRunMethod->SetPid(0.65,0.0015,0.11);
        gRunMethod->RunPid(0,90,480,0,1);//dis 575

ev3_speaker_play_tone(NOTE_C6, 300);//音4
         //Sカーブ
        gRunMethod->SetPid(0.8,0.001,0.12);
        gRunMethod->RunPid(0,80,570,0,1);//600
ev3_speaker_play_tone(NOTE_C6, 300);//音5
        gRunMethod->SetPid(1.1,0.001,0.12);
        gRunMethod->RunPid(0,80,630,0,1);//530
ev3_speaker_play_tone(NOTE_C6, 300);//音6
        //直3
        gRunMethod->SetPid(0.6,0.0015,0.1);
        gRunMethod->RunPid(0,90,700,0,1.5);
ev3_speaker_play_tone(NOTE_C6, 300);//音7
        //カーブ3
        gRunMethod->SetPid(0.9,0.0015,0.11);
        gRunMethod->RunPid(0,80,650,0,1);
        gRunMethod->RunPid(0,80,500,0,1);


ev3_speaker_play_tone(NOTE_C6, 300);//音8
        //ラストスパート dis 3600
        gRunMethod->SetPid(0.60,0.0015,0.11);
        gRunMethod->RunPid(0,100,1100,0,3.5);//1300
        gRunMethod->RunPid(0,100,900,0,2);//1000
        gRunMethod->RunPid(0,80,530,0,1);//540(go-ru)


        //go-lu
        //カーブ
ev3_speaker_play_tone(NOTE_C6, 300);
        gRunMethod->SetPid(1.1,0.001,0.12);
        gRunMethod->RunPid(0,50,240,0,0);//250
        gRunMethod->RunPid(0,20,260,0,0);//250
ev3_speaker_play_tone(NOTE_C6, 300);
        //灰色
        gRunMethod->SetPid(0.7,0.0015,0.1);
        gRunMethod->RunPid(0,10,200,25,-1);//170
        ev3_speaker_play_tone(NOTE_C6, 300);
        //灰色あと
ev3_speaker_play_tone(NOTE_C6, 300);

        //noru
        //gRunMethod->StarGazer(0,80,80);
        //gRunMethod->TailStopHere(500);
        gRunMethod->TailRun(230,65,65,83);
        gRunMethod->TailStopHere(200);
        //siso
        gRunMethod->TailStopHere(400);


        int gyro_start = gyroSensor->getAnglerVelocity();
        while(1){
            gRunMethod->TailRun(10,5,5,85);
            if(gyroSensor->getAnglerVelocity() - gyro_start > 30)
                break;
        }
ev3_speaker_play_tone(NOTE_C6, 300);
        gRunMethod->TailRun(50,7,7,35);
        gRunMethod->TailRun(300,0,0,35);

        gRunMethod->TailRun(130,-15,-15,35);
        gRunMethod->TailStopHere(200);

        gyro_start = gyroSensor->getAnglerVelocity();
        while(1){
            gRunMethod->TailRun(10,-13,-13,35);
            if(gyroSensor->getAnglerVelocity() - gyro_start < -45)//50
                break;
        }
ev3_speaker_play_tone(NOTE_C6, 300);

        gRunMethod->TailRun(100,-23,-23,85);
        gRunMethod->TailRun(500,0,0,85);

        gyro_start = gyroSensor->getAnglerVelocity();
        while(1){
            gRunMethod->TailRun(10,5,5,85);
            if(gyroSensor->getAnglerVelocity() - gyro_start > 30)
                break;
        }
ev3_speaker_play_tone(NOTE_C6, 300);
        gRunMethod->TailRun(50,7,7,35);
        gRunMethod->TailRun(100,0,0,35);

        gRunMethod->TailRun(100,50,50,60);
        gRunMethod->TailRun(50,20,20,60);
        gRunMethod->TailRun(200,0,0,60);
       
        gRunMethod->TailStopHere(1000);

        




       //stop
        
        



    
        
        





    
        //灰色

        break;
    }
    eflag = 1;
    ext_tsk();
}


int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void tail_control(signed int angle)
{
    float pwm = (float)(angle - ev3_motor_get_counts(tail_motor))*P_GAIN; /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    if (pwm == 0)
    {
        ev3_motor_stop(tail_motor, true);
    }
    else
    {
        ev3_motor_set_power(tail_motor, (signed char)pwm);
    }
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }
        fputc(c, bt); /* エコーバック */
    }
}

#define LN 1 // ログ収集周期、値が1は毎回収集
int count = 0;
int wait_event(void){
    if (ev3_button_is_pressed(BACK_BUTTON)) {
        eflag = 1;          // 終了フラグセット
        wup_tsk(MAIN_TASK); // バックボタン押下
    }
    if ((count++ % LN) == 0){
        //elog->input(gRloc->omega_d, 0, (int) gRloc->distance, 0);
    	elog->input(balancer->mForward, balancer->mTurn, balancer->mRightPwm, balancer->mLeftPwm);
    }
    gRloc->input(leftMotor->getCount(), rightMotor->getCount()); // 自己位置計算
    slp_tsk();
    return eflag;
}

/*
static int runonoff(int edge, signed char forward, int dist){
    int         start_dist = gRloc->distance;
    signed char turn;         // 旋回命令 
    signed char pwm_L, pwm_R; // 左右モータPWM出力
    while((gRloc->distance - start_dist) < dist){
        int32_t motor_ang_l, motor_ang_r;
        int gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); // バランス走行用角度に制御 

        if (sonar_alert() == 1) {// 障害物検知 
            forward = turn = 0; // 障害物を検知したら停止 
        } else {
            if (colorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK)/2){
                turn =  20; // 左旋回命令 
            } else {
                turn = -20; // 右旋回命令 
            }
        }

        // 倒立振子制御API に渡すパラメータを取得する 
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        // 倒立振子制御APIを呼び出し、倒立走行するための 
        // 左右モータ出力値を得る 
        balancer->setCommand(forward, turn);   // <1>
        balancer->update(gyro, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = balancer->getPwmRight();       // <3>
        pwm_R = balancer->getPwmLeft();        // <3>
        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);


        if (wait_event()) {
            ext_tsk();
        }

        //tslp_tsk(4); // 4msec周期起動 
        //slp_tsk();
    }
    return 0;
}
*/

