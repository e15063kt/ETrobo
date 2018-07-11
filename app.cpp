
// 力ラーセンサーRGB RAW値取得サンプルプログラム。

#include "ev3api.h"
#include "app.h"
#include <string.h>
#include <syssvc/serial.h>
#include "TouchSensor.h"
// #include "SonarSensor.h"
//#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "Evlog.h"
#include "Location.h"
//#include "ArmControl.h"
//#include "RunMethod.h"
#include "MotorControl.h"
#include "RunPattern.h"
#include "UI.h"
#include "BT.h"
//#include "ColorLightSensor.h"
#include "Scenario.h"
#include "HSV.h"
#include "Block_area.h"

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

// 下記のマクロは個体/環境に合わせて変更する必要があります
#define GYRO_OFFSET           0  // ジャイロセンサオフセット値(角速度0[deg/sec]時)
#define SONAR_ALERT_DISTANCE 10  // 超音波センサによる障害物検知距離[cm]
#define CMD_START         '1'    // リモートスタートコマンド

// LCDフォントサイズ
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6)//magic number
#define CALIB_FONT_HEIGHT (8)//magic number

// ログ収集最大件数
#define MAX_RECORD 100000

// 関数プロトタイプ宣言
int wait_event(void);

// オブジェクトへのポインタ定義
TouchSensor*    touchSensor;
// SonarSensor*    sonarSensor;
//ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*			armMotor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;
EvLog*			elog;
RoboLoc*		gRloc;
//ArmControl*		gArmControl;
//ArmControl*		gTailControl;
MotorControl*   gMotorControl;
//RunMethod*		gRunMethod;
RunPattern*		gRunPattern;
UI*				gUI;
BT*				gBT;
//ColorLight*		gColorLight;
Scenario*		gScenario;
HSV*            gHSV;
Block_area*     gB_area;



//*****************************************************************************
// 関数名 : main_task
// 概要 : 全体の初期化と終了処理を行うタスク
//*****************************************************************************
int eflag = 0;	// 終了フラグ
int mode = 0;   // 動作モード
static int bt_cmd = 0; // Bluetoothコマンド 1:リモートスタート
void main_task(intptr_t unused)
{
    // 各オブジェクトを生成・初期化する
    touchSensor = new TouchSensor(PORT_1);
    //colorSensor = new ColorSensor(PORT_2);
    gHSV         = new HSV(PORT_2);
    // sonarSensor  = new SonarSensor(PORT_3);
    ev3_sensor_config(EV3_PORT_3, ULTRASONIC_SENSOR);
    gyroSensor   = new GyroSensor(PORT_4);
	armMotor	 = new Motor(PORT_A);
    rightMotor   = new Motor(PORT_B);
    leftMotor    = new Motor(PORT_C);
    tailMotor    = new Motor(PORT_D);
    clock        = new Clock();
	gRloc		 = new RoboLoc(50., 65., 1.0);
    gB_area       = new Block_area();
	//gArmControl  = new ArmControl(armMotor);
	//gTailControl = new ArmControl(tailMotor);
    gMotorControl = new MotorControl(armMotor, tailMotor, leftMotor, rightMotor);

	gUI = new UI();		// ユーザインタフェースオブジェクト生成
	gBT = new BT();		// ブルーツース通信オブジェクト生成

	//gRunMethod = new RunMethod(armMotor, tailMotor, leftMotor, rightMotor, gHSV, gRloc);
    gRunPattern = new RunPattern(/*sonarSensor,*/ gMotorControl, gHSV, gRloc);
	gScenario = new Scenario(gRunPattern, gRloc, gB_area);
	elog = new EvLog(clock, /*sonarSensor,*/ gyroSensor, gMotorControl, gRunPattern, gHSV, gRloc, MAX_RECORD);

    // 画面初期化
    ev3_lcd_fill_rect(0,0,EV3_LCD_WIDTH,EV3_LCD_HEIGHT,EV3_LCD_WHITE);
    ev3_lcd_draw_string("main_task", 10, 0);

	mode = gUI->exec();		// ユーザが選択したモード 0、1、2

    // アームモーター初期化
    gMotorControl->arm_init_zero();

    // Bluetooth通信タスクの起動
    wup_tsk(BT_TASK);

    // 尻尾及びアーム、走行モーターエンコーダーリセット
	armMotor->reset();
	tailMotor->reset();
    gMotorControl->wheels_reset();

    ev3_led_set_color(LED_ORANGE); // 初期化完了通知

    // スタート待機
    while(1)
    {
        if (touchSensor->isPressed()) break; // タッチセンサが押された
        // uint8_t c = fgetc(bt); // 受信
        // fputc(c, bt); //Bluetoothエコーバック
        if (bt_cmd == 1)    break;  //リモートスタート
        //if (touchSensor->isPressed()) break; // タッチセンサが押された
		tslp_tsk(10);
    }

    // ジャイロセンサーリセット
    gyroSensor->reset();

    ev3_led_set_color(LED_GREEN); // スタート通知

    ev3_lcd_fill_rect(0,0,EV3_LCD_WIDTH,EV3_LCD_HEIGHT,EV3_LCD_WHITE);

    // 周期タスク起動
    act_tsk(WUP_TASK);

    //  終了フラグを監視
	#define LOG_SAVE 0
	#define LOG_END  1

    // 50ms毎にログをファイルに書き込み
	while (gBT->eflag == 0 && (eflag == 0)) {
		elog->output(LOG_SAVE);
		clock->sleep(50);
	}

    // 尻尾及びアーム、走行モーターエンコーダーリセット
    armMotor->reset();
    tailMotor->reset();
    gMotorControl->wheels_reset();

    ter_tsk(WUP_TASK);          // 周期タスク停止
    ter_tsk(DRIVE_TASK);        // ドライブタスク終了
    ter_tsk(ULTRASONIC_TASK);   // 超音波タスク終了

    ev3_lcd_fill_rect(0,0,EV3_LCD_WIDTH,EV3_LCD_HEIGHT ,EV3_LCD_WHITE);
    ev3_lcd_draw_string("Terminate tasks...", 10, 46);

	elog->output(LOG_END);

	gBT->file_transfer();	    // ログファイル転送
    ter_tsk(BT_TASK);           //　BTタスク終了
    ext_tsk();                  // メインタスク終了
}

//*****************************************************************************
// 関数名 : wup_task
// 概要 : 定期的にタスクを起こすタスク
//*****************************************************************************
void wup_task(intptr_t exinf)
{
    while(1){
        if (ev3_button_is_pressed(BACK_BUTTON)) {
            eflag = 1;          // 終了フラグセット
            break;
        }
        wup_tsk(DRIVE_TASK);
        if (gRunPattern->flag_NEO == true)
            wup_tsk(ULTRASONIC_TASK);
        clock->sleep(4);
    }
}

//*****************************************************************************
// 関数名 : drive_task
// 概要 : 走行体を制御するタスク
//*****************************************************************************
void drive_task(intptr_t exinf)
{
    slp_tsk();
    while(1){
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        if (mode == 0) {                // R Course
            gScenario->R();
            break;
        }
        else if (mode == 1) {           // L Course
            //gUI->InputCode();
            gB_area->ttest(gUI->InputCode());
            //gB_area->test2(gBT->get_bt());
            ev3_speaker_play_tone(NOTE_F5, 300);
            while(1){
                if (touchSensor->isPressed()) break; // タッチセンサが押された
            }
            gScenario->L();
            break;
        }
        else if (mode == 2) {           // RGB to HSV
            gScenario->RGB2HSV();
            break;
        }
        else if (mode == 3) {           // TEST
            gB_area->test2(gBT->get_bt());
            ev3_speaker_play_tone(NOTE_F5, 300);
            while(1){
                if (touchSensor->isPressed()) break; // タッチセンサが押された
            }
            // gScenario->DoPuzzle();
            gScenario->TEST();
            break;
        }
        else if (mode == 4) {           // L Course
            ev3_speaker_play_tone(NOTE_F5, 300);
            while(1){
                if (touchSensor->isPressed()) break; // タッチセンサが押された
            }
            gScenario->TEST();
            break;
        }
        // slp_tsk();
    }
    eflag = 1;
    ext_tsk();			// ドライブタスク終了
}

//*****************************************************************************
// 関数名 : wait_event
// 概要 : 定期的な起床要求待つ関数
//*****************************************************************************
#define LN 1 // ログ収集周期、値が1は毎回収集
int count = 0;
int count_u = 0;
int wait_event(void) {
    ev3_lcd_draw_string("drive_task", 10, 0);

    // RGB生値取得　+　HSV変換
    gHSV->Convert(mode, gRunPattern->flag_NEO);
    // gHSV->Disp();

    // ログ収集
	if ((count++ % LN) == 0)
		elog->input(0, count, count_u, 0, gRloc->omega_d);   // (int cflag, int x1, int x2, int x3, int x4)

    // 自己位置計算
	gRloc->input(leftMotor->getCount(), rightMotor->getCount());

    // バックボタン押下時、異常色検知時停止
    if (ev3_button_is_pressed(BACK_BUTTON) || gHSV->GetColorNumber() == 8) {
    // if (ev3_button_is_pressed(BACK_BUTTON)) {
        eflag = 1;                  // 終了フラグセット
        wup_tsk(MAIN_TASK);         // メインタスク起床
    }
	slp_tsk();
	return eflag;
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるコマンド受信用のタスク
//*****************************************************************************
void bt_task(intptr_t unused) {
    gBT->open();        // ブルーツースデバイスオープン
    gBT->recv_cmd();    // コマンド受信処理
}

//*****************************************************************************
// 関数名 : ultrasonic_task
// 引数 : unused
// 返り値 : なし
// 概要 : 超音波センサ読み出しとリセットを定期的に行うタスク
//*****************************************************************************
void ultrasonic_task(intptr_t unused) {
    // char sonar[20];
    slp_tsk();
    while (1) {
        // 25ms毎に超音波測距
        if (++count_u % 5 == 0){
            ev3_lcd_draw_string("ultrasonic_task", 10, 80);
            gRunPattern->sonar_distance = ev3_ultrasonic_sensor_get_distance(EV3_PORT_3);
            // sprintf(sonar,"%3d cm", gRunPattern->sonar_distance);
            // ev3_lcd_draw_string(sonar,40,100);
            // 2000ms毎に定期的なリセット
            if (count_u % 400 == 0)
            {
                ev3_sensor_config(EV3_PORT_3, ULTRASONIC_SENSOR);
                ev3_ultrasonic_sensor_get_distance(EV3_PORT_3);
                ev3_speaker_play_tone(NOTE_F5, 50);
                count_u = 0;
            }
        }
        slp_tsk();
    }
}}