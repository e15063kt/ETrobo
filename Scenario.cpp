#include "Scenario.h"
using namespace ev3api;
extern Result result[10];
extern int wait_event(void);
//extern	EvLog*	elog;


Scenario::Scenario(RunPattern* runP, RoboLoc* rloc, Block_area* barea)
	: mRP(runP), mRloc(rloc), mBArea(barea) {
        block_ptn = 5;
}

void Scenario::R(){
    // タイム計測
    mRP->setStartTime();
    // Rコース
    // スタート
    mRP->set_pid_params(0.00, 0.02, 0.0);
    mRP->run_pid(1, 30, 150);

    // for CS
    // // 直進1
    mRP->set_pid_params(2.4, 0.16, 0.004);
    mRP->run_pid(1, 80, 1950);
    ev3_speaker_play_tone(NOTE_A4, 300);
    // カーブ1
    mRP->set_pid_params(2.4, 0.03, 0.12);
    mRP->run_pid(1, 75, 3300);
    mRP->set_pid_params(3.4, 0.0, 0.1);
    mRP->run_pid(1, 60, 400);
    ev3_speaker_play_tone(NOTE_A4, 300);
    // カーブ2
    mRP->set_pid_params(2.45, 0.0025, 0.1);
    mRP->run_pid(1, 45, 1500);
    // mRP->set_pid_params(3.6, 0.04, 0.02);
    // mRP->run_pid(1, 50, 1500);
    mRP->set_pid_params(2.0, 0.04, 0.12);
    mRP->run_pid(1, 70, 1300);
    ev3_speaker_play_tone(NOTE_A4, 300);
    // 直進2
    mRP->reset_pid_params();
    mRP->set_pid_params(2.4, 0.12, 0.008);
    // mRP->run_pid(1, 80, 1580);   // 試走会にて20cmオーバー
    mRP->run_pid(1, 80, 1500);
    ev3_speaker_play_tone(NOTE_A4, 300);
    mRP->set_pid_params(0.0, 0.5, 0.0);
    mRP->run_pid(3, 45, 200);
    // カーブ3
    mRP->set_pid_params(2.4, 0.0, 0.1);
    // mRP->run_pid(1, 40, 1320);   // 試走会にて数cmオーバー
    mRP->run_pid(1, 40, 1200);
    mRP->set_pid_params(0.5, 0.05, 0.1);


    // ET相撲NEO
    mRP->flag_NEO = true;

    mRP->eyesight(30);
    mRP->AcrossTheLine(0, 258);
    mRP->TurnR90();

    // 1
    switch (mRP->RingDetect()) {
        case 0: // 押出
            mRP->move(30, 0, 100);
            mRP->move(-30, 1.95, 150);
            mRP->TurnR180();
            break;
        case 1: // ブロックの色を読まず押出
            mRP->move(30, 0, 87);
            mRP->move(-30, 1.95, 160);
            mRP->TurnR180();
            break;
        case 2: // ブロックの色を読まず寄切
        default:// 寄切
            mRP->SetTailAngle(60, -50);
            mRP->TurnR180();
            mRP->move(30, 0.5, 30);
            mRP->SetTailAngle(60, 50);
            break;
    }
    // 2
    switch (mRP->RingDetect()) {
        case 0: // 押出
            mRP->move(30, 0, 100);
            mRP->move(-30, 0, 175);
            mRP->TurnR90();
            break;
        case 1: // ブロックの色を読まず押出
            mRP->move(30, 0, 90);
            mRP->move(-30, 0, 175);
            mRP->TurnR90();
            break;
        case 2: // ブロックの色を読まず寄切
        default:// 寄切
            mRP->move(30, 0, 10);
            mRP->TurnL180();
            mRP->move(30, 0, 95);
            mRP->TurnL80();
            break;
    }

    mRP->run_pid(4, 35, 284);
    mRP->TurnR90();

    // 3
    switch (mRP->RingDetect()) {
        case 0: // 押出
            mRP->move(30, 0, 100);
            mRP->move(-30, 0.0, 150);
            mRP->TurnL180();
            break;
        case 1: // ブロックの色を読まず押出
            mRP->move(30, 0, 90);
            mRP->move(-30, 1.8, 160);
            mRP->TurnL180();
            break;
        case 2: // ブロックの色を読まず寄切
        default:// 寄切
            mRP->SetTailAngle(60, -115);
            mRP->TurnL180();
            mRP->move(30, 2.2, 30);
            mRP->SetTailAngle(60, 115);
            break;
    }
    // 4
    switch (mRP->RingDetect()) {
        case 0: // 押出
            mRP->move(30, 0, 100);
            mRP->move(-30, 0, 160);
            mRP->TurnR90();
            mRP->move(-10, -0.2, 15);
            break;
        case 1: // ブロックの色を読まず押出
            mRP->move(30, 0, 90);
            mRP->move(-30, 0, 170);
            mRP->TurnR90();
            mRP->move(-10, -0.2, 15);
            break;
        case 2: // ブロックの色を読まず寄切
        default:// 寄切
            mRP->SetTailAngle(60, -35);
            mRP->TurnR180();
            mRP->move(30, 0, 75);
            mRP->TurnL90();
            mRP->move(-10,-0.2, 15);
            mRP->SetTailAngle(60, 35);
            break;
    }

    // 線路跨ぎ
    mRP->eyesight(30);
    mRP->AcrossTheLine(1, 260);
    mRP->TurnR90();

    // 5
    switch (mRP->RingDetect()) {
        case 0: // 押出
            mRP->move(30, 0, 100);
            mRP->move(-30, 1.95, 155);
            mRP->TurnR180();
            break;
        case 1: // ブロックの色を読まず押出
            mRP->move(30, 0, 90);
            mRP->move(-30, 1.95, 165);
            mRP->TurnR180();
            break;
        case 2: // ブロックの色を読まず寄切
        default:// 寄切
            mRP->SetTailAngle(60, -50);
            mRP->TurnR180();
            mRP->move(30, -1.5, 35);
            mRP->SetTailAngle(60, 50);
            break;
    }
    // 6
    switch (mRP->RingDetect()) {
        case 0: // 押出
            mRP->move(30, 0, 100);
            mRP->move(-30, 0, 175);
            mRP->TurnR90();
            break;
        case 1: // ブロックの色を読まず押出
            mRP->move(30, 0, 90);
            mRP->move(-30, 0, 175);
            mRP->TurnR90();
            break;
        case 2: // ブロックの色を読まず寄切
        default:// 寄切
            mRP->move(30, 0, 10);
            mRP->TurnL180();
            mRP->move(30, 0, 95);
            mRP->TurnL80();
            break;
    }

    mRP->run_pid(4, 35, 284);
    mRP->TurnR90();

    // 7
    switch (mRP->RingDetect()) {
        case 0: // 押出
            mRP->move(30, 0, 100);
            mRP->move(-30, 0, 150);
            mRP->TurnL180();
            break;
        case 1: // ブロックの色を読まず押出
            mRP->move(30, 0, 80);
            mRP->move(-30, 1.8, 150);
            mRP->TurnL180();
            break;
        case 2: // ブロックの色を読まず寄切
        default:// 寄切
            mRP->SetTailAngle(60, -115);
            mRP->TurnL180();
            mRP->move(30, 2.2, 30);
            mRP->SetTailAngle(60, 115);
            break;
    }
    // 8
    switch (mRP->RingDetect()) {
        case 0: // 押出
            mRP->move(30, 0, 100);
            mRP->move(-30, 0, 160);
            mRP->TurnR90();
            mRP->move(-10, -1.0, 35);
            break;
        case 1: // ブロックの色を読まず押出
            mRP->move(30, 0, 95);
            mRP->move(-30, 0, 155);
            mRP->TurnR90();
            mRP->move(-10, -1.0, 35);
            break;
        case 2: // ブロックの色を読まず寄切
        default:// 寄切
            mRP->SetTailAngle(60, -35);
            mRP->TurnR180();
            mRP->move(30, 0, 80);
            mRP->TurnL90();
            mRP->move(-10, 1.5, 10);
            mRP->SetTailAngle(60, 35);
            break;
    }

    // 懸賞
    // mRP->KenShow();

    mRP->eyesight(30);
    mRP->AcrossTheLine(2, 100);

    mRP->flag_NEO = false;
    mRP->reset_pid_params();
    mRP->run_pid(0, 30, 200);
    mRP->run_pid(0, 45, 250);
    mRP->TurnR(85);
    mRP->run_pid(1, 45, 430);
    mRP->banzai();
}

void Scenario::L(){
    //b_area->test2(gBT->get_bt());
	// Lコース
	// スタート
    //mRP->TurnR(90);


	mRP->set_pid_params(0.0, 0.0, 0.0);
	mRP->run_pid(0, 30, 100);
	// 直進1
	mRP->set_pid_params(0.6, 0.1, 0.05);
	mRP->run_pid(0, 61, 1950);//60
	// 第１コーナー
	mRP->set_pid_params(1.8, 0.00, 0.1);
	mRP->run_pid(0, 44, 1850);
    // 第２コーナー
	mRP->set_pid_params(1.6, 0.01, 0.1);
	mRP->run_pid(0, 55, 3000);	//速度余裕有


    // 第３コーナー
	mRP->set_pid_params(1.6, 0.00, 0.1);
	mRP->run_pid(0, 43, 500);
	// 第4コーナー
    mRP->set_pid_params(1.6, 0.00, 0.1);
	mRP->run_pid(0, 44, 1100);//44

	// 直進2
    mRP->reset_pid_params();
    //mRP->set_pid_params(2.2, 1.8, 0.05);
    //mRP->run_pid(1, 80, 1500);
	mRP->set_pid_params(0.6, 0.1, 0.05);
	mRP->run_pid(0, 61, 1500);// 60, 1300
    //試走会にて200mmオーバー

    mRP->move(30, 2, 100);
    //灰色
    //mRP->set_pid_params(0.0, 0.0, 0.0);
    //mRP->run_pid(0, 35, 200);
    //エッジ切り替え1,40
    //mRP->run_pid(1, 20, 100);

    
    //Reset PID
    mRP->reset_pid_params();
    mRP->set_pid_params(0.6, 0.1, 0.1);
    mRP->run_pid(1, 30, 100);
    
    DoPuzzle();
}

void Scenario::RGB2HSV(void){
    int color;
    mRP->SetArmAngle(30, 45);
    color = mRP->RGB2HSV();

    switch(color) {
        case 0:;
        case 1:;
        case 2:;
        case 3:;
        case 4:;
        case 5:;
        case 6:;
        default:;
    }
}

void Scenario::TEST(void){
    // mRP->SetArmAngle(80, -20); //-20
    // mRP->set_pid_params(0.4, 0.15, 0.006);
    // mRP->ColorDetect(0);
    // mRP->Turn(-30);
    // mRP->ColorDetect(0);
    // mRP->Turn(-90);
    // mRP->ColorDetect(0);
    // mRP->Turn(-30);
    // mRP->ColorDetect(0);
    // mRP->Turn(-60);
    // mRP->ColorDetect(0);
    // mRP->Turn(-120);
    // mRP->ColorDetect(0);
    // mRP->Turn(-60);
    // while(1){
    // mRP->ColorDetect(0);
    // mRP->Turn(-120);
    // mRP->ColorDetect(0);
    // mRP->Turn(-60);
    mRP->flag_NEO = true;
    mRP->eyesight(30);
}

void Scenario::DoPuzzle(){
    mRP->reset_pid_params();
    //パズル侵入
    mRP->set_pid_params(1., 1.7, 0.01);
    mRP->SetArmAngle(80, -20); //-20
    mRP->ColorDetect(0);
    mRP->move(-20, 0, 120);
    mRP->reset_pid_params();
    mRP->set_pid_params(0.4, 0.15, 0.006);


    //入口選択
//    block_ptn = mBArea->select(10000);
    block_ptn = mBArea->select();

    //配列の2番目から入るリングを取得
    if (result[block_ptn].orders[1] == 10){
        mRP->ColorDetect(0);
    } else {
        mRP->TurnR(30);
        mRP->LineDetect(1);
        mRP->ColorDetect(0);

    }

    //ここからパズル（予定
    //Block()が実際に動作し、並べ終わった後
    //ゴールエリアまでの距離を返す(int)
    int goalDist = Block();

    if(goalDist != 0){
	    //黒線を探しブロックエリアから脱出
	    mRP->LineDetect(0);

	    //PIDを使い、車庫前まで移動
	    mRP->set_pid_params(1.8, 0.34, 0.1);    //1.5, 0.24, 0.1
	    mRP->run_pid(16, 40, goalDist);

	    mRP->Chusya();
	}
}

int Scenario::Block(void){
    //mRP->SetArmAngle(30, -30);  //pwm, angle 30 -20
    //mRP->set_pid_params(0.4, 0.1, 0.0);   //0.4 0.1 0
    //mRP->ColorDetect(0);
    //mRP->Turn(-90);
    int i = 3;
    char time[30];
    // iは3づつ増える
    // orders[0]には全体の長さの３の倍数になってる
    while (i < result[block_ptn].orders[0] - 3){
        switch(result[block_ptn].orders[i]){
            case 110:   //TURN
                ev3_speaker_play_tone(NOTE_AS4, 300);
                sprintf(time,"Turn :%3d :%2d", result[block_ptn].orders[i + 1], result[block_ptn].orders[i + 2]);
                ev3_lcd_draw_string(time,0,60);
                mRP->Turn(result[block_ptn].orders[i + 1]);
                break;
            case 111:   //TURN_BLOCK
                ev3_speaker_play_tone(NOTE_D5, 300);
                sprintf(time,"Turn_B :%3d :%2d", result[block_ptn].orders[i + 1], result[block_ptn].orders[i + 2]);
                ev3_lcd_draw_string(time,0,60);
                mRP->TurnWithBlock(result[block_ptn].orders[i + 1]);
                break;
            case 112:   //FORWARD
                ev3_speaker_play_tone(NOTE_F5, 300);
                sprintf(time,"Forward :%3d :%2d", result[block_ptn].orders[i + 1], result[block_ptn].orders[i + 2]);
                ev3_lcd_draw_string(time,0,60);
                mRP->ColorDetect(0);
                break;
            case 113:   //FORWARD_BLOCK
                ev3_speaker_play_tone(NOTE_A5, 300);
                sprintf(time,"Forward_B :%3d :%2d", result[block_ptn].orders[i + 1], result[block_ptn].orders[i + 2]);
                ev3_lcd_draw_string(time,0,60);
                if (result[block_ptn].orders[i + 2])
                    mRP->ColorDetect(0);
                else
                    mRP->move(22, -2, 230);
                break;
            case 114:   //RELEASE
                ev3_speaker_play_tone(NOTE_C6, 300);
                sprintf(time,"Release :%3d :%2d", result[block_ptn].orders[i + 1], result[block_ptn].orders[i + 2]);
                ev3_lcd_draw_string(time,0,60);
                (result[block_ptn].orders[i + 2]) ? mRP->Release(0) : mRP->Release(1);
                break;
            default:    //END
                ev3_speaker_play_tone(NOTE_B6, 300);
                sprintf(time,"ERROR! :%3d :%2d", result[block_ptn].orders[i + 1], result[block_ptn].orders[i + 2]);
                ev3_lcd_draw_string(time,0,60);
                break;
        }
        i = i + 3;
        if (wait_event())   ext_tsk();
    }

    if (result[block_ptn].orders[i] == 120){
    		return 0;
    } else if(result[block_ptn].orders[i+1] > 300){ //298 or 317
        return 450;
    } else {
        return 560;
    } 
}

void Scenario::L_SHORT(){
    //b_area->test2(gBT->get_bt());
    // Lコース
    // スタート
    //mRP->TurnR(90);
    mRP->SetArmAngle(30, -30);  //pwm, angle
    mRP->set_pid_params(0.6, 0.1, 0.05);

    mRP->ShortCut();
    DoPuzzle();
}