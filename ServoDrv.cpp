//------------------------------------------------------------------------------
// ServoSequenceクラス
//------------------------------------------------------------------------------
#include <arduino.h>
#include "ServoDrv.h"

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// コンストラクタ
ServoDriver::ServoDriver()
{
  // 何もしない
}
ServoDriver::Set()
{
  pinMode(port, OUTPUT);
  digitalWrite(port, HIGH);

  lPW.onDeg = map((float)onDeg,offDeg,onDeg,MinAngle,MaxAngle);    // ON時の角度
  lPW.offDeg = map((float)offDeg,offDeg,onDeg,MinAngle,MaxAngle);   // OFF時の角度
  lPW.onDelta = mapfloat(((float)(onDeg - offDeg) / onSpeed / 100),(float)offDeg,(float)onDeg,0,(float)(MaxAngle-MinAngle)); // offDegからonDegまでの10ms時の移動量を算出
  lPW.offDelta = mapfloat(((float)(offDeg - onDeg) / offSpeed / 100),(float)offDeg,(float)onDeg,0,(float)(MaxAngle-MinAngle)); // offDegからonDegまでの10ms時の移動量を算出
  lPW.nowDeg = 0;

  ctState = sdir;

  SERVO.attach(port, MinAngle, MaxAngle);
  delay(20);
  SERVO.detach();
  
  state = ST_STANDABY;
}

int ServoDriver::nowState()
{
  return state; 
}

void ServoDriver::writeCV(void)
{
  Dcc.setCV(cv,ctState);        // 最終値のアクセサリ番号をCV_sdirに書き込み
}

void ServoDriver::SVattach( void )
{
  SERVO.attach(port, MinAngle, MaxAngle);
}

void ServoDriver::ServoWrite(int ref)
{
  SERVO.writeMicroseconds(ref);
}

void ServoDriver::SVdetach( void )
{
  SERVO.detach();  
}

void ServoDriver::led( void )
{
  if(ctState == 0){ //0:div/ 1:str|
    digitalWrite(STR,OFF);
    digitalWrite(DIV,ON);
  } else {
    digitalWrite(STR,ON);
    digitalWrite(DIV,OFF);
  }
}

// c:closed 直線　t:thrown 分岐
void ServoDriver::change(unsigned char ct)
{
  ctState = ct; 
}

// ServoSequence ステートマシン（状態遷移）
void ServoDriver::stateCheck()
{      
  switch(state){
      case ST_STANDABY:               // 起動時一回だけの処理
        led();
        if(ctState == sdir ){     // 前回最後のSTR/DIVが同じ？
          if(ctState == 0){           // OFF?(c:直線)
            SVattach();
            lPW.nowDeg = lPW.offDeg;
            ServoWrite((int)lPW.nowDeg);
          } else {                    // ON?
            SVattach();
            lPW.nowDeg = lPW.onDeg;
            ServoWrite((int)lPW.nowDeg);
          }
          SVdetach();
          state = ST_IDLE;
          break;
        } else { // EEPROMの状態とコマンドステーションが異なっていた時の初回処理
          if(sdir != 0 and ctState == 0){
              nextDeg = lPW.offDeg;
              nextDelta = lPW.offDelta;
              state = ST_RUN;         
          } else {
              nextDeg = lPW.onDeg;
              nextDelta = lPW.onDelta;
              state = ST_RUN;
          }
        }
        break;

      case ST_IDLE:
            if(ctState == 0 ){           // ServoB:OFF
              if(lPW.nowDeg == lPW.offDeg){   // 最終値まで行っていたら抜ける
                state = ST_IDLE;
                return;
              }
              SVattach();
              nextDeg = lPW.offDeg;
              nextDelta = lPW.offDelta;
            } else if(ctState != 0 ){    // ServoB:ON
              if(lPW.nowDeg == lPW.onDeg){    // 最終値まで行っていたら抜ける
                state = ST_IDLE;
                return;
              }
              SVattach();
              nextDeg = lPW.onDeg;
              nextDelta = lPW.onDelta;
            }
                  
            if(lPW.nowDeg - nextDeg < 0){
              updownFlg = UP;
            } else {
              updownFlg = DOWN;
            }
            state = ST_RUN;
      
            break;

    case ST_RUN:
            ServoWrite((int)lPW.nowDeg);
            lPW.nowDeg = lPW.nowDeg + nextDelta;

            if( ((updownFlg == DOWN) && (lPW.nowDeg <= nextDeg)) || ((updownFlg == UP) && (lPW.nowDeg >= nextDeg)) ) {       // 下りONまで行った？ or 上りONまで行った？
              lPW.nowDeg = nextDeg;
              ServoWrite((int)nextDeg);
              writeCV();
              SVdetach();
              led();
              state = ST_IDLE;
            }
            break;
      default:
            break;
  }   
}
