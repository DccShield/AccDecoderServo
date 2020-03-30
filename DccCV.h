#ifndef _DccCV_h_
#define _DccCV_h_

//#define DECODER_ADDRESS 3
#define DECODER_ADDRESS 1   // アクセサリなのでアドレスは1


#define ON 1
#define OFF 0
//#define UP 1
//#define DOWN 0

#define CV_VSTART    2
#define CV_ACCRATIO   3
#define CV_DECCRATIO  4
#define CV_F0_FORWARD 33
#define CV_F0_BACK 34
#define CV_F1 35
#define CV_F2 36
#define CV_F3 37
#define CV_F4 38
#define CV_F5 39
#define CV_F6 40
#define CV_F7 41
#define CV_F8 42
#define CV_F9 43
#define CV_F10 44
#define CV_F11 45
#define CV_F12 46
#define CV_49_F0_FORWARD_LIGHT 49
#define CV_DIMMING_SPEED 50
#define CV_DIMMING_LIGHT_QUANTITY 51
#define CV_ROOM_DIMMING 52

#define CV_zeroDeg 60       // 0deg時のPWMの値
#define CV_ninetyDeg 61     // 90deg時のPWMの値
#define CV_onDeg 62         // on時の角度
#define CV_offDeg 63        // off時の角度
#define CV_initDeg 64       // 起動時の角度
#define CV_onSpeed 65       // off->onに移行する時間
#define CV_offSpeed 66      // on->offに移行する時間
#define CV_sdir 67          // 最後のdir保持用
#define CV_function 68   // サーボモータファンクッション番号
#define CV_dummy 69         // dummy
#define MAN_ID_NUMBER 166   // Manufacture ID //
#define MAN_VER_NUMBER  01  // Release Ver CV07 //

//Task Schedule
unsigned long gPreviousL5 = 0;

//進行方向
uint8_t gDirection = 128;

//モータ制御関連の変数
uint32_t gSpeedRef = 1;

uint16_t gAccessoryAddress = 1;
uint16_t gAccessoryAddress2 = 0;

uint8_t  gCTevent = 0;
uint16_t gAccAdr = 0;
uint16_t gDir = 0;




//CV related
uint8_t gCV1_SAddr = 3; 
uint8_t gCVx_LAddr = 3;
uint8_t gCV29_Conf = 0;
uint8_t gCV68_servoAdder = 4; // サーボアドレス
uint8_t gCV49_fx = 20;

uint8_t ServoEnable = 0;
  
#endif
