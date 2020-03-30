// DCC Accessory Decoder Servo
// 2ch サーボデコーダ
// http://dcc.client.jp
// http://ayabu.blog.shinobi.jp

#include <arduino.h>
#include <avr/eeprom.h>   //required by notifyCVRead() function if enabled below
#include <Servo.h>
#include "DccCV.h"
#include "ServoDrv.h"
#include "NmraDcc.h"


#define DEBUG      //リリースのときはコメントアウトすること
//#define DEBUG_M    //リリースのときはコメントアウトすること 速度・ファンクションパケット表示

//各種設定、宣言

#define PIN_F0_F  3     // D3 PD3,PWM
#define PIN_F0_R  4     // D4 PD4
#define PIN_AUX1  5     // D5 PD5
#define PIN_AUX2  6     // D6 PD6
#define PIN_AUX3  7     // D7 PD7
#define PIN_AUX4  8     // D8 PB0
#define PIN_AUX5  9     // D9 PB1,DIGITAL TR,PWM
#define PIN_AUX6  10    // D10 PB2,DIGITAL TR,PWM
#define PIN_AUX7  11    // D11 PB3,DIGITAL TR,PWM

#define PIN_SERVO1 5    // D5 PD5
#define PIN_SERVO2 4    // D4 PD4
#define PIN_LED_DIV1 6  // D6 PD6 SERVO1用DIV(分岐) LED
#define PIN_LED_STR1 7  // D7 PD7 SERVO1用STR(直進) LED
#define PIN_LED_DIV2 8  // D8 PB0 SERVO2用DIV(分岐) LED
#define PIN_LED_STR2 9  // D9 PB1 SERVO2用STR(直進) LED

void Dccinit(void);

//使用クラスの宣言
NmraDcc   Dcc;
DCC_MSG  Packet;

ServoDriver ServoCH0;
ServoDriver ServoCH1;

struct CVPair {
  uint16_t  CV;
  uint8_t Value;
};

CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, 1},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_29_CONFIG, 0b10000000},             // CV29 Software sw CV29＝128 アクセサリデコーダ
//  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DECODER_ADDRESS}, // CV01
//  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},               // CV09 The LSB is set CV 1 in the libraries .h file, which is the regular address location, so by setting the MSB to 0 we tell the library to use the same address as the primary address. 0 DECODER_ADDRESS
//  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},          // CV17 XX in the XXYY address
//  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},          // CV18 YY in the XXYY address
//  {CV_29_CONFIG, 128 },                                // CV29 Make sure this is 0 or else it will be random based on what is in the eeprom which could caue headaches
  {47, 180}, // ServoA ON時の角度
  {48, 0},   // ServoA OFF時の角度
  {49, 0},   // ServoA 電源切る前の角度
  {50, 5},   // ServoA OFF->ONのスピード sec
  {51, 1},   // ServoA ON->OFFのスピード sec
  {52, 0},   // ServoA gDirの最新値保存用 STR/DIV
  {53, 0},   // ServoA のファンクション番号 F0
  
  {54, 180}, // ServoB ON時の角度
  {55, 0},   // ServoB OFF時の角度
  {56, 0},   // ServoB 電源切る前の角度
  {57, 1},   // ServoB OFF->ONのスピード sec
  {58, 1},   // ServoB ON->OFFのスピード sec
  {59, 0},   // ServoB gDirの最新値保存用 STR/DIV
  {60, 1},   // ServoB のファンクション番号 F1
};

void(* resetFunc) (void) = 0;  //declare reset function at address 0

//uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
uint8_t FactoryDefaultCVIndex = 0;

void notifyDccReset(uint8_t hardReset );


void ServoInit(void)
{
  //Init CVs
  ServoCH0.ch = 0;
  ServoCH0.port = PIN_SERVO1;
  ServoCH0.onDeg = Dcc.getCV( 47 );    // ON時の角度
  ServoCH0.offDeg = Dcc.getCV( 48 );   // OFF時の角度
  ServoCH0.initDeg = Dcc.getCV( 49 );  // 電源切る前の角度
  ServoCH0.onSpeed = Dcc.getCV( 50 );  // OFF->ONのスピード
  ServoCH0.offSpeed = Dcc.getCV( 51 ); // ON->OFFのスピード
  ServoCH0.sdir = Dcc.getCV( 52 );     // gDirの最新値保存用 STR/DIV
  ServoCH0.cv = 52;
  ServoCH0.servoAddress = Dcc.getCV( 53 );// サーボモータをON/OFFするファンクッション番号
  ServoCH0.STR = PIN_LED_STR1;
  ServoCH0.DIV = PIN_LED_DIV1;
  ServoCH0.MinAngle = 670;
  ServoCH0.MaxAngle = 2600;
  ServoCH0.Set();
    
  ServoCH1.ch = 1;
  ServoCH1.port = PIN_SERVO2;
  ServoCH1.onDeg = Dcc.getCV( 54 );    // ON時の角度
  ServoCH1.offDeg = Dcc.getCV( 55 );   // OFF時の角度
  ServoCH1.initDeg = Dcc.getCV( 56 );  // 電源切る前の角度
  ServoCH1.onSpeed = Dcc.getCV( 57 );  // OFF->ONのスピード
  ServoCH1.offSpeed = Dcc.getCV( 58 ); // ON->OFFのスピード
  ServoCH1.sdir = Dcc.getCV( 59 );     // gDirの最新値保存用 STR/DIV
  ServoCH1.cv = 59;
  ServoCH1.servoAddress = Dcc.getCV( 60 );// サーボモータをON/OFFするファンクッション番号  
  ServoCH1.STR = PIN_LED_STR2;
  ServoCH1.DIV = PIN_LED_DIV2;
  ServoCH1.MinAngle = 400;
  ServoCH1.MaxAngle = 2100;
  ServoCH1.Set();

}



//------------------------------------------------------------------
// Arduino固有の関数 setup() :初期設定
//------------------------------------------------------------------
void setup()
{
  //ファンクションの割り当てピン初期化
  pinMode(PIN_F0_F, OUTPUT);
  digitalWrite(PIN_F0_F, OFF);
  pinMode(PIN_F0_R, OUTPUT);
  digitalWrite(PIN_F0_R, OFF);
  pinMode(PIN_AUX1, OUTPUT);
  digitalWrite(PIN_AUX1, OFF);
  pinMode(PIN_AUX2, OUTPUT);
  digitalWrite(PIN_AUX2, OFF);
  pinMode(PIN_AUX3, OUTPUT);
  digitalWrite(PIN_AUX3, OFF);
  pinMode(PIN_AUX4, OUTPUT);
  digitalWrite(PIN_AUX4, OFF);
  pinMode(PIN_AUX5, OUTPUT);
  digitalWrite(PIN_AUX5, OFF);
  pinMode(PIN_AUX6, OUTPUT);
  digitalWrite(PIN_AUX6, OFF);
  pinMode(PIN_AUX7, OUTPUT);
  digitalWrite(PIN_AUX7, OFF);

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Hello,Smile Function Decoder for Servo motor");
#endif
  
  Dccinit();
  ServoInit();
  
  //Reset task
  gPreviousL5 = millis();
}

//---------------------------------------------------------------------
// Arduino main loop
//---------------------------------------------------------------------
void loop() {
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())    // この処理何？ Dcc.isSetCVReady() はEEPROMが使用可能か確認,1で書き込み可能
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }

  if( gCTevent == 1 ){
//Serial.print("gAccessoryAddress:");
//Serial.println((int)gAccessoryAddress);
//Serial.print("gAccAdr):");
//Serial.println((int)gAccAdr);
    gCTevent = 0;
    if( gAccessoryAddress == gAccAdr){
        ServoCH0.change(gDir);
     }
    if( gAccessoryAddress2 == gAccAdr){
        ServoCH1.change(gDir);    
    }
  }

  if(ServoEnable == 0 && ( millis() - gPreviousL5) >=500){  // 初回起動時にDCCパケット受信していないため、E2P-ROMのSTR/DIVの不一致を検出してしまうため遅延させる
    ServoEnable = 1;
  }

  if(ServoEnable == 1){
    if ( (millis() - gPreviousL5) >= 10) // 100:100msec  10:10msec  Function decoder は 10msecにしてみる。
    {
      ServoCH0.stateCheck();
      ServoCH1.stateCheck();
      gPreviousL5 = millis();
    
    }
  }
}

//------------------------------------------------------------------
// アクセサリアドレス取得
//------------------------------------------------------------------
uint16_t getMyAddr_Acc(void)
{
  uint16_t  Addr ;
  uint8_t   CV29Value ;

  CV29Value = Dcc.getCV( CV_29_CONFIG ) ;

  if( CV29Value & 0b10000000 ) { // Accessory Decoder? 
    Addr = ( Dcc.getCV( CV_ACCESSORY_DECODER_ADDRESS_MSB ) << 8 ) | Dcc.getCV( CV_ACCESSORY_DECODER_ADDRESS_LSB ) ;
  }
  else   // Multi-Function Decoder?(アドレス4桁？)
  {
    if( CV29Value & 0b00100000 )  // Two Byte Address?
      Addr = ( Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB ) << 8 ) | Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB ) ;

    else
      Addr = Dcc.getCV( 1 ) ;
  }

  return Addr ;
}

//------------------------------------------------------------------
// アクセサリ命令のon/offの処理
// This function is called whenever a normal DCC Turnout Packet is received
//------------------------------------------------------------------
void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State)
{
  uint16_t aAccAdress = Addr;
  uint8_t aDir = (OutputAddr & 0b001);
  
    //DEBUG
    // Serial.print("notifyDccAccState: ") ;
    // Serial.print(Addr,DEC) ;
    // Serial.print(",BAdr:");
    // Serial.print(BoardAddr,DEC) ;
    // Serial.print(",OAdr:");
    // Serial.print(OutputAddr,DEC) ;
    // Serial.print(", Target Adr:");
    // Serial.print(aAccAdress,DEC) ;
    // Serial.print(",");
    // Serial.print(State, HEX) ;
    // Serial.print("==");
    // Serial.print(gAccessoryAddress,DEC) ;
    // Serial.println(",");

 
  //アドレスチェック(CVから得た11bit分の信号を比較)
//  if( gAccessoryAddress == aAccAdress){
//      ServoCH0.change(aDir);
//  }
//  if( gAccessoryAddress2 == aAccAdress){
//      ServoCH1.change(aDir);    
//  }
  gCTevent = 1;
  gAccAdr = aAccAdress;
  gDir = (OutputAddr & 0b001);
}


//---------------------------------------------------------------------
//DCC速度信号の受信によるイベント
//extern void notifyDccSpeed( uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed )
//---------------------------------------------------------------------
extern void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
}


//------------------------------------------------------------------
// CVをデフォルトにリセット(Initialize cv value)
// Serial.println("CVs being reset to factory defaults");
//------------------------------------------------------------------
void resetCVToDefault()
{
  for (int j = 0; j < FactoryDefaultCVIndex; j++ ) {
    Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
  }
};

//------------------------------------------------------------------
// CV8 によるリセットコマンド受信処理
//------------------------------------------------------------------
void notifyCVResetFactoryDefault()
{

  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
  
#if 0  
  //When anything is writen to CV8 reset to defaults.

  resetCVToDefault();
  Serial.println("Resetting...");
  delay(1000);  //typical CV programming sends the same command multiple times - specially since we dont ACK. so ignore them by delaying
  resetFunc();
#endif
};

//------------------------------------------------------------------
// CV Ackの処理
// そこそこ電流を流さないといけない
//------------------------------------------------------------------
void notifyCVAck(void)
{
//サーボモータを動かすとギミックを壊しかねないのでコメントアウト
//Serial.println("notifyCVAck");
#if 0 
  digitalWrite(O1,HIGH);
  digitalWrite(O2,HIGH);
  digitalWrite(O3,HIGH);
  digitalWrite(O4,HIGH);
  delay( 6 );
  digitalWrite(O4,LOW);
  digitalWrite(O3,LOW);
  digitalWrite(O2,LOW);
  digitalWrite(O1,LOW);
#endif
//MOTOR_Ack();
}

void MOTOR_Ack(void)
{
//  analogWrite(O4, 128);
//  delay( 6 );  
//  analogWrite(O4, 0);
}

//------------------------------------------------------------------
// DCC初期化処理）
//------------------------------------------------------------------
void Dccinit(void)
{

  //DCCの応答用負荷ピン
#if defined(DCCACKPIN)
  //Setup ACK Pin
  pinMode(DccAckPin, OUTPUT);
  digitalWrite(DccAckPin, 0);
#endif

#if !defined(DECODER_DONT_DEFAULT_CV_ON_POWERUP)
  if ( Dcc.getCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS) == 0xFF ) {   //if eeprom has 0xFF then assume it needs to be programmed
    notifyCVResetFactoryDefault();
  } else {
    Serial.println("CV Not Defaulting");
  }
#else
  Serial.println("CV Defaulting Always On Powerup");
  notifyCVResetFactoryDefault();
#endif

  // Setup which External Interrupt, the Pin it's associated with that we're using, disable pullup.
  Dcc.pin(0, 2, 0);   // ArduinoNANO D2(PD2)pinをDCC信号入力端子に設定

  // Call the main DCC Init function to enable the DCC Receiver
  // void NmraDcc::init( uint8_t ManufacturerId, uint8_t VersionId, uint8_t Flags, uint8_t OpsModeAddressBaseCV )
  // FLAGS_MY_ADDRESS_ONLY        0x01  // Only process DCC Packets with My Address
  // FLAGS_AUTO_FACTORY_DEFAULT   0x02  // Call notifyCVResetFactoryDefault() if CV 7 & 8 == 255
  // FLAGS_OUTPUT_ADDRESS_MODE    0x40  // CV 29/541 bit 6
  // FLAGS_DCC_ACCESSORY_DECODER  0x80  // CV 29/541 bit 7
//  Dcc.init( MAN_ID_NUMBER, 100,   FLAGS_MY_ADDRESS_ONLY , 0 );  // Function decoder only
  Dcc.init( MAN_ID_NUMBER, MAN_VER_NUMBER, FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, 0 );  // アクセサリデコーダ

  //アクセサリアドレス(下位2bitを考慮)の先修得
  gAccessoryAddress = getMyAddr_Acc();          // ch1
  gAccessoryAddress2 = gAccessoryAddress + 1;   // ch2
  
  //Init CVs
//  gCV1_SAddr = Dcc.getCV( CV_MULTIFUNCTION_PRIMARY_ADDRESS );   // Function decoder only
//  gCVx_LAddr = (Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB ) << 8) + Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB ); // Function decoder only
}


//------------------------------------------------------------------
// CV値が変化した時の処理（特に何もしない）
//------------------------------------------------------------------
extern void     notifyCVChange( uint16_t CV, uint8_t Value) {
   //CVが変更されたときのメッセージ
  #ifdef DEBUG
    Serial.print("CV "); 
    Serial.print(CV); 
    Serial.print(" Changed to "); 
    Serial.println(Value, DEC);
  #endif
};


//------------------------------------------------------------------
// This function is called whenever a DCC Signal Aspect Packet is receivedz
//------------------------------------------------------------------
void notifyDccSigState( uint16_t Addr, uint8_t OutputIndex, uint8_t State)
{
  // Serial.print("notifyDccSigState: ") ;
  // Serial.print(Addr,DEC) ;
  // Serial.print(',');
  // Serial.print(OutputIndex,DEC) ;
  // Serial.print(',');
  // Serial.println(State, HEX) ;
}



//------------------------------------------------------------------
// Resrt処理（特に何もしない）
//------------------------------------------------------------------
void notifyDccReset(uint8_t hardReset )
{
}
