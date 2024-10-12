#include <string.h>
#include <algorithm> 
#include <iostream>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>
#include <PCF8574.h>

using namespace std;

PCF8574 Encoder(0x24);

SoftwareSerial mySerial(32,13);

unsigned long startMillis;   
unsigned long currentMillis;

unsigned long servodelaystart;
unsigned long servodelaycurrent;

unsigned long servo2delaystart;
unsigned long servo2delaycurrent;

Servo servo1;
Servo servo2;

const int motor1a = 14;
const int motor1b = 12;
const int motor2a = 26;
const int motor2b = 27;
const int motor3a = 33;
const int motor3b = 25;

const int PWM1 = 5;
const int PWM2 = 18;
const int PWM3 = 19;

const int Hallsensor1 = 4;
const int Hallsensor2 = 16;
const int Hallsensor3 = 17;

void Drive_Motor(int motorA,int motorB,int motorPWM,int motorvalue){
  if(motorvalue < -255){motorvalue = -255;}
  if(motorvalue > 255){motorvalue = 255;}
  if(motorvalue > 0){
    digitalWrite(motorA,HIGH);
    digitalWrite(motorB,LOW);
    analogWrite(motorPWM,abs(motorvalue));
  }else if(motorvalue < 0){
    digitalWrite(motorA,LOW);
    digitalWrite(motorB,HIGH);
    analogWrite(motorPWM,abs(motorvalue));
  }else{
    digitalWrite(motorA,HIGH);
    digitalWrite(motorB,HIGH);
    analogWrite(motorPWM,0);
  }
}

void setup(){
    Serial.begin(115200);
    mySerial.begin(115200);

    startMillis = millis();
    servodelaystart = millis();
    servo2delaystart = millis();

    servo1.attach(15);
    servo2.attach(23);

    Encoder.pinMode(P0,INPUT_PULLDOWN);
    Encoder.pinMode(P1,INPUT_PULLDOWN);
    Encoder.pinMode(P2,INPUT_PULLDOWN);
    Encoder.pinMode(P3,INPUT_PULLDOWN);
    Encoder.pinMode(P4,INPUT_PULLDOWN);
    Encoder.pinMode(P5,INPUT_PULLDOWN);
    // Encoder.digitalRead(P0);

    pinMode(motor1a,OUTPUT);
    pinMode(motor1b,OUTPUT);
    pinMode(motor2a,OUTPUT);
    pinMode(motor2b,OUTPUT);
    pinMode(motor3a,OUTPUT);
    pinMode(motor3b,OUTPUT);

    pinMode(PWM1,OUTPUT);
    pinMode(PWM2,OUTPUT);
    pinMode(PWM3,OUTPUT);

    pinMode(Hallsensor1,INPUT_PULLDOWN);
    pinMode(Hallsensor2,INPUT_PULLDOWN);
    pinMode(Hallsensor3,INPUT_PULLDOWN);

    servo1.write(95);
    // servo2.write(170);

    servo2.write(0);
    delay(150);
    servo2.write(180);
    delay(150);
    servo2.write(90);

    Drive_Motor(motor1a,motor1b,PWM1,-100);
    delay(580);
    Drive_Motor(motor1a,motor1b,PWM1,0);
}

bool controllers_Lock = 1;
bool Reaching_mode = 0; //0 = move , 1 = reach
bool Mode = 0; // 0 = mode1 , 1 = mode2

int controllerLH;
int controllerLV;
int controllerRH;
int controllerRV;
int controllerTrigL;
int controllerTrigR;
int controllerBtnA;
int controllerBtnB;
int controllerBtnX;
int controllerBtnY;

int scancount;

int servopos = 95;
// int servopos2 = 170;

int servo2index = 0;
String servo2state[] = {"Open","Close"};
double ArmMultiplier;
double ElbowMultiplier;
double ArmHoriMultiplier;
int servodelay;
int servo2delay;

void loop(){
  currentMillis = millis();
  servodelaycurrent = millis();
  servo2delaycurrent = millis();

  String str = "";
  String str1 = "";
  while(mySerial.available()){
    str.concat(char(mySerial.read()));
  }

  str.trim();

  while(Serial.available()){
    str1.concat(char(Serial.read()));
    }
  
  str1.trim();

  // Serial.println(str);

  scancount = sscanf(str1.c_str(),"(%d:%d:%d:%d::%d:%d::%d:%d:%d:%d)",&controllerLH,&controllerLV,&controllerRH,&controllerRV,&controllerTrigL,&controllerTrigR,&controllerBtnA,&controllerBtnB,&controllerBtnX,&controllerBtnY);


  // if(scancount == 10){
  // Serial.print(controllerLH);
  // Serial.print("  ");
  // Serial.print(controllerLV);
  // Serial.print("  ");
  // Serial.print(controllerRH);
  // Serial.print("  ");
  // Serial.print(controllerRV);
  // Serial.print("  ");
  // Serial.print(controllerTrigL);
  // Serial.print("  ");
  // Serial.print(controllerTrigR);
  // Serial.print("  ");
  // Serial.print(controllerBtnA);
  // Serial.print("  ");
  // Serial.print(controllerBtnB);
  // Serial.print("  ");
  // Serial.print(controllerBtnX);
  // Serial.print("  ");
  // Serial.println(controllerBtnY);
  // }



  ArmHoriMultiplier = 0.35;
  ArmMultiplier = 0.45;
  ElbowMultiplier = 0.35;
  servodelay = 20;
  // servo2delay = 10;

  /// Jacobian here SOON

    Drive_Motor(motor1a,motor1b,PWM1,controllerRH*ArmHoriMultiplier);
    Drive_Motor(motor2a,motor2b,PWM2,(-1)*controllerRV*ArmMultiplier);
    Drive_Motor(motor3a,motor3b,PWM3,(-1)*controllerLV*ElbowMultiplier);
    delay(2);

    // Serial.print("motor 1 = ");
    // Serial.print(controllerRH*ArmHoriMultiplier);
    // Serial.print("  motor 2 = ");
    // Serial.print((-1)*controllerRV*ArmMultiplier);
    // Serial.print("  motor 3 = ");
    // Serial.print((-1)*controllerLV*ArmMultiplier);


    if(servodelaycurrent - servodelaystart > servodelay){
      servodelaystart = servodelaycurrent;
      if(controllerTrigL && servopos < 180){
        servo1.write(servopos);
        servopos += 1;
      }else if(controllerTrigR  && servopos > 0){
        servo1.write(servopos);
        servopos -= 1;
      }
    }

    //button A,B  for griper
    // if(servo2delaycurrent - servo2delaystart > servo2delay){
    //   servo2delaystart = servo2delaycurrent;
    //   if(controllerBtnA && servopos2 < 180){
    //     servo2.write(servopos2);
    //     servopos2 += 1;
    //   }else if(controllerBtnB  && servopos2 > 100){
    //     servo2.write(servopos2);
    //     servopos2 -= 1;
    //   }
    // }

    if(controllerBtnA || controllerBtnB){
      if(controllerBtnA){
        servo2.write(75);
      }else if(controllerBtnB){
        servo2.write(105);
      }
    }else servo2.write(90);
}
