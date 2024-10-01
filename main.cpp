#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <string.h>
#include <algorithm> 
#include <iostream>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>

using namespace std; 

SoftwareSerial mySerial(32,13);

unsigned long startMillis;   
unsigned long currentMillis;

unsigned long servodelaystart;
unsigned long servodelaycurrent;


Servo servo1;
Servo servo2;

const int motor1encodeA = 4;
const int motor1encodeB = 16;
const int motor2encodeA = 17;
const int motor2encodeB = 5;
const int motor3encodeA = 18;
const int motor3encodeB = 19;

const int motor1a = 14;
const int motor1b = 12;
const int motor2a = 26;
const int motor2b = 27;
const int motor3a = 33;
const int motor3b = 25;

void Drive_Motor(int motorA,int motorB,int motorvalue){
  if(motorvalue < -255){motorvalue = -255;}
  if(motorvalue > 255){motorvalue = 255;}
  if(motorvalue > 0){
    analogWrite(motorA,abs(motorvalue));
    analogWrite(motorB,0);
  }else if(motorvalue < 0){
    analogWrite(motorA,0);
    analogWrite(motorB,abs(motorvalue));
  }else{
    analogWrite(motorA,0);
    analogWrite(motorB,0);
  }
}


void setup(){
    Serial.begin(115200);
    mySerial.begin(115200);

    startMillis = millis();
    servodelaystart = millis();

    servo1.attach(22);
    servo2.attach(23);

    pinMode(motor1encodeA,INPUT_PULLDOWN);
    pinMode(motor1encodeB,INPUT_PULLDOWN);
    pinMode(motor2encodeA,INPUT_PULLDOWN);
    pinMode(motor2encodeB,INPUT_PULLDOWN);
    pinMode(motor3encodeA,INPUT_PULLDOWN);
    pinMode(motor3encodeB,INPUT_PULLDOWN);

    pinMode(motor1a,OUTPUT);
    pinMode(motor1b,OUTPUT);
    pinMode(motor2a,OUTPUT);
    pinMode(motor2b,OUTPUT);
    pinMode(motor3a,OUTPUT);
    pinMode(motor3b,OUTPUT);


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

double ArmMultiplier = 0.6;

int servopos = 0;
int servodelayconst = 15;
int servo1delay;

int servo2index = 0;
String servo2state[] = {"Open","Close"};

void loop(){
  currentMillis = millis();
  servodelaycurrent = millis();

  String str = "";
  String str1 = "";
  while(Serial.available()){
    str.concat(char(Serial.read()));
  }

  str.trim();

  while(mySerial.available()){
    str1.concat(char(mySerial.read()));
    }
  
  str1.trim();

  // Serial.println(str);

  if(str == "Controller UnLock"){
    // Serial.println(":D");
    controllers_Lock = 0;

    }else if(str == "Controller Lock"){
      // Serial.println("8P");
      controllers_Lock = 1;
    }

  if( (str == "Mode Reaching") || (str == "Reaching Mode mode1") || (str == "Reaching Mode mode2")){
    Reaching_mode = 1;
  }else if( (str == "Mode Moving") || (str == "Moving Mode Walk") || (str == "Moving Mode Rush")){
    Reaching_mode = 0;
  }

  if(str == "Reaching Mode mode1"){
    Mode = 0;
  }else if(str == "Reaching Mode mode2"){
    Mode = 1;
  }

  // Serial.print("lock ");
  // Serial.print(controllers_Lock);
  // Serial.print("reach ");
  // Serial.print(Reaching_mode);
  // Serial.print("mode ");
  // Serial.println(Mode);
  // delay(500);  


  if((controllers_Lock == 0) && (Reaching_mode == 1)){
    
    // Serial.println("Reachmode");
    
    
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


    if(Mode == 0){
      // Serial.println("          mode 1");

      /// Jacobian here SOON

        Drive_Motor(motor1a,motor1b,controllerRH*ArmMultiplier);
        Drive_Motor(motor2a,motor2b,controllerRV*ArmMultiplier);
        Drive_Motor(motor3a,motor3b,controllerLV*ArmMultiplier);
        delay(2);

      Serial.print("motor 1 = ");
      Serial.print(controllerRH);
      Serial.print("  motor 2 = ");
      Serial.print(controllerRV);
      Serial.print("  motor 3 = ");
      Serial.println(controllerLV);

    }if(Mode == 1){ //griper pitch   and  grap
    // Serial.println("                  mode 2");
      if(servodelaycurrent - servodelaystart > 5){
        servodelaystart = servodelaycurrent;
        if(controllerTrigR && servopos < 180){
          servo1.write(servopos);
          servopos += 1;
        }else if(controllerTrigL  && servopos > 0){
          servo1.write(servopos);
          servopos -= 1;
      }
      }
        
      //button A  for griper
      if(controllerBtnA && (currentMillis - startMillis > 500)){
        startMillis = currentMillis;
        servo2index += 1;

        if(servo2index%2 == 0){
          servo2.write(0);
        }else if(servo2index%2 == 1){
          servo2.write(180);
        }
      }


      Serial.print("servopos = ");
      Serial.println(servopos);

      Serial.print("griper = ");
      Serial.println(servo2state[servo2index%2]);
    }



  }else if((controllers_Lock == 0) && (Reaching_mode == 0)){
    //moving mode
    // Serial.println("                    moving");
  }else if((controllers_Lock == 1)){
    //locked
  }

}