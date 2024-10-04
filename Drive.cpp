#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <math.h>
#include <string>
#include <SoftwareSerial.h>
#include <PCF8574.h>

//encode1
const int motor1encodeA = 4;
const int motor1encodeB = 16;
//encode2
const int motor2encodeA = 17;
const int motor2encodeB = 5;
//encode3
const int motor3encodeA = 18;
const int motor3encodeB = 19;
//motor1
const int motor1a =12;
const int motor1b =14;
//motor2
const int motor2a =27;
const int motor2b =26;
//motor3
const int motor3a =25;
const int motor3b =33;
//indicator LED
const int RLED = 23;
const int GLED = 15;
const int BLED = 2;
const int YLED = 0;

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("28:ea:0b:d0:38:a4");

PCF8574 Ultrasonic(0x27);

SoftwareSerial mySerial(32,13);

unsigned long startMillis;   
unsigned long currentMillis;

unsigned long ESPstartMillis;   
unsigned long ESPcurrentMillis;

unsigned long Tdelaystart;
unsigned long Tdelaycurrent;

unsigned long USStartMillis;
unsigned long USCurrentMillis;


void setup() {
  Serial.begin(115200);
  Serial.println("Starting NimBLE Client");
  xboxController.begin();

  startMillis = millis();
  ESPstartMillis = millis();
  Tdelaystart = millis();
  USStartMillis = millis();

  Ultrasonic.begin();

  pinMode(RLED,OUTPUT);
  pinMode(GLED,OUTPUT);
  pinMode(YLED,OUTPUT);
  pinMode(BLED,OUTPUT);

  pinMode(motor1a,OUTPUT);
  pinMode(motor1b,OUTPUT);
  pinMode(motor2a,OUTPUT);
  pinMode(motor2b,OUTPUT);
  pinMode(motor3a,OUTPUT);
  pinMode(motor3b,OUTPUT);

  pinMode(motor1encodeA,INPUT_PULLDOWN);
  pinMode(motor1encodeB,INPUT_PULLDOWN);
  pinMode(motor2encodeA,INPUT_PULLDOWN);
  pinMode(motor2encodeB,INPUT_PULLDOWN);
  pinMode(motor3encodeA,INPUT_PULLDOWN);
  pinMode(motor3encodeB,INPUT_PULLDOWN);

  Ultrasonic.pinMode(P0,OUTPUT);
  Ultrasonic.pinMode(P1,INPUT);
  Ultrasonic.pinMode(P2,INPUT_PULLDOWN);
  Ultrasonic.pinMode(P3,INPUT_PULLDOWN);
  // Encoder.digitalWrite(P0,HIGH);

  mySerial.begin(115200);
}

void Vibration(bool a,bool b,bool c,bool d,int x,int y){
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  repo.v.select.left = a;
  repo.v.select.right = b;
  repo.v.select.shake = c;
  repo.v.select.center = d;
  repo.v.power.center = x;
  repo.v.power.left = x;
  repo.v.power.right = x;
  repo.v.power.shake = x;
  repo.v.timeActive = y/10;
  xboxController.writeHIDReport(repo);
}

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

void Range_Finder(){

}

int start = 0;
bool Controller_Lock = 1;
int Controll_Mode_index = 0;
String Controll_Mode[] = {"Moving","Reaching"};
int Moiving_Mode_index =0;
String Moiving_Mode[] = {"Walk","Rush"};
double movingmodemultiply[] = {0.5, 1.0};
int Reaching_Mode_index =0;
String Reaching_Mode[] = {"mode1","mode2"};

/***
Lock : green
moving : 
  move walk : blue
  move rush : blue red
reaching :
  mode1 : yellow
  mode2 : yellow red
***/

int controllertheshold;
int rotationmultiply;

int controllerLV;
int controllerLH;
int controllerRH;
int controllerRV;
int controllerTrigL;
int controllerTrigR;
int controllerBtnA;
int controllerBtnB;
int controllerBtnX;
int controllerBtnY;


int Motor1_value;
int Motor2_value;
int Motor3_value;


void loop() {
  currentMillis = millis();
  ESPcurrentMillis = millis();
  Tdelaycurrent = millis();

  xboxController.onLoop();
  if (xboxController.isConnected()) {
    if (xboxController.isWaitingForFirstNotification()) {
      Serial.println("waiting for first notification");
    } else {
      if(start <=0){
        Vibration(1,1,1,1,70,250);
        start+=1;
      };

      if(Controller_Lock == 0){
        if(xboxController.xboxNotif.btnXbox && (currentMillis - startMillis >= 400)){
        startMillis = currentMillis;
        Controller_Lock =1;
        Serial.println("Controller Lock");
        Vibration(0,0,1,1,100,100);
        }
        
        if(xboxController.xboxNotif.btnLB && (currentMillis - startMillis >= 400)){
        startMillis = currentMillis;
        Controll_Mode_index+=1;
        Serial.print("Mode ");
        Serial.println(Controll_Mode[Controll_Mode_index%2]);
        Vibration(1,0,0,0,70,250);
        }

        if(Controll_Mode[Controll_Mode_index%2] == "Moving"){

          if(xboxController.xboxNotif.btnRB && (currentMillis - startMillis >= 400)){
            startMillis = currentMillis;
            Moiving_Mode_index+=1;
            Serial.print("Moving Mode ");
            Serial.println(Moiving_Mode[Moiving_Mode_index%2]);
            Vibration(0,1,0,0,70,120);
            };

          if((Moiving_Mode_index%2)==0){
              digitalWrite(RLED,LOW);
              digitalWrite(GLED,LOW);
              digitalWrite(BLED,HIGH);
              digitalWrite(YLED,LOW);
          }else if((Moiving_Mode_index%2)==1){
              digitalWrite(RLED,HIGH);
              digitalWrite(GLED,LOW);
              digitalWrite(BLED,HIGH);
              digitalWrite(YLED,LOW);
            }

          String str = "";
          while(Serial.available()){
            str.concat(char(Serial.read()));
          }
          str.trim();
          // Serial.println(str);


          //wait for ultrasonic init

//moving code here
          controllertheshold = 12;
          rotationmultiply = 2;

          if(abs((-1)*(xboxController.xboxNotif.joyLVert/128)+256) > controllertheshold){
            controllerLV = (-1)*(xboxController.xboxNotif.joyLHori/128)+256 ;
          }else controllerLV = 0;

          if(abs((-1)*(xboxController.xboxNotif.joyLHori/128)+256) > controllertheshold){
            controllerLH = (-1)*(xboxController.xboxNotif.joyLVert/128)+256;
          }else controllerLH = 0;

          if(abs((-1)*(xboxController.xboxNotif.joyRHori/128)+256) > controllertheshold){
            controllerRH = ((-1)*(xboxController.xboxNotif.joyRHori/128)+256)*rotationmultiply;
          }else controllerRH = 0;

          //ultrasonic cal here
          //
          //ultrasonic cal above


          Motor1_value = (-controllerLV + (-controllerRH * 0.125 * 3)) * movingmodemultiply[Moiving_Mode_index%2];
          Motor2_value = ((controllerLV * sin(PI/6)) - (controllerLH * cos(PI/6)) + (-controllerRH * 0.125 * 3)) * movingmodemultiply[Moiving_Mode_index%2];
          Motor3_value = ((controllerLV * sin(PI/6)) + (controllerLH * cos(PI/6)) + (-controllerRH * 0.125) * 3) * movingmodemultiply[Moiving_Mode_index%2];

          if(xboxController.xboxNotif.joyRHori || xboxController.xboxNotif.joyLVert || xboxController.xboxNotif.joyLHori){
          // if(xboxController.xboxNotif.joyRHori || xboxController.xboxNotif.joyLVert || xboxController.xboxNotif.joyLHori){
            // Serial.print("Motor1_value = ");
            // Serial.println(Motor1_value);
            // Serial.print("Motor2_value = ");
            // Serial.println(Motor2_value);
            // Serial.print("Motor3_value = ");
            // Serial.println(Motor3_value);
            // Serial.println("hello");
            Drive_Motor(motor1a,motor1b,Motor1_value);
            Drive_Motor(motor2a,motor2b,Motor2_value);
            Drive_Motor(motor3a,motor3b,Motor3_value);
            delay(2);
          }

//moveing code above

        }else 
        if(Controll_Mode[Controll_Mode_index%2] == "Reaching"){

          if(xboxController.xboxNotif.btnRB && (currentMillis - startMillis >= 400)){
            startMillis = currentMillis;
            Reaching_Mode_index+=1;
            Serial.print("Reaching Mode ");
            Serial.println(Reaching_Mode[Reaching_Mode_index%2]);
            Vibration(0,1,0,0,70,120);
            };

          if((Reaching_Mode_index%2)==0){
              digitalWrite(RLED,LOW);
              digitalWrite(GLED,LOW);
              digitalWrite(BLED,LOW);
              digitalWrite(YLED,HIGH);
            }else if ((Reaching_Mode_index%2)==1){
              digitalWrite(RLED,HIGH);
              digitalWrite(GLED,LOW);
              digitalWrite(BLED,LOW);
              digitalWrite(YLED,HIGH);
            }

//Arm code

          if(abs((-1)*(xboxController.xboxNotif.joyLHori/128)+256) > controllertheshold){
            controllerLH = (-1)*(xboxController.xboxNotif.joyLHori/128)+256 ;
          }else controllerLH = 0;

          if(abs((-1)*(xboxController.xboxNotif.joyLVert/128)+256) > controllertheshold){
            controllerLV = (-1)*(xboxController.xboxNotif.joyLVert/128)+256 ;
          }else controllerLV = 0;

          if(abs((-1)*(xboxController.xboxNotif.joyRHori/128)+256) > controllertheshold){
            controllerRH = (-1)*(xboxController.xboxNotif.joyRHori/128)+256 ;
          }else controllerRH = 0;

          if(abs((-1)*(xboxController.xboxNotif.joyRVert/128)+256) > controllertheshold){
            controllerRV = (-1)*(xboxController.xboxNotif.joyRVert/128)+256 ;
          }else controllerRV = 0;

          if(xboxController.xboxNotif.trigLT/4){
            controllerTrigL = xboxController.xboxNotif.trigLT/4 ;
          }else controllerTrigL = 0;

          if(xboxController.xboxNotif.trigRT/4){
            controllerTrigR = xboxController.xboxNotif.trigRT/4 ;
          }else controllerTrigR = 0;

          if(xboxController.xboxNotif.btnA){
            controllerBtnA = xboxController.xboxNotif.btnA;
          }else controllerBtnA = 0 ;

          if(xboxController.xboxNotif.btnB){
            controllerBtnB = xboxController.xboxNotif.btnB;
          }else controllerBtnB = 0 ;

          if(xboxController.xboxNotif.btnX){
            controllerBtnX = xboxController.xboxNotif.btnX;
          }else controllerBtnX = 0 ;

          if(xboxController.xboxNotif.btnY){
            controllerBtnY = xboxController.xboxNotif.btnY;
          }else controllerBtnY = 0 ;


          if(Tdelaycurrent - Tdelaystart > 2){
          Tdelaystart = Tdelaycurrent;
          mySerial.print("(");
          //LH
          mySerial.print(controllerLH);
          mySerial.print(":");
          //LV
          mySerial.print(controllerLV);
          mySerial.print(":");
          //RH
          mySerial.print(controllerRH);
          mySerial.print(":");
          //RV
          mySerial.print(controllerRV);
          mySerial.print("::");
          //TrigL
          mySerial.print(controllerTrigL);
          mySerial.print(":");
          //TrigR
          mySerial.print(controllerTrigR);
          mySerial.print("::");
          //BntA
          mySerial.print(controllerBtnA);
          mySerial.print(":");
          //BntB
          mySerial.print(controllerBtnB);
          mySerial.print(":");
          //BntX
          mySerial.print(controllerBtnX);
          mySerial.print(":");
          //BntY
          mySerial.print(controllerBtnY);
          mySerial.println(")");

          }


//Arm code abrove
        }
      }else if(Controller_Lock == 1){
        if(xboxController.xboxNotif.btnLB && xboxController.xboxNotif.btnRB && (currentMillis - startMillis >= 400)){
          startMillis = currentMillis;
          Controller_Lock = 0;
          Serial.println("Controller UnLock");
          Vibration(0,0,1,1,100,100);
        }
        digitalWrite(RLED,LOW);
        digitalWrite(GLED,HIGH);
        digitalWrite(BLED,LOW);
        digitalWrite(YLED,LOW);
      }
    }
  }else{

    if(currentMillis - startMillis >= 350){
      startMillis = currentMillis;
      if(digitalRead(RLED) == LOW){
        digitalWrite(RLED,HIGH);
        digitalWrite(BLED,HIGH);
        digitalWrite(GLED,LOW);
        digitalWrite(YLED,LOW);
      }else{
        digitalWrite(RLED,LOW);
        digitalWrite(BLED,LOW);
        digitalWrite(GLED,HIGH);
        digitalWrite(YLED,HIGH);
      }
    }
    // Serial.println("not connected");
    if (!xboxController.isConnected() && (ESPcurrentMillis - ESPstartMillis >= 15000)) {//20000
      ESPstartMillis = ESPcurrentMillis;
      ESP.restart();

    }
  }
}
