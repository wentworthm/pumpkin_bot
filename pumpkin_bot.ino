#define BLYNK_PRINT Serial
 
#define BLYNK_USE_DIRECT_CONNECT
 
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <analogWrite.h>
 
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "zBPnqXLXzZAEH2fQaFko_QGayMgwhYwA ";

int RightMotorSpeed = 2;  
int LeftMotorSpeed = 4; 
int RightMotorDir = 5; 
int LeftMotorDir = 18; 

int minRange = 312;//462;//
int maxRange = 712;//562;//

int minspeed = 130;
int maxspeed = 255;
int nospeed = 0;

BLYNK_WRITE(V0)
{
  int x = param[0].asInt();
  int y = param[1].asInt();
  moveControl(x,y);
}

void moveControl(int x, int y)
{
  int distance=sqrt(pow(512-x,2)+pow(512-y,2));
  
//Move Forward
    if(y >= maxRange && x >= minRange && x<= maxRange)
    {
        digitalWrite( RightMotorDir,HIGH);
        digitalWrite(LeftMotorDir,HIGH);
        analogWrite(RightMotorSpeed, map(distance,200,512,130,255));
        analogWrite(LeftMotorSpeed , map(distance,200,512,125,10));
        Serial.println("Move forward");
    }
//Move Forward Right
    else if(x <= minRange && y >= maxRange)
    {
        digitalWrite( RightMotorDir, HIGH);
        digitalWrite(LeftMotorDir,LOW);
        analogWrite(RightMotorSpeed,map(distance,200,512,130,255));
//        analogWrite(LeftMotorSpeed ,maxspeed);
        Serial.println("Move forward right");
    }
//Move Forward Left
    else if(x >= maxRange && y >= maxRange)
    {
        digitalWrite( RightMotorDir,LOW);
        digitalWrite(LeftMotorDir,HIGH);
//        analogWrite(RightMotorSpeed,nospeed);
        analogWrite(LeftMotorSpeed ,map(distance,200,512,125,10));
        Serial.println("Move forward left");
    }
//No Move
    else if(y < maxRange && y > minRange && x < maxRange && x > minRange)
    {
        digitalWrite( RightMotorDir,LOW);
        digitalWrite(LeftMotorDir,LOW);

        Serial.println("Stop");
    }
//Move Backward
    else if(y <= minRange && x >= minRange && x <= maxRange)
    {
        digitalWrite( RightMotorDir,HIGH);
        digitalWrite(LeftMotorDir,HIGH);
        analogWrite(RightMotorSpeed,map(distance,200,512,125,0));
        analogWrite(LeftMotorSpeed ,map(distance,200,512,130,245));

        Serial.println("Move backward");
    }
//Move Backward Right
    else if(y <= minRange && x >= maxRange)
    {
        digitalWrite( RightMotorDir,LOW);
        digitalWrite(LeftMotorDir,HIGH);
        analogWrite(RightMotorSpeed,map(distance,200,512,125,0));
        analogWrite(LeftMotorSpeed ,map(distance,200,512,130,245));
        Serial.println("Move backward right");
    }
//Move Backward Left
    else if(y <= minRange && x <= minRange)
    {
        digitalWrite( RightMotorDir,HIGH);
        digitalWrite(LeftMotorDir,LOW);
        analogWrite(RightMotorSpeed,map(distance,200,512,125,0));
//        analogWrite(LeftMotorSpeed ,nospeed);
        Serial.println("Move backward left");
    }
    else if(y >= minRange && y<= maxRange && x <= maxRange)
    {
        digitalWrite( RightMotorDir,HIGH);
        digitalWrite(LeftMotorDir,HIGH);
        analogWrite(RightMotorSpeed,map(distance,200,512,130,255));
        analogWrite(LeftMotorSpeed ,map(distance,200,512,130,245));
        Serial.println("Turn right");
    }
    else if(y >= minRange && y<= maxRange && x >= maxRange)
    {
        digitalWrite( RightMotorDir,HIGH);
        digitalWrite(LeftMotorDir,HIGH);
        analogWrite(RightMotorSpeed,map(distance,200,512,125,0));
        analogWrite(LeftMotorSpeed ,map(distance,200,512,125,10));
        Serial.println("Turn left");
    }
}

void setup()
{
    Serial.begin(9600);
    
    Serial.println("Waiting for connections...");
    Blynk.setDeviceName("Geoffrey Bot");
    Blynk.begin(auth);

    pinMode(RightMotorSpeed, OUTPUT);
    pinMode(LeftMotorSpeed , OUTPUT);
    pinMode(RightMotorDir, OUTPUT);
    pinMode(LeftMotorDir, OUTPUT);

    //digitalWrite(RightMotorSpeed, LOW);
    //digitalWrite(LeftMotorSpeed , LOW);
    digitalWrite(RightMotorDir, LOW);
    digitalWrite(LeftMotorDir, LOW);
}

void loop()
{
  Blynk.run();
}
