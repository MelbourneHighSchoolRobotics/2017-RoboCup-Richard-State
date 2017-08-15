/*
Example sketch to demonstrate use of Movement.
This skecth demonstrates the use of dirveMotors.
--------------------------------------------------------

Created 24/8/2014 by Peter Drew. Last modified 14/8/2016 by Peter Drew.
*/

#include "RB_Movement.h"
#include "RB_Motor.h"
#include "Compass.h"
#include "Wire.h"
//compass HMC5883L(20,21,0x1E);

Movement bot(6, 7, 4, 5, 3, 2); //this creates an instance of the object and sets motor pins

void setup() { //nothing is needed in setup
  
  Serial.begin(9600);
  // Serial.print("start");
  pinMode(13,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  pinMode(22,OUTPUT);
  digitalWrite(22,HIGH);
  /**
  Wire.begin();
  // Serial.print("hmc");
  HMC5883L.init();
  // Serial.print("done");d
  **/
}

void loop() {
  // Serial.println(digitalRead(39));
  /**
  //bot.moveRotate(5);
  bot.manualMotors(5,0,0);
  delay(500);
  bot.allStop();
  delay(200);
  bot.manualMotors(0,5,0);
  delay(500);
  bot.allStop();
  delay(200);
  bot.manualMotors(0,0,5);
  delay(500);
  bot.allStop();
  // Serial.print("fuck");
  **/
}

