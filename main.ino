/**
*Richardbot 0.0.1

*/


#include "RB_Movement.h"
#include "RB_Motor.h"

Movement bot(6,7,4,5,2,3); //this creates an instance of the object and sets motor pins
#define MOTOR_ON 1
//----------IR CONFIG-----------//
//Maximum IR counter Value (max 255)
#define MAX_COUNTER 32
#define MIN_THRESHOLD 10
// * The number of IR sensors on the robot.
#define IR_NUM 6
// * The pin used for powering all of the IR sensors (in parallel)
#define IR_UNLOCK_PIN 23
// * Enable or disable debug info
// * No noticable performance impact normally.
// * To enable, set to 1.  To disable, set to 0

// * Sensor to Arduino pin mappings 
// * Note: Analog sensors follow on from digital pins (or use A0 notation)
// * If a sensor is broken set it to 255 to disable it
                //SENSOR: 1   2   3 4  5   6   7   8   9  10  11 12
byte IRSensors[IR_NUM] = {255,48,50,52,51,49}; 
/**
48:Front left
50:Front Mid
52: Front right
51: Far right forwards
59: Far right back
**/
//temp hack, one more then normal with pin 255 so that 0 is never highest unless there is no signal
//----------END IR CONFIG----------//

//----------COLOUR SENSOR CONFIG -------------//
#define LIGHT_UNLOCK_PIN 22 //Digital 22
#define BLACK_MIN 320
#define BLACK_MAX 330

#define WHITE_MIN 320
#define WHITE_MAX 330
#define CLR_LIGHT_PIN 10 //Digital colour sensor pin
#define CLR_NUM 1
byte CLRSensors[CLR_NUM] = {A0};//,A1,A2}; //analgog colour sensor pins
int CLRValues[CLR_NUM];
byte CLRColours[CLR_NUM]; // 1 for green 2 for black 3 for white
//--------------END COLOR SENSOR CONFIG-----------//
#define DEBUG 1

// * The baud rate to use for debugging output
// * Keep this low to avoid communication errors
// * Only relevant if DEBUG == 1
#define DEBUG_BAUD_RATE 9600

//-------------------!!END CONFIG!!--------------//
//--------------------!!GLOBALS!!-------------//

// * The best IR sensor currently (0 means no sensor)
// * Invariant: 0 <= IR_BEST <= SENSORS <= 255
byte IR_BEST = 0;

// * The number of loops so far
// * Invariant: 0 <= counter <= MAX_COUNTER <= 255
byte counter = 0;

// * The current values for each sensor (0 to MAX_COUNTER)
byte IRValues[IR_NUM];  

void initIR() {
  // Initialize the diodes and enable power
  pinMode(IR_UNLOCK_PIN, OUTPUT);
  digitalWrite(IR_UNLOCK_PIN, HIGH);
  //Set all IR values to 0, and set them as input
  for (byte i = 0;i < IR_NUM;i++) {
    IRValues[i] = 0;
    pinMode(IRSensors[i], INPUT);
  }
}

void readIR() {
  for (byte i = 0;i < IR_NUM;i++) { //iterate over each sensor if not disabled
    if (IRSensors[i] < 255) { // if sensor not disabled
      IRValues[i] += (digitalRead(IRSensors[i]) == HIGH ? 0 : 1);// if sensor is high, add 1
    }
  }
  counter++;
  if(counter >= MAX_COUNTER) {
    getBestIR();
  }
}

void getBestIR() {
  // If finished a block, calculate the best.
    counter = 0; // reset counter
    // * the index of the best sensor
    byte b_index = 0;
    // * the reading of the best sensor
    byte b_value = 0;
    
    for (byte i = 0;i < IR_NUM;i++) { //find highest value
      if (IRValues[i] > b_value) {
        b_index = i;
        b_value = IRValues[i];
      }
      // debug stuff, show us the value of each sensor
      if (DEBUG) {
        Serial.print(i, DEC);
        Serial.print(",");
        Serial.print(IRSensors[i]);
        Serial.print("=");
        Serial.print(IRValues[i], DEC);
        Serial.print(" | ");
      }
      IRValues[i] = 0; //reset to 0 for next time
    }

    // set the new best sensor
    if (b_value > MIN_THRESHOLD) {
      IR_BEST = b_index;
    }
    else {
      IR_BEST = 0;
    }

    //Make sure counter is reset
    counter = 0;
   
   	// reset all sensors by dropping power for 2ms
    digitalWrite(IR_UNLOCK_PIN, LOW);
    delay(2); //TODO POSSIBLY, CHANGE TO MILLIS
    digitalWrite(IR_UNLOCK_PIN, HIGH);

    // more debug, print best, TODO CHCEK
    if (DEBUG) {
      Serial.print("BEST = ");
      Serial.print(IR_BEST, DEC);
      Serial.print(" (");
      Serial.print(IRSensors[b_index-1], DEC);
      Serial.print("): ");
      Serial.println(b_value, DEC);
    }

}

void initCLR() { // temp 26/07/2017
  pinMode(LIGHT_UNLOCK_PIN, OUTPUT);
  digitalWrite(LIGHT_UNLOCK_PIN, HIGH);
}
void readCLR() {
  for(byte i = 0; i <CLR_NUM;i++) {
    int reading = analogRead(CLRSensors[i]);
    CLRValues[i] = reading;
    if ((reading < WHITE_MAX) && (reading > WHITE_MIN)) {
      CLRColours[i] = 2; //WHITE
    }
    else if ((reading < BLACK_MAX) && (reading > BLACK_MIN)) {
      CLRColours[i] = 3; //BLACK
    }
    else {
      CLRColours[i] = 1; // GREEN
    }
    if (DEBUG) {
      Serial.print("C");
      Serial.print(i);
      Serial.print(":");
      Serial.print(" ");
      Serial.print(reading);
    }
 
  }
  if (DEBUG) {
    Serial.println();
  }
}
//-----------Arduino base code-------------//
void setup() {
  // Enable Debug
  if (DEBUG) {
    Serial.begin(DEBUG_BAUD_RATE);
  }

  initIR();
  initCLR();

}

void loop() {
  // Read the sensors and get the current reading
  readIR();
  /**
  readCLR();
  int currentMot[3] = {bot.getMotorVelocity(1),bot.getMotorVelocity(2),bot.getMotorVelocity(3)};
  int invMot[3] = {0,0,0};
  for(byte i = 0; i<3;i++) {
    invMot[i] = currentMot[i] * -1;
    //Serial.print(invMot[i]);
  }
  for(byte i = 0; i<CLR_NUM;i++) {
    switch(CLRColours[i]) {
      case 1: //green
        break;
      case 2: // boundary - move in the opposite way you were
        bot.moveStraight(-180,10); // opposite dir doesn't work atm, temp test back
        //bot.manualMotors(invMot[0],invMot[1],invMot[2]);
        break;
      case 3: // black
        break;
    }
  }
**/
  if(MOTOR_ON == 1) {
    switch (IR_BEST) { //ultra temp
      case 0:
        bot.allStop(); //can't find anything
        break;
      case 1: //TODO check about ramping bceause the bot turns when 1 or 2 due to motor bias? 12/08/2017
        bot.manualMotors(10,0,-10); //move left
        //bot.manualMotors(-10,10,30); //rotate left and move forwards
        break;
      case 2:
        bot.moveStraight(0,10); //?
        break;
      case 3:
        bot.manualMotors(-10,0,10); //        move right
        //bot.manualMotors(-10,10,-30); //rotate right and move forwards
        break;
      case 4:
        bot.manualMotors(-10,0,10); //move right again?
       // bot.manualMotors(0,-10,10);            //move back and right****check******
      case 5:
        bot.moveStraight(-180,10); //move backwards
    }
  }
}