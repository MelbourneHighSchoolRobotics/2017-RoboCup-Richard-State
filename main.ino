/**
*Richardbot 0.0.1

*/

#include <Wire.h> //used for compass sensor
#include "Compass.h"
compass HMC5883L(20,21,0x1E);
int initialHeading;
#include "RB_Movement.h"
#include "RB_Motor.h"
// manual manualMotors reference code
//      right anti, lef anti, back anticlockwise
//bot.manualMotors(1,1,1)
Movement bot(6,7,4,5,2,3); //this creates an instance of the object and sets motor pins
#define MOTOR_ON 1
//----------Switch config-------//
#define CALLIBRATE_PIN 13
#define RUN_PIN 12


//----------IR CONFIG-----------//
//Maximum IR counter Value (max 255)
#define MAX_COUNTER 32
#define MIN_THRESHOLD 10
// * The number of IR sensors on the robot.
#define IR_NUM 7
// * The pin used for powering all of the IR sensors (in parallel)
#define IR_UNLOCK_PIN 23
// * Enable or disable debug info
// * No noticable performance impact normally.
// * To enable, set to 1.  To disable, set to 0

// * Sensor to Arduino pin mappings 
// * Note: Analog sensors follow on from digital pins (or use A0 notation)
// * If a sensor is broken set it to 255 to disable it
                //SENSOR: 1   2   3 4  5   6   7   8   9  10  11 12
byte IRSensors[IR_NUM] = {255,48,50,52,51,49,47}; 
unsigned long lastBL = 0; //globals for millis for left and right IR
unsigned long lastBR = 0;
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
#define BLACK_MIN 300
#define BLACK_MAX 323

#define WHITE_MIN 300
#define WHITE_MAX 323
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
  pinMode(CALLIBRATE_PIN,INPUT_PULLUP);
  pinMode(RUN_PIN,INPUT_PULLUP);
  initIR();
  initCLR();


}
boolean doneCallib = false;
void loop() {
  
  boolean run = !digitalRead(RUN_PIN);
  boolean callibrate = !digitalRead(CALLIBRATE_PIN);
  /**if (DEBUG) {
    Serial.print("Run");
    Serial.print(doneCallib);
    Serial.print("callibrate");
    Serial.print(callibrate);
    Serial.print( " | ");
  } **/
  if(callibrate) {
    if (!doneCallib) {
      Serial.println("Begin!");
      delay(500);
      Serial.print("Callibrating...");
      bot.moveRotate(30);
      HMC5883L.calibrate();
      
      delay(2000); //rotate for n sec while we callibrate
      Serial.print("???");
      bot.allStop();
      delay(1000);
      HMC5883L.setZeroBearing();
      initialHeading = 0; 
      Serial.println("DONE!");
      doneCallib = true;
    }
}
  else if (run) {
    doneCallib = false;
    // Read the sensors and get the current reading
    readIR();
    
    //readCLR();

    //COLOUR MOVEMENT
    /**
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
          //bot.moveStraight(-180,10); // opposite dir doesn't work atm, temp test back
          bot.manualMotors(invMot[0],invMot[1],invMot[2]);
          //bot.allStop();
          delay(1000);
          break;
        case 3: // black
          break;
      }
    }
    **/
  // IR MOVEMENT
    byte lastMov = 0;
    if(MOTOR_ON == 1) {
      
      switch (IR_BEST) { //ultra temp
        
        case 0:
          bot.allStop(); //can't find anything
          lastMov = 0;
          break;
          
        case 1: //front 
          //bot.allStop();
          bot.manualMotors(30,0,-30); //move left
          //bot.manualMotors(-10,10,30); //rotate left and move forwards
          lastMov = 1;
          break;
        case 2: //straight mid front
          //bot.allStop();
          bot.moveStraight(0,30); //?
          lastMov = 2;
          break;
        case 3: //front left
          //bot.allStop();
          bot.manualMotors(-30,0,10); //        move right
          //bot.manualMotors(-10,10,-30); //rotate right and move forwards
          lastMov = 3;
          break;
          
        case 4://mid right
          bot.manualMotors(-30,0,30); //MOVE RIGHT is slightly back anyway due to triangle
        /**
          if(( millis() - lastBR) < 100) {
            bot.allStop();
            bot.moveStraight(-180,10); //move backwards staggered
            if(( millis() - lastBR > 500)) {
              lastBL = millis();
            }
          }
          else {
            bot.allStop();
            bot.manualMotors(60,0,-60);  //move to the right staggered
          }
          **/
          lastMov = 4;
          break;
        case 5: //directly behind
          bot.manualMotors(60,0,-60); //move to the right (doesn't matter right or left)
          lastMov = 5;
          break;

        case 6: //mid left
          bot.manualMotors(10,0,-10); //move left is slightly back anyway due to triangel
        /**
          if(( millis() - lastBL) > 100) {
            bot.allStop();
            bot.moveStraight(-180,10); //move backwards staggered
            if(( millis() - lastBL > 500)) {
              lastBL = millis();
            }
            
            bot.allStop();
          }
          else {
            bot.allStop();
            bot.manualMotors(-60,0,60);  //move to the left staggered
            
          }
          **/
          lastMov = 6;
          break;
      }
    }
  }
  else {
    bot.allStop();
    doneCallib = false;
    Serial.println(HMC5883L.getRealBearing());
  }
}