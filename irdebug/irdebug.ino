/**
  Richardbot 0.0.1

*/

#include <Wire.h> //used for compass sensor
#include "Compass.h" // only for callibration
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
compass HMC5883L(20, 21, 0x1E);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // assign a unique id to sensor
int initialHeading;
#define REWRITE_CALLIBRATION 0 //only set to 1 if you want to recallibrate
#include "RB_Movement.h"
#include "RB_Motor.h"

// manual manualMotors reference code
//      right anti, lef anti, back anticlockwise
//bot.manualMotors(1,1,1)
Movement bot(6, 7, 4, 5, 3, 2); //this creates an instance of the object and sets motor pins
#define MOTOR_ON 1
//----------Switch config-------//
#define CALLIBRATE_PIN 13
#define RUN_PIN 12


//----------IR CONFIG-----------//
//Maximum IR counter Value (max 255)
#define MAX_COUNTER 32
#define MIN_THRESHOLD 10
// * The number of IR sensors on the robot. +1 for the "disabled" 0 sensor
#define IR_NUM 7 + 1
// * The pin used for powering all of the IR sensors (in parallel)
#define IR_UNLOCK_PIN 23
// * Enable or disable debug info
// * No noticable performance impact normally.
// * To enable, set to 1.  To disable, set to 0

// * Sensor to Arduino pin mappings
// * Note: Analog sensors follow on from digital pins (or use A0 notation)
// * If a sensor is broken set it to 255 to disable it
//SENSOR: 1   2   3 4  5   6   7   8   9  10  11 12
byte IRSensors[IR_NUM] = {255, 48, 50, 52,53, 51, 49, 47};
//SENSOR POSITOINS             FL  MD  FR  
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

#define DEBUG 1 //master (en serial)
#define DEBUG_COMPASS 0
#define DEBUG_IR 1
#define DEBUG_CLR 0

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
  for (byte i = 0; i < IR_NUM; i++) {
    IRValues[i] = 0;
    pinMode(IRSensors[i], INPUT);
  }
}

void readIR() {
  for (byte i = 0; i < IR_NUM; i++) { //iterate over each sensor if not disabled
    if (IRSensors[i] < 255) { // if sensor not disabled
      IRValues[i] += (digitalRead(IRSensors[i]) == HIGH ? 0 : 1);// if sensor is high, add 1
    }
  }
  counter++;
  if (counter >= MAX_COUNTER) {
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

  for (byte i = 0; i < IR_NUM; i++) { //find highest value
    if (IRValues[i] > b_value) {
      b_index = i;
      b_value = IRValues[i];
    }
    // debug stuff, show us the value of each sensor
    if (DEBUG_IR) {
      // Serial.print(i, DEC);
      // Serial.print(",");
      // Serial.print(IRSensors[i]);
      // Serial.print("=");
      // Serial.print(IRValues[i], DEC);
      // Serial.print(" | ");
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
    // Serial.print("BEST = ");
    // Serial.print(IR_BEST, DEC);
    // Serial.print(" (");
    // Serial.print(IRSensors[b_index], DEC);
    // Serial.print("): ");
    // Serial.println(b_value, DEC);
}
//-----------Arduino base code-------------//
void setup() {
  // Enable Debug
  Serial.begin(DEBUG_BAUD_RATE);
  initIR();

}
void loop() {
  readIR();
}
