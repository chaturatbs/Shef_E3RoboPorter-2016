#include <Servo.h>
//#include "MegunoLink.h"

//TimePlot data;

//
// Pin assignments
//

const byte leftSensorPin_I  = 2;
const byte rightSensorPin_I = 3;

const byte leftPWMPin_O  = 9;
const byte rightPWMPin_O = 10;

//
// Constants
//

const unsigned int maxRPM = 120; 
const unsigned int wheelCircum = 785; //mm (Wheel diameter is 25cm)
const unsigned int pulsesPerRevolution = 360;
const unsigned int pidPeriod = 100; //ms

//
// Create Servo objects
//

Servo leftWheelServo;
Servo rightWheelServo;

//
// Global variables
//


float leftKp  = 0.1;
float leftKi  = 0.2;
float leftKd  = 0.1;
float rightKp = 0.1;
float rightKi = 0.2;
float rightKd = 0.1;

volatile int leftSensorPIDCount  = 0;
volatile int rightSensorPIDCount = 0;

volatile int leftSensorDistanceCount = 0;
volatile int rightSensorDistanceCount = 0;

volatile int leftError = 0;
volatile int rightError = 0;

volatile int leftLastError = 0;
volatile int rightLastError = 0;

volatile int leftErrorSum = 0;
volatile int rightErrorSum = 0;

volatile int leftErrorDiff = 0;
volatile int rightErrorDiff = 0;

volatile int leftOffset = 0;
volatile int rightOffset = 0;

volatile int desiredLeftPIDCount = 0;
volatile int desiredRightPIDCount = 0;

int skip = 0;
int leftSerialSpeed = 0;
int rightSerialSpeed = 0;

int timercount = 0;
int sendcount = 0;

char startByte;
char leftHex[4] = {0,0,0,'\0'};
char rightHex[4] = {0,0,0,'\0'};
char recNum;
char endByte;

String leftCountString;
String rightCountString;

char dump;
char *ptr;


//
// Intial Setup
//

void setup() {

  // Set up hardware interrupts
  attachInterrupt(digitalPinToInterrupt(leftSensorPin_I) , leftSensorISR , CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightSensorPin_I), rightSensorISR, CHANGE);

  // Set up timer interrupt
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 124; // Compare value

  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 << CS21) | (0 << CS20); // 1024 prescaler
  TIMSK2 |= (1 << OCIE2A);
  
  // Set up outputs
  leftWheelServo.attach(leftPWMPin_O);
  rightWheelServo.attach(rightPWMPin_O);

  leftWheelServo.write(90);
  rightWheelServo.write(90);

  // Initialise serial communication
  Serial.begin(19200);
}

//
// Hardware Interrupts for counting rotary encoder pulses
//

void leftSensorISR () {
  if(digitalRead(4) == digitalRead(2)) {
    leftSensorPIDCount --;
    leftSensorDistanceCount --;
  } else {
    leftSensorPIDCount ++;    
    leftSensorDistanceCount ++;
  }
}

void rightSensorISR () {
  if(digitalRead(5) == digitalRead(3)) {
    rightSensorPIDCount ++;
    rightSensorDistanceCount ++;
  } else {
    rightSensorPIDCount --;
    rightSensorDistanceCount --;
  }
}


//
// PID Controller - for both wheels 
//

ISR (TIMER2_COMPA_vect) {

  sei(); //Re-enable all interrupts

  if (timercount >= 49) {

  leftError = desiredLeftPIDCount - leftSensorPIDCount;
  rightError = desiredRightPIDCount - rightSensorPIDCount;

  leftSensorPIDCount = 0;
  rightSensorPIDCount = 0;

  leftErrorSum += leftError * leftKi;
  rightErrorSum += rightError * rightKi;  

  leftErrorSum = constrain(leftErrorSum, -60, 90);
  rightErrorSum = constrain(rightErrorSum, -60, 90);

  leftErrorDiff = leftError - leftLastError;
  rightErrorDiff = rightError - rightLastError;

  leftOffset  = (leftKp  * leftError) + leftErrorSum + (leftErrorDiff * leftKd);
  rightOffset = (rightKp * rightError) + rightErrorSum + (rightErrorDiff * rightKd);
  
  leftOffset = constrain(leftOffset, -60, 90); 
  rightOffset = constrain(rightOffset, -60, 90);

  Serial.print(leftOffset);
  Serial.print(", ");
  Serial.println(rightOffset);
   
  leftWheelServo.write(90 + leftOffset);
  rightWheelServo.write(90 + rightOffset);
//
//  if (sendcount >= 5) {
//
//    leftCountString = String(abs(leftSensorDistanceCount), HEX);
//    rightCountString = String(abs(rightSensorDistanceCount), HEX);
//
//    if (leftSensorDistanceCount >= 0) {
//      Serial.print("+");
//    } else {
//      Serial.print("-");
//    }
//        
//    Serial.print(leftCountString);
//    Serial.print(",");
//
//    if (rightSensorDistanceCount >= 0) {
//      Serial.print("+");
//    } else {
//      Serial.print("-");
//    }
//    Serial.println(rightCountString);
//    
//    leftSensorDistanceCount = 0;
//    rightSensorDistanceCount = 0;
//    sendcount = 0;
//  } else {
//    sendcount ++;
//  }


  
  timercount = 0;
  } else {
    timercount++;
  }


}

//
// Movement functions
//

void moveRobot (int left, int right) {

 desiredLeftPIDCount = (left / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
 desiredRightPIDCount = (right / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);

 

//      Serial.print(" - Left count is: ");
//      Serial.print(desiredLeftPIDCount);
//      
//      Serial.print(" Right count is: ");
//      Serial.println(desiredRightPIDCount);

}

//
// Main Loop
//

void loop() {

  if (Serial.available() >= 1) {

    startByte = Serial.read();

    if (startByte == '$'){    
      while(Serial.available() < 8){}
      
      leftHex[0] = Serial.read();
      leftHex[1] = Serial.read();
      leftHex[2] = Serial.read();
      rightHex[0] = Serial.read();
      rightHex[1] = Serial.read();
      rightHex[2] = Serial.read();
      recNum = Serial.read();
      endByte = Serial.read();   
               
      leftSerialSpeed = strtol(leftHex, &ptr, 16);
      rightSerialSpeed = strtol(rightHex, &ptr, 16);
      
      if (endByte == '\n'){
        Serial.println("VALID COMMAND");      
        moveRobot(leftSerialSpeed, rightSerialSpeed);
      } else {  
        Serial.println("INVALID COMMAND");
        moveRobot(0,0);
      }
    } else {
      moveRobot(0,0);
      Serial.println("INVALID COMMAND");
    }

  }

}

