#include <PID_AutoTune_v0.h>

#include <PID_v1.h>

#include <MegunoLink.h>
#include <CommandHandler.h>
#include <ArduinoTimer.h>
#include <CircularBuffer.h>
#include <EEPROMStore.h>
#include <Filter.h>

#include <Servo.h>

TimePlot LeftWheelPlot;

//
// Pin assignments
//

const byte leftSensorPin_I  = 3;
const byte rightSensorPin_I = 2;
const byte leftPWMPin_O  = 9;
const byte rightPWMPin_O = 10;

//
// Constants
//

float leftKp  = 1; // NEED TO CHANGE
float leftKi  = 0; // NEED TO CHANGE
float leftKd  = 0; // NEED TO CHANGE
float rightKp = 0; // NEED TO CHANGE
float rightKi = 0; // NEED TO CHANGE
float rightKd = 0; // NEED TO CHANGE

const unsigned int maxRPM = 120; // Set the maximum speed that can be set

const unsigned int wheelCircum = 785; //mm (Wheel diameter is 25cm)

const unsigned int pulsesPerRevolution = 192;

const unsigned int pidPeriod = 200; // in ms

//
// Create Servo objects
//

Servo leftWheelServo;
Servo rightWheelServo;

//
// Global variables
//

volatile double leftSensorPIDCount  = 0;
volatile unsigned int rightSensorPIDCount = 0;
volatile unsigned int leftSensorDistanceCount = 0;
volatile unsigned int rightSensorDistanceCount = 0;

volatile double leftOffset = 0;
volatile double rightOffset = 0;

volatile int desiredSpeed = 70;

volatile double desiredLeftPIDCount = 0;
volatile double desiredRightPIDCount = 0;


PID leftPID(&leftSensorPIDCount, &leftOffset, &desiredLeftPIDCount, leftKp, leftKi, leftKd, DIRECT);

PID_ATune leftPIDTune(&leftSensorPIDCount, &leftOffset);



char serialCommand = 0;
int serialValue= 0;

int timercount=0;


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
  TCCR2B |= (1 << CS22) | (1 << CS21) | (0 << CS20);
  TIMSK2 |= (1 << OCIE2A);
  
  // Set up outputs
  leftWheelServo.attach(9);
  rightWheelServo.attach(10);

  leftWheelServo.write(90);
  rightWheelServo.write(90);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(0,90);
  leftPID.SetSampleTime(100);


  // Initialise serial communication
  Serial.begin(19200);
  Serial.println("PID Controller Plot");
  
}

//
// Hardware Interrupts for counting rotary encoder pulses
//

void leftSensorISR () {
  leftSensorPIDCount ++;
  leftSensorDistanceCount ++;
  //Serial.print("*");
}

void rightSensorISR () {
  rightSensorPIDCount ++;
  rightSensorDistanceCount ++;
}

//
// PID Controller - for both wheels 
//

ISR (TIMER2_COMPA_vect) {

  sei();

  if (timercount >= 49) {


  LeftWheelPlot.SendData("Speed", leftSensorPIDCount);

  leftPID.Compute();

  // Clear sensor counters
  leftSensorPIDCount  = 0;
  rightSensorPIDCount = 0;
  
  leftWheelServo.write(90 + leftOffset);
  rightWheelServo.write(90 + rightOffset);


  timercount = 0;
  } else {
    timercount++;
  }





  
}

//
// Movement functions
//

void moveForward (int distance) {
 desiredLeftPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution / 10;
 desiredRightPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution / 10;

}

void moveBackward (int distance) {
 desiredLeftPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution;
 desiredRightPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution; 

}

void rotateLeft (int angle) {
  
}

void rotateRight (int angle) {
  
}

// Set Speed function - sets the forward/reverse speed in rpm - not intended to be used in normal operation, only for testing
void setSpeed (int speed) { 
  if ((speed >= 0) && (speed <= maxRPM)) {
    desiredSpeed = speed;
  } else {
    desiredSpeed = 0;
    Serial.print("Error, invalid speed set. Speed must be between 0 and ");
    Serial.println(maxRPM);
  }
  
}

// Immediately stops both wheels
void stopMoving () {
  leftWheelServo.write(90);
  rightWheelServo.write(90);

  desiredLeftPIDCount = 0;
  desiredRightPIDCount = 0;  
}

//
// Main Loop
//

void loop() {
  leftPIDTune.Runtime();
  if (Serial.available() >= 2) {
    serialCommand = Serial.read();
    serialValue = Serial.parseInt();

    
    if (serialCommand == 'X') {
      stopMoving();
    } else if (serialCommand == 'F') {
      moveForward(serialValue);
    } else if (serialCommand == 'B') {
      moveBackward(serialValue);
    } else if (serialCommand == 'L') {
      rotateLeft(serialValue);
    } else if (serialCommand == 'R') {
      rotateRight(serialValue);
    } else if (serialCommand == 'S') {
      setSpeed(serialValue);
    } else if (serialCommand == 'P') {
      leftKp = (float)serialValue/(float)10;
    } else if (serialCommand == 'I') {
      leftKi = (float)serialValue/(float)10;
    } else if (serialCommand == 'D') {
      leftKd = (float)serialValue/(float)10;
    } else {
      stopMoving();
    }

    
  }

}

