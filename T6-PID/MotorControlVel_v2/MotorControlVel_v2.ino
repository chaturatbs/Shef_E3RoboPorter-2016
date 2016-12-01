#include <Servo.h>

//
// Pin assignments
//

const byte leftSensorPin_I  = 2;
const byte rightSensorPin_I = 3;
const byte leftPWMPin_O  = 10;
const byte rightPWMPin_O = 9;

//
// Constants
//

const unsigned int maxRPM = 120; 
//const unsigned int wheelCircum = 785; //mm (Wheel diameter is 25cm)
const unsigned int pulsesPerRevolution = 192;
const unsigned int pidPeriod = 200; //ms

//
// Create Servo objects
//

Servo leftWheelServo;
Servo rightWheelServo;

//
// Global variables
//


float leftKp  = 0;//0.2;
float leftKi  = 1;//0.5;
float rightKp = 0;
float rightKi = 1;

volatile int leftSensorPIDCount  = 0;
volatile int rightSensorPIDCount = 0;

volatile int leftSensorDistanceCount = 0;
volatile int rightSensorDistanceCount = 0;

volatile int leftError = 0;
volatile int rightError = 0;

volatile int leftLastError = 0;
volatile int rightLastError = 0;

volatile int leftOffset = 0;
volatile int rightOffset = 0;

volatile int desiredSpeed = 60;

volatile int desiredLeftPIDCount = 0;
volatile int desiredRightPIDCount = 0;

boolean leftDirection = 0; // 0 - forward, 1 - backward
boolean rightDirection = 0; // 0 - forward, 1 - backward

int skip = 0;
int leftSerialSpeed = 0;
int rightSerialSpeed = 0;

int timercount=0;

//
// Intial Setup
//

void setup() {

  // Set up hardware interrupts
  attachInterrupt(digitalPinToInterrupt(leftSensorPin_I) , leftSensorISR , FALLING);
  attachInterrupt(digitalPinToInterrupt(rightSensorPin_I), rightSensorISR, FALLING);

  // Set up timer interrupt
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 124; // Compare value

  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler
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
  leftSensorPIDCount ++;
  leftSensorDistanceCount ++;
}

void rightSensorISR () {
  rightSensorPIDCount ++;
  rightSensorDistanceCount ++;
}

//
// PID Controller - for both wheels 
//

ISR (TIMER2_COMPA_vect) {

  sei(); //Re-enable all interrupts

  if (timercount >= 12) {

  leftError = desiredLeftPIDCount - leftSensorPIDCount;
  rightError = desiredRightPIDCount - rightSensorPIDCount;
  Serial.print(leftSensorPIDCount);
  Serial.print(",");
  Serial.println(rightSensorPIDCount);
  
  leftSensorPIDCount = 0;
  rightSensorPIDCount = 0;

  leftOffset  += (leftKp  * abs((leftError  - leftLastError ))) + (leftKi  * leftError );
  rightOffset += (rightKp * abs((rightError - rightLastError))) + (rightKi * rightError);

  leftLastError = leftError;
  rightLastError = rightError;
  
  leftOffset = constrain(leftOffset, 0, 90 - (leftDirection * 30)); // If forwards, then 0-90. If backwards, then 0-60
  rightOffset = constrain(rightOffset, 0, 90 - (rightDirection * 30));

  if (leftDirection == 0){
    leftWheelServo.write(90 + leftOffset);   
  } else {
    leftWheelServo.write(90 - leftOffset);
  }
  if (rightDirection == 0){
    rightWheelServo.write(90 + rightOffset);   
  } else {
    rightWheelServo.write(90 - rightOffset);
  }

  timercount = 0;
  } else {
    timercount++;
  }
}

void moveRobot (int left, int right) {
  if (left >= 0) {
    leftDirection = 0;    
  } else {
    leftDirection = 1;
  }
  if (right >= 0) {
    rightDirection = 0;
  } else {
    rightDirection = 1;
  }

 desiredLeftPIDCount = (abs(left) / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
 desiredRightPIDCount = (abs(right) / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);

//      Serial.print(" - Left count is: ");
//      Serial.print(desiredLeftPIDCount);
//      
//      Serial.print(" Right count is: ");
//      Serial.println(desiredRightPIDCount);

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

void stopMoving () {
  desiredLeftPIDCount = 0;
  desiredRightPIDCount = 0;  
}

//
// Main Loop
//

void loop() {

  if (Serial.available() >= 4) {
      leftSerialSpeed = Serial.parseInt();
      skip = Serial.read();
      rightSerialSpeed = Serial.parseInt();  
      skip = Serial.read();

      Serial.print("Left speed is: ");
      Serial.print(leftSerialSpeed);
      Serial.print(" Right speed is: ");
      Serial.print(rightSerialSpeed);

      moveRobot(leftSerialSpeed, rightSerialSpeed);
  }

}

