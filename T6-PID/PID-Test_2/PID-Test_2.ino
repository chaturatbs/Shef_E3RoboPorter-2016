#include <Servo.h>
Servo leftWheelServo;
Servo rightWheelServo;

#define ppr 192
#define radius 10

#define leftSensorPin_I 2
#define rightSensorPin_I 3
#define leftPWMPin_O 10
#define rightPWMPin_O 9

const unsigned int maxRPM = 150;
const unsigned int wheelCircum = 785; //mm (Wheel diameter is 25cm)
const unsigned int pulsesPerRevolution = 192;
const unsigned int pidPeriod = 200; //ms

float leftKp  = 0.1;
float leftKi  = 0.7;
float rightKp = 0.1;
float rightKi = 0.7;

volatile int leftPulseCount  = 0;
volatile int rightPulseCount = 0;

volatile int leftSensorDistanceCount = 0;
volatile int rightSensorDistanceCount = 0;

volatile int leftError = 0;
volatile int rightError = 0;

volatile int leftLastError = 0;
volatile int rightLastError = 0;

volatile int leftOffset = 0;
volatile int rightOffset = 0;

volatile int desiredSpeed_L = 60;
volatile int desiredSpeed_L = 60;

float vLeft = 0;
float vRight = 0;

volatile int desiredLeftPIDCount = 0;
volatile int desiredRightPIDCount = 0;

boolean leftDirection = 0; // 0 - forward, 1 - backward
boolean rightDirection = 0; // 0 - forward, 1 - backward

char serialCommand = 0;
int serialValue= 0;

int timercount=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Set up hardware interrupts
  attachInterrupt(digitalPinToInterrupt(leftSensorPin_I) , leftSensorISR , CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightSensorPin_I), rightSensorISR, CHANGE);

  // Set up outputs
  leftWheelServo.attach(leftPWMPin_O);
  rightWheelServo.attach(rightPWMPin_O);

  leftWheelServo.write(90);
  rightWheelServo.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:

}
