// Module name - T6_PID-Test.ino
// Module Description -
//
//Bsed on interrupt code by J. Richardson
//Author - C.Samarakoon
//Created - 14/11/2016
//Modified - 14/11/2016

#include<Servo.h>
//#include<String.h>

Servo leftWheelServo; //Right
Servo rightWheelServo; //Left

//#define leftSensorPin_I 3
//#define rightSensorPin_I 2
#define ppr 192
#define radius 10

#define leftSensorPin_I 2
#define rightSensorPin_I 3
#define leftPWMPin_O 10
#define rightPWMPin_O 9

bool Ltoggle = false;
int LStart = 0;
int LEnd = 0;
float LSpeed = 0.0;
int Lperiod = 0;

bool Rtoggle = false;
int RStart = 0;
int REnd = 0;
float RSpeed = 0.0;
int Rperiod = 0;

const unsigned int maxRPM = 150;
const unsigned int wheelCircum = 785; //mm (Wheel diameter is 25cm)
//const unsigned int pulsesPerRevolution = 192;
const float pidPeriod = 200; //ms

bool interToggle = false;

volatile int speedval = 90;
volatile int timerCount = 0;

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

volatile int desiredSpeed = 60;

volatile int desiredLeftPIDCount = 0;
volatile int desiredRightPIDCount = 0;

boolean leftDirection = 0; // 0 - forward, 1 - backward
boolean rightDirection = 0; // 0 - forward, 1 - backward

char serialCommand = 0;
int serialValue= 0;

int timercount=0;

float vLeft = 0;
float vRight = 0;
int speedOffset = 90; //offset from 90*
int dist = 0;
char dir = 0;
char cmdState = 1;
String inputBuffer = "";

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  // Set up hardware interrupts
  attachInterrupt(digitalPinToInterrupt(leftSensorPin_I) , leftSensorISR , CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightSensorPin_I), rightSensorISR, CHANGE);

  leftWheelServo.attach(10);
  rightWheelServo.attach(9);

  leftWheelServo.write(90);
  rightWheelServo.write(90);

  //pinMode(rightSensorPin_I, INPUT);
  //pinMode(leftSensorPin_I, INPUT);

  // Set up timer interrupt
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 124; // Compare value

  TCCR2A |= (1 << WGM21); // CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler
  TIMSK2 |= (1 << OCIE2A);
}

//
// Hardware Interrupts for counting rotary encoder pulses
//

void leftSensorISR () {
  leftPulseCount ++;
  leftSensorDistanceCount ++;
}

void rightSensorISR () {

  rightPulseCount ++;
  rightSensorDistanceCount ++;
}


ISR(TIMER2_COMPA_vect) {

  sei(); //Re-enable all interrupts

  if (timerCount > 24){ //63.9ms
    LSpeed = ((60000/pidPeriod)*leftPulseCount/ppr);
    RSpeed = ((60000/pidPeriod)*rightPulseCount/ppr);
    
    leftError = desiredLeftPIDCount - leftPulseCount;
    rightError = desiredRightPIDCount - rightPulseCount;
    
    leftOffset  += (leftKp  * (leftError  - leftLastError )) + (leftKi  * leftError );
    rightOffset += (rightKp * (rightError - rightLastError)) + (rightKi * rightError);

    leftLastError = leftError;
    rightLastError = rightError;
  
    leftPulseCount = 0;
    rightPulseCount = 0;

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
      
    timerCount = 0;
    //cmdState = 1;
    //Serial.println(cmdState, DEC);
    
  } else {
    timerCount++;
  }
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

char motorActuator(int dir, int dist){
  int sRight = 0;
  int sLeft = 0;
  int tDelay = 0;

  //Serial.print("Setting servo to ");
  if ((dir>0) && (dir<5)){
    switch (dir) {
      case 1: //Forward
        sRight = 90+speedOffset;
        sLeft = 90+speedOffset;
        tDelay = 2000;
        break;
      case 2: //Reverse
        sRight = 90-speedOffset;
        sLeft = 90-speedOffset;
        tDelay = 2000;
        break;
      case 3: //Left
        sRight = 90+(speedOffset);
        sLeft = 90-(speedOffset);
        tDelay = 1000;
        break;
      case 4: //Right
        sRight = 90-(speedOffset);
        sLeft = 90+(speedOffset);
        tDelay = 1000;
        break;
      
      desiredLeftPIDCount = (desiredSpeed / (float)60) * ppr / (float)(1000 / pidPeriod);
      desiredRightPIDCount = (desiredSpeed/ (float)60) * ppr / (float)(1000 / pidPeriod);
    }
  } else {
      cmdState = 5;
      return 5;
  }

  cmdState = 2;
  //Serial.println("2"); //setting motor
  leftWheelServo.write(sRight);
  rightWheelServo.write(sLeft);
  // delay(tDelay);
  // leftWheelServo.write(90);
  // rightWheelServo.write(90);
  // return 0;
}

void sendPlotData(String seriesName, float data)
{
  Serial.print("{");
  Serial.print(seriesName);
  Serial.print(",T,");
  Serial.print(data);
  Serial.println("}");
}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(3000);
  //motorActuator(1, dist);
  
  if (Serial.available() > 0) {
      dist = 0;
      dir = 0;
      inputBuffer = "";
      //Serial.print("Data Rec...");
      //Serial.println("3");  //data recieved
      inputBuffer = Serial.readStringUntil('/n');
      //Serial.println(inputBuffer);
      switch (inputBuffer[0]) {
        case 70: // char F
          dir = 1;
          cmdState = 3;
          break;
        case 66: // char B
          dir = 2;
          cmdState = 3;
          break;
        case 82: // char L
          dir = 3;
          cmdState = 3;
          break;
        case 76: // char R
          dir = 4;
          cmdState = 3;
          break;
        case 88: //char X or halt
          cmdState = 3;
          leftWheelServo.write(90);
          rightWheelServo.write(90);
          cmdState = 1;
      }
      serialFlush();
      timerCount = 0;
      if (cmdState == 3) {
        //Serial.println(dir);
        motorActuator(dir, dist);
      }
  }
  Serial.print(LSpeed);
  Serial.print(" ");
  Serial.print(RSpeed);
  Serial.print(" ");
  Serial.print(leftPulseCount);
  Serial.print(" ");
  Serial.print(rightPulseCount);
  Serial.print('\r');
  Serial.print('\n');

  //Serial.print("%f %f %u %u %u %u\r", LSpeed, RSpeed,0,0,0,0);
}
