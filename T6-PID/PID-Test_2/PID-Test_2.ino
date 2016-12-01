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
//const unsigned int wheelCircum = 785; //mm (Wheel diameter is 25cm)
//const unsigned int pulsesPerRevolution = 192;
const unsigned int pidPeriod = 200; //ms

float leftKp  = 1;//0.1;
float leftKi  = 0;//0.7;
float rightKp = 1;//0.1;
float rightKi = 0;//0.7;

//volatile int leftPulseCount  = 0;
//volatile int rightPulseCount = 0;
//
//volatile int leftSensorDistanceCount = 0;
//volatile int rightSensorDistanceCount = 0;

volatile int leftError = 0;
volatile int rightError = 0;

volatile int leftLastError = 0;
volatile int rightLastError = 0;

volatile int leftOffset = 0;
volatile int rightOffset = 0;

volatile int vLeftDemand = 60;
volatile int vRightDemand = 60;

float vLeft = 0;
float vRight = 0;
//
//volatile int desiredLeftPIDCount = 0;
//volatile int desiredRightPIDCount = 0;

boolean leftDirection = 0; // 1 - forward, -1 - backward
boolean rightDirection = 0; // 1 - forward, -1 - backward

char serialCommand = 0;
int serialValue= 0;
int rightPW = 0;
int rightPW_avg = 0;
int leftPW = 0;
int leftPW_avg = 0;
int timercount=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);

  // Set up hardware interrupts
  attachInterrupt(digitalPinToInterrupt(leftSensorPin_I) , leftSensorISR , FALLING);
  attachInterrupt(digitalPinToInterrupt(rightSensorPin_I), rightSensorISR, FALLING);

  // Set up outputs
  leftWheelServo.attach(leftPWMPin_O);
  rightWheelServo.attach(rightPWMPin_O);

  //leftWheelServo.write(90);
  //rightWheelServo.write(90);

  // Set up timer interrupt
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 124; // Compare value

  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler
  TIMSK2 |= (1 << OCIE2A);
}

void leftSensorISR () {
  leftPW = pulseIn(leftSensorPin_I,LOW);
  leftPW_avg += (int)((leftPW - leftPW_avg)/5);
  Serial.print("L");
  Serial.print(leftPW_avg);
}

void rightSensorISR () {
  rightPW = pulseIn(rightSensorPin_I,LOW);
  rightPW_avg += (int)((rightPW - rightPW_avg)/5);
  Serial.print("R");
  Serial.print(rightPW_avg);
  
}

ISR (TIMER2_COMPA_vect) {

  sei(); //Re-enable all interrupts
  
  if (timercount >= 24) {
    vLeft = leftPW_avg;
    vRight = rightPW_avg;
    
    leftError = vLeftDemand - vLeft;
    rightError = vRightDemand - vRight;
  
    Serial.println(vRight);
  

  //leftOffset  += (leftKp  * abs((leftError  - leftLastError ))) + (leftKi  * leftError );
  //rightOffset += (rightKp * abs((rightError - rightLastError))) + (rightKi * rightError);

  //leftLastError = leftError;
  //rightLastError = rightError;
  
  //leftOffset = constrain(leftOffset, 0, 90 - (leftDirection * 30)); // If forwards, then 0-90. If backwards, then 0-60
  //rightOffset = constrain(rightOffset, 0, 90 - (rightDirection * 30));

    //leftWheelServo.write(90 + (leftDirection*leftOffset));
    //rightWheelServo.write(90 + (rightDirection*rightOffset));  
    
    timercount = 0;
  } else {
    timercount++;
  }
}
void moveRobot (int left, int right) {
  if (left >= 0) {
    leftDirection = 1;    
  } else {
    leftDirection = -1;
  }
  if (right >= 0) {
    rightDirection = 1;
  } else {
    rightDirection = -1;
  }
  vLeftDemand = left;
  vRightDemand = right;
}
void loop() {
  // put your main code here, to run repeatedly:
  char skip;
  if (Serial.available() >= 4) {
      vLeftDemand = Serial.parseInt();
      skip = Serial.read();
      vRightDemand= Serial.parseInt();  
      skip = Serial.read();

      Serial.print("Left speed is: ");
      Serial.print(vLeftDemand);
      Serial.print(" Right speed is: ");
      Serial.print(vRightDemand);

      //moveRobot(vLeftDemand, vRightDemand);
  }
  leftWheelServo.write(170);
  rightWheelServo.write(170);  
}
