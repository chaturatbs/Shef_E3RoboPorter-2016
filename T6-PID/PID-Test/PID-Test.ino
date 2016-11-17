// Module name - T4_cmdTest.ino
// Module Description -
//
//Bsed on interrupt code by J. Richardson
//Author - C.Samarakoon
//Created - 14/11/2016
//Modified - 14/11/2016

#include<Servo.h>
//#include<String.h>

Servo test1; //Right
Servo test2; //Left

#define leftSense 3
#define rightSense 2
#define ppr 130
#define radius 10

volatile int speedval = 90;
volatile int count = 0;

float vLeft = 0;
float vRight = 0;
int speedOffset = 50; //offset from 90*
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

  Serial.begin(19200);

  test1.attach(10);
  test2.attach(9);
  test1.write(90);
  test2.write(90);

  pinMode(rightSense, INPUT);
  pinMode(leftSense, INPUT);
  // Set up timer interrupt
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 124; // Compare value

  TCCR2A |= (1 << WGM21); // CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler
  TIMSK2 |= (1 << OCIE2A);
}

ISR(TIMER2_COMPA_vect) {
  if (count > 200){ // Roughly 0.5s
//    //if (speedval != 90){
//    speedval = 90;
//    //Serial.println("resetting motor...");
//    test1.write(speedval);
//    test2.write(speedval);
//    //}
    count = 0;
    //cmdState = 1;
    Serial.println(cmdState, DEC);
  } else {
    count++;
  }
}

void getPulseWidth(){
  float rightPW = 0;
  float leftPW = 0;
  
  rightPW = pulseIn(rightSense, HIGH, 60000);
  leftPW = pulseIn(leftSense, HIGH, 60000);

  vRight = 60000000/(rightPW*ppr);
  vLeft = 60000000/(leftPW*ppr);  
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
    }
  } else {
      cmdState = 5;
      return 5;
  }

  cmdState = 2;
  Serial.println("2"); //setting motor
  test1.write(sRight);
  test2.write(sLeft);
  // delay(tDelay);
  // test1.write(90);
  // test2.write(90);
  // return 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
      dist = 0;
      dir = 0;
      inputBuffer = "";
      //Serial.print("Data Rec...");
      Serial.println("3");  //data recieved
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
          test1.write(90);
          test2.write(90);
          cmdState = 1;
      }
      serialFlush();
      count = 0;
      if (cmdState == 3) {
        //Serial.println(dir);
        motorActuator(dir, dist);
      }
  }
  //Serial.println(cmdState, DEC);
  getPulseWidth();
  Serial.print(vRight);
  Serial.print(",");
  Serial.print(vLeft);
  Serial.println();
  delay(10);
}
