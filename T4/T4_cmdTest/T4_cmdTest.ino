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

volatile int speedval = 90;
volatile int count = 0;

int speedOffset = 60; //offset from 90*
int dist = 0;
char dir = 0;
char cmdState = 0;
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
  if (count > 250){ // Roughly 0.5s
    //if (speedval != 90){
    speedval = 90;
    //Serial.println("resetting motor...");
    test1.write(speedval);
    test2.write(speedval);
    //}
    count = 0;
    cmdState = 1;
    Serial.println(cmdState, DEC);
  } else {
    count++;
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
      //Serial.print("Data Rec...");
      Serial.println("3");  //data recieved
      inputBuffer = Serial.readStringUntil('/n');
      Serial.println(inputBuffer);
      switch (inputBuffer[0]) {
        case 70: // char F
          dir = 1;
          cmdState = 1;
          break;
        case 66: // char B
          dir = 2;
          cmdState = 1;
          break;
        case 82: // char L
          dir = 3;
          cmdState = 1;
          break;
        case 76: // char R
          dir = 4;
          cmdState = 1;
          break;
        case 88: //char X or halt
          test1.write(90);
          test2.write(90);
          cmdState = 1;
      }
      serialFlush();
      count = 0;
      if (cmdState == 1) {
        //Serial.println(dir);
        motorActuator(dir, dist);
      }
  }
}
