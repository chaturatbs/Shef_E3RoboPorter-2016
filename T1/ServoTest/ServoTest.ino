#include<Servo.h>

Servo test1;
Servo test2;

volatile int speedval;

volatile int count = 0;

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   

void setup() {
  // put your setup code here, to run once:

  Serial.begin(19200);


  test1.attach(9);
  test2.attach(10);
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
    if (speedval != 90){
       speedval = 90;
       //Serial.println("resetting motor...");
       test1.write(speedval);
       test2.write(speedval);  
    }
    count = 0;
  } else {
    count++;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
      Serial.print("Data Rec...");
      speedval =Serial.parseInt();
      count = 0;
      test1.write(speedval);
      test2.write(speedval);
      
      Serial.print("Setting servo to ");
      Serial.println(speedval);
      serialFlush();
  }

}
