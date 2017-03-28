
//define pins

#define lf_trig 12
#define lf_echo 13
#define lc_trig 10
#define lc_echo 11
#define lb_trig 8
#define lb_echo 9
#define rf_trig 6
#define rf_echo 7
#define rc_trig 4
#define rc_echo 5
#define rb_trig 2
#define rb_echo 3

//#import String

int usMeasure (int trigPin, int echoPin) {
  long duration =0 , distance = 0;
  long tStart = 0, tEnd = 0;
  int timeout = 50000;
  //reset pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //send out a pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);

  //measure pulse
  tStart = micros();
  duration = pulseIn(echoPin, HIGH, 50000);
  tEnd = micros();

  if ((tEnd-tStart) < 50000) {
    distance = duration/58;
    if (distance > 250) {
      distance = 250;
    }
  } else {
    Serial.println("Timeout");
    distance = 270;
  }

  return distance;
}

void setup() {
  Serial.begin (19200);

  pinMode(lf_trig, OUTPUT);
  pinMode(lf_echo, INPUT);
  pinMode(lc_trig, OUTPUT);
  pinMode(lc_echo, INPUT);
  pinMode(lb_trig, OUTPUT);
  pinMode(lb_echo, INPUT);

  pinMode(rf_trig, OUTPUT);
  pinMode(rf_echo, INPUT);
  pinMode(rc_trig, OUTPUT);
  pinMode(rc_echo, INPUT);
  pinMode(rb_trig, OUTPUT);
  pinMode(rb_echo, INPUT);

}

void loop() {
  int usArray[6];
  long tStart = micros();
  long tEnd = 0;
  int i = 0;
  //String userInput = ""

  // if (Serial.available() > 0){
  //   userInput = Serial.readStringUntil("\n");
  //   if userInput == "id?"
  //     Serial.print('us2\n');
  // }

  // lf,lc,lb,rf,rc,rb

  usArray[0] = usMeasure(lf_trig, lf_echo);
  usArray[5] = usMeasure(rb_trig, rb_echo);
  usArray[2] = usMeasure(lb_trig, lb_echo);
  usArray[3] = usMeasure(rf_trig, rf_echo);
  usArray[1] = usMeasure(lc_trig, lc_echo);
  usArray[4] = usMeasure(rc_trig, rc_echo);
  usArray[4] = usMeasure(rc_trig, rc_echo);

   //Serial.print(usArray[0]);
   //Serial.print(usArray[1]);
   //Serial.println("end");
  for (i=0; i<6; i++) {
    Serial.print(usArray[i]);
    if (i != 5) {
      Serial.print(',');
    }
  }

  tEnd = micros();
  //Serial.print("execution time (micros)- ");
  //Serial.println(tEnd - tStart);
  Serial.print('\n');

}
