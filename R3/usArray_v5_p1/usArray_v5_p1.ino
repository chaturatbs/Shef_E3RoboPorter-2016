//define pins

#define c_trig 12
#define c_echo 13
#define fl_trig 10
#define fl_echo 11
#define fr_trig 8
#define fr_echo 9
#define l_trig 6
#define l_echo 7
#define r_trig 4
#define r_echo 5
#define b_trig 2
#define b_echo 3
#define d_trig A0
#define d_echo A1

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
  tStart = mircos();
  duration = pulseIn(echoPin, HIGH, timeout);
  tEnd = micros();

  if tEnd-tStart < timeout {
    distance = duration/58;
    if (distance > 200) {
      distance = 200;
    }
  } else {
    distance = 250;
  }

  return distance;
}

void setup() {
  Serial.begin (19200);
  pinMode(c_trig, OUTPUT);
  pinMode(c_echo, INPUT);
  pinMode(fl_trig, OUTPUT);
  pinMode(fl_echo, INPUT);
  pinMode(fr_trig, OUTPUT);
  pinMode(fr_echo, INPUT);
  pinMode(l_trig, OUTPUT);
  pinMode(l_echo, INPUT);
  pinMode(r_trig, OUTPUT);
  pinMode(r_echo, INPUT);
  pinMode(b_trig, OUTPUT);
  pinMode(b_echo, INPUT);
}

void loop() {
  int usArray[6];
  long tStart = micros();
  long tEnd = 0;
  int i = 0;

  usArray[0] = usMeasure(c_trig, c_echo);
  usArray[3] = usMeasure(l_trig, l_echo);
  usArray[4] = usMeasure(r_trig, r_echo);
  usArray[1] = usMeasure(fl_trig, fl_echo);
  usArray[5] = usMeasure(b_trig, b_echo);
  usArray[2] = usMeasure(fr_trig, fr_echo);

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
  //delay(50);
}
