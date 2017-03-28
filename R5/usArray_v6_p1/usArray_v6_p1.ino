//define pins

#define c_trig 4
#define c_echo 5

#define fl_trig 10
#define fl_echo 11
#define fr_trig 8
#define fr_echo 9

#define ft_trig 6
#define ft_echo 7

#define bl_trig 12
#define bl_echo 13
#define bc_trig 2
#define bc_echo 3

#define br_trig A0
#define br_echo A1

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

  //Serial.print("measured");
  if (tEnd-tStart < 50000) {
    distance = duration/58;
    if (distance > 200) {
      distance = 200;
    }
  } else {
    distance = 270;
  }
  //Serial.print(distance);
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
  pinMode(ft_trig, OUTPUT);
  pinMode(ft_echo, INPUT);
  pinMode(bl_trig, OUTPUT);
  pinMode(bl_echo, INPUT);
  pinMode(bc_trig, OUTPUT);
  pinMode(bc_echo, INPUT);
  pinMode(br_trig, OUTPUT);
  pinMode(br_echo, INPUT);
}

void loop() {
  int usArray[7];
  long tStart = micros();
  long tEnd = 0;
  int i = 0;

  //Serial.print("starting");
  // ft,fb,fl,fr,bl,bm,br

  usArray[2] = usMeasure(fl_trig, fl_echo);
  usArray[4] = usMeasure(bl_trig, bl_echo);
  usArray[1] = usMeasure(c_trig, c_echo);
  usArray[6] = usMeasure(br_trig, br_echo);
  usArray[3] = usMeasure(fr_trig, fr_echo);
  usArray[5] = usMeasure(bc_trig, bc_echo);
  usArray[0] = usMeasure(ft_trig, ft_echo);

   //Serial.print(usArray[0]);
   //Serial.print(usArray[1]);
   //Serial.println("end");
  for (i=0; i<7; i++) {
    Serial.print(usArray[i]);
    if (i != 6) {
      Serial.print(',');
    }
  }
  tEnd = micros();
  //Serial.print("execution time (micros)- ");
  //Serial.println(tEnd - tStart);
  Serial.print('\n');
  //delay(50);
}
