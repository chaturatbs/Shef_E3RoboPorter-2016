//define pins

#define t_trig 12
#define t_echo 13
#define lf_trig 10
#define lf_echo 11
#define rf_trig 8
#define rf_echo 9
#define lb_trig 6
#define lb_echo 7
#define bl_trig 4
#define bl_echo 5
#define rb_trig 2
#define rb_echo 3
#define br_trig A1
#define br_echo A0

int pulseAll(){
  digitalWrite(t_trig, LOW);
  digitalWrite(lf_trig, LOW);
  digitalWrite(rf_trig, LOW);
  digitalWrite(lb_trig, LOW);
  digitalWrite(rb_trig, LOW);
  digitalWrite(bl_trig, LOW);
  digitalWrite(br_trig, LOW);


  delayMicroseconds(2);

  //send out a pulse
  digitalWrite(t_trig, HIGH);
  digitalWrite(lf_trig, HIGH);
  digitalWrite(rf_trig, HIGH);
  digitalWrite(lb_trig, HIGH);
  digitalWrite(rb_trig, HIGH);
  digitalWrite(bl_trig, HIGH);
  digitalWrite(br_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(t_trig, LOW);
  digitalWrite(lf_trig, LOW);
  digitalWrite(rf_trig, LOW);
  digitalWrite(lb_trig, LOW);
  digitalWrite(rb_trig, LOW);
  digitalWrite(bl_trig, LOW);
  digitalWrite(br_trig, LOW);
}

int sendPulse(int trigPin, int echoPin){
  //reset pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //send out a pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
}

int usMeasure (int trigPin, int echoPin) {
  long duration =0 , distance = 0;

  //reset pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //send out a pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);

  //measure pulse
  duration = pulseIn(echoPin, HIGH, 50000);
  distance = duration/58;
  if (distance > 200) {
    distance = 200;
    //Serial.println(0);
  } else {
    //Serial.println(distance);
    //Serial.println(" cm");
  }
  return distance;
}

void setup() {
  Serial.begin (19200);
  pinMode(t_trig, OUTPUT);
  pinMode(t_echo, INPUT);
  pinMode(lf_trig, OUTPUT);
  pinMode(lf_echo, INPUT);
  pinMode(rf_trig, OUTPUT);
  pinMode(rf_echo, INPUT);
  pinMode(lb_trig, OUTPUT);
  pinMode(lb_echo, INPUT);
  pinMode(rb_trig, OUTPUT);
  pinMode(rb_echo, INPUT);
  pinMode(bl_trig, OUTPUT);
  pinMode(bl_echo, INPUT);
  pinMode(br_trig, OUTPUT);
  pinMode(br_echo, INPUT);
}

void loop() {
  int usArray[7];
  long tStart = micros();
  long tEnd = 0;
  int i = 0;

  usArray[0] = usMeasure(t_trig, t_echo);
  usArray[3] = usMeasure(lb_trig, lb_echo);
  usArray[4] = usMeasure(rb_trig, rb_echo);
  usArray[1] = usMeasure(lf_trig, lf_echo);
  usArray[5] = usMeasure(bl_trig, bl_echo);
  usArray[2] = usMeasure(rf_trig, rf_echo);
  usArray[6] = usMeasure(br_trig, br_echo);

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
