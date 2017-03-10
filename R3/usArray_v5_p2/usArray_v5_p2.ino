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

#import String

int usMeasure (int trigPin, int echoPin) {
  long duration =0 , distance = 0;
  long tStart = 0, tEnd = 0;
  int timeout = 60000;
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
  String userInput = ""

  if (Serial.available() > 0){
    userInput = Serial.readStringUntil("\n");
    if userInput == "id?"
      Serial.print('us2\n');
  }
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

}
