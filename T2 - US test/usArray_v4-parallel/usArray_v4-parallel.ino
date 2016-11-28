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
#define d_trig 0
#define d_echo 1

int usArray[6] = {0,0,0,0,0,0};
long pulseDurations[6] = {0,0,0,0,0,0};

void pulseAll(){
  //reset pins
  //digitalWrite(trigPin, LOW);
  PORTB = B00000000;
  PORTD = B00000000;
  delayMicroseconds(2);

  //send out a pulse
  //digitalWrite(trigPin, HIGH);
  PORTB = PORTB|B00010101;
  PORTD = PORTD|B01010100;
  delayMicroseconds(10); // Added this line

  //digitalWrite(trigPin, LOW);
  PORTB = PORTB&B11101010;
  PORTD = PORTD&B01010101;
}

void parallelMeasure() {
  int tStart = millis();
  int tEnd = 0;
  bool keepLooping = 1;
  int pD = PIND;
  int pB = PINB;
  int sensorState = B00000000;
  int sensorArray[6] = {0,0,0,0,0,0};
  long startTime[6] = {0,0,0,0,0,0};
  long endTime[6] = {0,0,0,0,0,0};
  char j = 0;

  do {
    keepLooping = 0;

    for (size_t i = 0; i < 3; i++) {
      j = i+3;
      if (sensorState&(B00000001 << i) != pD&(B00001000 << (i*2))) {
        sensorArray[5-i]++;
        if (sensorArray[5-i] == 1) {
          sensorState = sensorState|(B00000001 << i);
          startTime[5-i] = micros();
        } else if (sensorArray[5-i] == 2) {
          endTime[5-i] = micros();
        }
      }

      if (sensorState&(B00000001 << j) != pB&(B00000010 << (i*2))) {
        sensorArray[5-j]++;
        if (sensorArray[5-j] == 1) {
          sensorState = sensorState|(B00000001 << j);
          startTime[5-j] = micros();
        } else if (sensorArray[5-j] == 2) {
          endTime[5-j] = micros();
        }
      }
    }

    for (size_t i = 0; i < 6; i++) {
        if (sensorArray[i] != 2) {
          keepLooping = 1;
        }
      }

  } while(keepLooping);

  for (size_t i = 0; i < 6; i++) {
      pulseDurations[i] = endTime[i] - startTime[i];
  }
}

void parallelConvert() {
  for (size_t i = 0; i < 6; i++) {
    usArray[i] = pulseDurations[i]/58;
    if (usArray[i] > 300) {
      usArray[i] = 300;
    }
  }
}
void setup() {
  Serial.begin (19200);

  //DDRD = B01010110
  //DDRB = B00010101

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
  long tStart = micros();
  long tEnd = 0;
  int i = 0;

  pulseAll();
  parallelMeasure();
  parallelConvert();

  for (i=0; i<6; i++) {
    Serial.print(usArray[i]);
    if (i != 5) {
      Serial.print(',');
    }
  }
  //delay(50);
  tEnd = micros();
  Serial.print("execution time (micros)- ");
  Serial.println(tEnd - tStart);
}
