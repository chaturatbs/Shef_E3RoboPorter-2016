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
  PORTB = PORTB&B11000000;
  PORTD = PORTD&B00000011;
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

  do {
    keepLooping = 0;

    //check Back sensor
    if ((sensorState&B00000001) != (pD&B00001000)) {
      Serial.println("in IF");
      sensorArray[5]++;
      if (sensorArray[5] == 1) {
        Serial.println("5 is 1");
        sensorState = sensorState|B00000001;
        startTime[5] = micros();
      } else if (sensorArray[5] == 2) {
        Serial.println("5 is 2");
        endTime[5] = micros();
        //MOTE THIS OVERFLOWS! NEED PROTECTION.
      }
    }

    //check R sensor
    if ((sensorState&B00000010) != (pD&B00100000)) {
      sensorArray[4]++;
      if (sensorArray[4] == 1) {
        sensorState = sensorState|B00000010;
        startTime[4] = micros();
      } else if (sensorArray[4] == 2) {
        endTime[4] = micros();
      }
    }

    //check L sensor
    if ((sensorState&B00000100) != (pD&B10000000)) {
      sensorArray[3]++;
      if (sensorArray[3] == 1) {
        sensorState = sensorState|B00000100;
        startTime[3] = micros();
      } else if (sensorArray[3] == 2) {
        endTime[3] = micros();
      }
    }

    //check FR sensor
    if ((sensorState&B00001000) != (pB&B00000010)) {
      sensorArray[2]++;
      if (sensorArray[2] == 1) {
        sensorState = sensorState|B00001000;
        startTime[2] = micros();
      } else if (sensorArray[2] == 2) {
        endTime[2] = micros();
      }
    }

    //check FL sensor
    if ((sensorState&B00010000) != (pB&B00001000)) {
      sensorArray[1]++;
      if (sensorArray[1] == 1) {
        sensorState = sensorState|B00010000;
        startTime[1] = micros();
      } else if (sensorArray[1] == 2) {
        endTime[1] = micros();
      }
    }

    //check F sensor
    if ((sensorState&B00100000) != (pB&B00100000)) {
      sensorArray[0]++;
      if (sensorArray[0] == 1) {
        sensorState = sensorState|B00100000;
        startTime[0] = micros();
      } else if (sensorArray[0] == 2) {
        endTime[0] = micros();
      }
    }

    for (size_t i = 0; i < 6; i++) {
        if (sensorArray[i] != 2) {
          keepLooping = 1;
        }
    }
    
    tEnd = millis();
    if ((tEnd - tStart) > 60){
      Serial.print("measure timeout ");
      keepLooping = 0;
    }
  
  } while(keepLooping);
  
  for (size_t i = 0; i < 6; i++) {
      pulseDurations[i] = endTime[i] - startTime[i];
  }
}

void parallelConvert() {
  for (size_t i = 0; i < 6; i++) {
    usArray[i] = pulseDurations[i]/58;
    if ((usArray[i] > 300) || (usArray[i] < 0)) {
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

  //Serial.println("starting loop");
  pulseAll();
  //Serial.println("parallel measure");
  parallelMeasure();
  //Serial.println("parallel convert");
  parallelConvert();

  for (i=0; i<6; i++) {
    Serial.print(usArray[i]);
    if (i != 5) {
      Serial.print(',');
    }
  }
  //delay(50);
  tEnd = micros();
  Serial.print(" execution time (micros)- ");
  Serial.println(tEnd - tStart);
}
