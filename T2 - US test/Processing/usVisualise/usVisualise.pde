import processing.serial.*;
Serial usPort;

int[] porter_ctr = new int[2];
int[] sense_c = new int[2];
int[] sense_fl = new int[2];
int[] sense_fr = new int[2];
int[] sense_l = new int[2];
int[] sense_r = new int[2];
int[] sense_b = new int[2];
float sensorAngle = 0.3;

int[] distances = new int[6];
int inByte = 0;
int lf = 10;
String inBuffer = "";
String delims = "[,]+";
float[] AvgDistances = new float[6];
int nAvg = 15;

void mAverage(int n) {
  int i = 0;

  for (i = 0; i < distances.length; i++) {
    AvgDistances[i] = AvgDistances[i] + (distances[i] - AvgDistances[i])/n;
    //AvgDistances[i] = AvgDistances[i]/n;
  }

}

void setup() {
  int size_x = 800;
  int size_y = 800;
  size(800, 800);
  //int[] porter_ctr = new int[2]
  porter_ctr[0] = size_x/2;
  porter_ctr[1] = size_y/2;

  sense_c[0] = porter_ctr[0];
  sense_c[1] = porter_ctr[1]-72;

  sense_b[0] = porter_ctr[0];
  sense_b[1] = porter_ctr[1]+72;

  sense_fl[0] =  sense_c[0]-72;
  sense_fl[1] =  sense_c[1];

  sense_fr[0] = sense_c[0]+72;
  sense_fr[1] =  sense_c[1];

  sense_l[0] = sense_fl[0];
  sense_l[1] = porter_ctr[1];

  sense_r[0] = sense_fr[0];
  sense_r[1] = porter_ctr[1];

  printArray(Serial.list());
  usPort = new Serial(this, Serial.list()[0], 19200);

  int j = 0;

  while ((usPort.available() > 0) && (j < nAvg)) {
    inBuffer = usPort.readStringUntil('\n');
    if (inBuffer != null) {
      inBuffer = inBuffer.trim();

      String[] tokens = inBuffer.split(delims);
      for (int i = 0; i < tokens.length; i++) { //<>//
        distances[i] = Integer.parseInt(tokens[i]);
      }
    }
    mAverage(nAvg);
    j++;
  }
}

void draw() {
  background(240);
  fill(120); //<>//
  rect(porter_ctr[0]-71, porter_ctr[1]-71, 142, 142);
  //ellipse(porter_ctr[0], porter_ctr[1], 100, 100);
  strokeWeight(3);
  fill(230);
  line(porter_ctr[0]-90, porter_ctr[1]+40,porter_ctr[0]+90, porter_ctr[1]+40);
  strokeWeight(1);

  while (usPort.available() > 0) {

    inBuffer = usPort.readStringUntil('\n');
    if (inBuffer != null) {
      inBuffer = inBuffer.trim();
      //print(inBuffer);

      String[] tokens = inBuffer.split(delims);
      //print(tokens);
      for (int i = 0; i < tokens.length; i++) { //<>//
         distances[i] = Integer.parseInt(tokens[i]);
         //print(distances);
       }
      //println(str(distances));
      //inByte = Integer.parseInt(inBuffer.trim());
       //print(inByte);
    } //
  }

  mAverage(nAvg);

  fill(200);
  arc(sense_c[0], sense_c[1], AvgDistances[0]*2,  AvgDistances[0]*2, -sensorAngle-HALF_PI, sensorAngle-HALF_PI, PIE);
  arc(sense_b[0], sense_b[1],  AvgDistances[5]*2,  AvgDistances[5]*2, -sensorAngle+HALF_PI, sensorAngle+HALF_PI, PIE);
  arc(sense_fl[0], sense_fl[1],  AvgDistances[1]*2,  AvgDistances[1]*2, -sensorAngle-HALF_PI, sensorAngle-HALF_PI, PIE);
  arc(sense_fr[0], sense_fr[1],  AvgDistances[2]*2,  AvgDistances[2]*2, -sensorAngle-HALF_PI, sensorAngle-HALF_PI, PIE);
  arc(sense_l[0], sense_l[1],  AvgDistances[3]*2,  AvgDistances[3]*2, -sensorAngle-PI, sensorAngle-PI, PIE);
  arc(sense_r[0], sense_r[1],  AvgDistances[4]*2,  AvgDistances[4]*2, -sensorAngle, sensorAngle, PIE);


  //noFill();
  //arc(sense_c[0], sense_c[1], distances[0]*2, distances[0]*2, -sensorAngle-HALF_PI, sensorAngle-HALF_PI, PIE);
  //arc(sense_b[0], sense_b[1], distances[5]*2, distances[5]*2, -sensorAngle+HALF_PI, sensorAngle+HALF_PI, PIE);
  //arc(sense_fl[0], sense_fl[1], distances[1]*2, distances[1]*2, -sensorAngle-HALF_PI, sensorAngle-HALF_PI, PIE);
  //arc(sense_fr[0], sense_fr[1], distances[2]*2, distances[2]*2, -sensorAngle-HALF_PI, sensorAngle-HALF_PI, PIE);
  //arc(sense_l[0], sense_l[1], distances[3]*2, distances[3]*2, -sensorAngle-PI, sensorAngle-PI, PIE);
  //arc(sense_r[0], sense_r[1], distances[4]*2, distances[4]*2, -sensorAngle, sensorAngle, PIE);

  textSize(16);
  fill(0, 102, 153);
  text(AvgDistances[0], sense_c[0], sense_c[1]);
  text(AvgDistances[1], sense_fl[0], sense_fl[1]);
  text(AvgDistances[2], sense_fr[0], sense_fr[1]);
  text(AvgDistances[3], sense_l[0], sense_l[1]);
  text(AvgDistances[4], sense_r[0], sense_r[1]);
  text(AvgDistances[5], sense_b[0], sense_b[1]);

  //textSize(16);
  //fill(0, 102, 153);
  //text(distances[0], sense_c[0], sense_c[1]);
  //text(distances[1], sense_fl[0], sense_fl[1]);
  //text(distances[2], sense_fr[0], sense_fr[1]);
  //text(distances[3], sense_l[0], sense_l[1]);
  //text(distances[4], sense_r[0], sense_r[1]);
  //text(distances[5], sense_b[0], sense_b[1]);

}