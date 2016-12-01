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

float dispScale = 1;
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
  }
}

void setup() {
  int size_x = (int)(400*dispScale);
  int size_y = (int)(400*dispScale);
  size(400, 400);
  //int[] porter_ctr = new int[2]
  porter_ctr[0] = size_x/2;
  porter_ctr[1] = size_y/2;

  sense_c[0] = porter_ctr[0];
  sense_c[1] = (int)(porter_ctr[1]-36*dispScale);

  sense_b[0] = porter_ctr[0];
  sense_b[1] = (int)(porter_ctr[1]+36*dispScale);

  sense_fl[0] =  (int)(sense_c[0]-36*dispScale);
  sense_fl[1] =  sense_c[1];

  sense_fr[0] = (int)(sense_c[0]+36*dispScale);
  sense_fr[1] =  sense_c[1];

  sense_l[0] = sense_fl[0];
  sense_l[1] = porter_ctr[1];

  sense_r[0] = sense_fr[0];
  sense_r[1] = porter_ctr[1];

  printArray(Serial.list());
  try{
    usPort = new Serial(this, Serial.list()[0], 19200);
  } catch (IndexOutOfBoundsException e) {
    System.err.println("IndexOutOfBoundsException: " + e.getMessage());
  }
    

  int j = 0;

  try{ //<>//
    while ((usPort.available() > 0) && (j < nAvg)) {
      inBuffer = usPort.readStringUntil('\n');
      if (inBuffer != null) {
        inBuffer = inBuffer.trim();
  
        String[] tokens = inBuffer.split(delims);
        for (int i = 0; i < tokens.length; i++) { //<>//
          distances[i] = Integer.parseInt(tokens[i]);
        }
      }
      mAverage(nAvg); //<>//
      j++;
    }
  } catch (NullPointerException e){
    System.err.println("NullPointerException: " + e.getMessage());
  }
}

void draw() {
  background(240);
  fill(120); //<>//
  rect(porter_ctr[0]-36*dispScale, porter_ctr[1]-36*dispScale, 71*dispScale, 71*dispScale);
  strokeWeight(3);
  fill(230);
  line(porter_ctr[0]-45*dispScale, porter_ctr[1]+20*dispScale,porter_ctr[0]+45*dispScale, porter_ctr[1]+20*dispScale);
  strokeWeight(1);
  try { //<>//
    while (usPort.available() > 0) {
  
      inBuffer = usPort.readStringUntil('\n');
      if (inBuffer != null) {
        inBuffer = inBuffer.trim();
        String[] tokens = inBuffer.split(delims);
        for (int i = 0; i < tokens.length; i++) { //<>//
           distances[i] = Integer.parseInt(tokens[i]);
         }
      }
    }
  } catch (NullPointerException e){
    System.err.println("NullPointerException: " + e.getMessage());
  }

  mAverage(nAvg);

  fill(200);
  arc(sense_c[0], sense_c[1], AvgDistances[0]*dispScale,  AvgDistances[0]*dispScale, -sensorAngle-HALF_PI, sensorAngle-HALF_PI, PIE);
  arc(sense_b[0], sense_b[1],  AvgDistances[5]*dispScale,  AvgDistances[5]*dispScale, -sensorAngle+HALF_PI, sensorAngle+HALF_PI, PIE);
  arc(sense_fl[0], sense_fl[1],  AvgDistances[1]*dispScale,  AvgDistances[1]*dispScale, -sensorAngle-HALF_PI, sensorAngle-HALF_PI, PIE);
  arc(sense_fr[0], sense_fr[1],  AvgDistances[2]*dispScale,  AvgDistances[2]*dispScale, -sensorAngle-HALF_PI, sensorAngle-HALF_PI, PIE);
  arc(sense_l[0], sense_l[1],  AvgDistances[3]*dispScale,  AvgDistances[3]*dispScale, -sensorAngle-PI, sensorAngle-PI, PIE);
  arc(sense_r[0], sense_r[1],  AvgDistances[4]*dispScale,  AvgDistances[4]*dispScale, -sensorAngle, sensorAngle, PIE);

  textSize(12);
  fill(0, 102, 153);
  text((int)AvgDistances[0], sense_c[0], sense_c[1]);
  text((int)AvgDistances[1], sense_fl[0], sense_fl[1]);
  text((int)AvgDistances[2], sense_fr[0], sense_fr[1]);
  text((int)AvgDistances[3], sense_l[0], sense_l[1]);
  text((int)AvgDistances[4], sense_r[0], sense_r[1]);
  text((int)AvgDistances[5], sense_b[0], sense_b[1]);

}