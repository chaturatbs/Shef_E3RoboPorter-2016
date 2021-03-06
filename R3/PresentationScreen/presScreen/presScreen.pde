import processing.serial.*;
import org.gicentre.utils.stat.*;    // For chart classes.
import grafica.*;
import java.util.Random;

public GPlot positionPlot;
public GPlot targetPlot;
GPointsArray posPoints = new GPointsArray(10);
GPointsArray tgtPoints = new GPointsArray(10);

Serial usPort;
Serial mPort;

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

float[] lastPos = new float[2];

BarChart barChart;

float[] speeds = new float[2];
float[] AvgSpeeds = new float[2];

float [] leftSpeeds = new float[40];
float [] rightSpeeds = new float[40];
float [] timeSeries = new float[40];

float [] demandedSpeedL = new float[40];
float [] demandedSpeedR = new float[40];

int [] targetLocation = new int[2];
int [] porterLocation_Global = new int[2];
int [] porterLocation_Local = new int[2];
float porterOrientation = 0;
float targetOrientation = 0;
float porterYaw = 0;

XYChart leftWheel;
XYChart rightWheel;


void dataShiftBack(float left, float right){
  for (int i = 0; i < leftSpeeds.length-1; i++) {
    leftSpeeds[i] = leftSpeeds[i+1];
    rightSpeeds[i] = rightSpeeds[i+1];
    timeSeries[i] = timeSeries[i+1];
  }
  leftSpeeds[leftSpeeds.length-1] = left;
  rightSpeeds[rightSpeeds.length-1] = right;
  timeSeries[timeSeries.length-1] = timeSeries[timeSeries.length-2] + 1;
}

void setup() {
  size(100, 100);

  printArray(Serial.list());

  try{
    usPort = new Serial(this, "COM8", 19200);
  } catch (IndexOutOfBoundsException e) {
    System.err.println("IndexOutOfBoundsException: " + e.getMessage());
  }

  String[] args = {"TwoFrameTest"};
  USapplet US_sensors = new USapplet();
  SpeedApplet speedDisplay = new SpeedApplet();
  OrientationApplet orientationDisplay = new OrientationApplet();
  PositionApplet positionDisplay = new PositionApplet();
  PApplet.runSketch(args, US_sensors);
  PApplet.runSketch(args, speedDisplay);
  PApplet.runSketch(args, orientationDisplay);
  PApplet.runSketch(args, positionDisplay);
}

void draw() {
  background(0);

  try {
    while (usPort.available() > 0) {
      inBuffer = usPort.readStringUntil('\n');
      if (inBuffer != null) {
        inBuffer = inBuffer.trim();
        println(inBuffer);
        String[] tokens = inBuffer.split(delims);
        //println(tokens.length);
        if (tokens.length == 23){
          for (int i = 0; i < 6; i++) {
             distances[i] = int(Float.parseFloat(tokens[i+1]));
          }

          speeds[0] = Float.parseFloat(tokens[9]);
          speeds[1] = Float.parseFloat(tokens[10]);

          dataShiftBack(speeds[1], speeds[0]);
          leftWheel.setData(timeSeries, leftSpeeds);
          leftWheel.setMinX(min(timeSeries));
          rightWheel.setMinX(min(timeSeries));
          leftWheel.setMaxX(max(timeSeries));
          rightWheel.setMaxX(max(timeSeries));
          rightWheel.setData(timeSeries, rightSpeeds);

          porterOrientation = degrees(Float.parseFloat(tokens[18]));

          targetLocation[0] = Integer.parseInt(tokens[11]);
          targetLocation[1] = Integer.parseInt(tokens[12]);
          porterLocation_Global[0] = Integer.parseInt(tokens[13]);
          porterLocation_Global[1] = Integer.parseInt(tokens[14]);
        }
      }
    }
  } catch (NullPointerException e){
    System.err.println("NullPointerException: " + e.getMessage());
    speeds[0] = 0;
    speeds[1] = 0;
  }
}

public class USapplet extends PApplet {

  void mAverage(int n) {
    int i = 0;
    for (i = 0; i < distances.length; i++) {
      AvgDistances[i] = AvgDistances[i] + (distances[i] - AvgDistances[i])/n;
    }
  }

  public void settings() {
    size(400, 400);
  }
  
  public void setup(){
    surface.setLocation(410, 310);
    
    int size_x = (int)(400*dispScale);
    int size_y = (int)(400*dispScale);
    
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
  }

  public void draw() {
    background(240);
    fill(120);
    rect(porter_ctr[0]-36*dispScale, porter_ctr[1]-36*dispScale, 71*dispScale, 71*dispScale);
    strokeWeight(3);
    fill(230);
    line(porter_ctr[0]-45*dispScale, porter_ctr[1]+20*dispScale,porter_ctr[0]+45*dispScale, porter_ctr[1]+20*dispScale);
    strokeWeight(1);

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
}

public class SpeedApplet extends PApplet {
  int tCount = 0;

  public void settings(){
    size(600,280);
  }
  public void setup() {
    
    surface.setLocation(0, 0);
    
    leftWheel = new XYChart(this);
    rightWheel = new XYChart(this);

    rightWheel.setData(timeSeries, rightSpeeds);
    leftWheel.setData(timeSeries, leftSpeeds);

    // Axis formatting and labels.
    leftWheel.showXAxis(true);
    leftWheel.showYAxis(true);

    leftWheel.setMinY(0);
    rightWheel.setMinY(0);

    leftWheel.setMaxY(100);
    rightWheel.setMaxY(100);

    leftWheel.setYFormat("###");  // M
    leftWheel.setXFormat("0000");      // Time

    rightWheel.setYFormat("###");  //
    rightWheel.setXFormat("0000");      // Time

    // Symbol colours
    leftWheel.setPointColour(color(180,50,50,100));
    leftWheel.setPointSize(5);
    leftWheel.setLineWidth(1);

    rightWheel.setPointColour(color(50,50,180,100));
    rightWheel.setPointSize(5);
    rightWheel.setLineWidth(1);
  }

  public void draw() {
    background(255);
    textSize(12);
    try {
      leftWheel.draw(15,15,width-30,height-30);
      rightWheel.draw(30,5,width-60,height-30);
    } catch (NullPointerException e){
    }

    //Draw a title over the top of the chart.
    fill(120);
    textSize(20);
    text("Motor Speeds", 70,30);
    textSize(11);
    text("(in RPM)",
          70,45);
  }
}

public class OrientationApplet extends PApplet {

  private float x = 0;
  private float y = 0;
    
  public void settings() {
    size(400, 400);
  }
  public void setup(){
    surface.setLocation(0, 310);
  }
  public void draw() {
    background(240);
    fill(150);
    stroke(255,255,255);
    ellipse(200, 200, 250, 250);
    strokeWeight(4);
    //draw north
    line(200, 200, 200, 50);

    //draw target

    int dX = targetLocation[0] - porterLocation_Global[0];
    int dY = targetLocation[1] - porterLocation_Global[1];

    if (dX != 0)
        targetOrientation = atan(dY / dX);
    else
        targetOrientation = HALF_PI;



    x = 130 * cos(targetOrientation+HALF_PI);
    y = 130 * sin(targetOrientation+HALF_PI);

    stroke(50,200,50);
    strokeWeight(3);
    line(200, 200, 200-x, 200-y);

    x = 0;
    y = 0;

    x = 130 * cos(radians(porterOrientation+90));
    y = 130 * sin(radians(porterOrientation+90));

    //draw porter orientation
    stroke(50,50,200);
    strokeWeight(3);
    line(200, 200, 200-x, 200-y);
  }
}

public class PositionApplet extends PApplet {

  public void settings() {
    size(500, 400);
  }

  public void setup(){
    positionPlot = new GPlot(this);
    targetPlot = new GPlot(this);
    
    surface.setLocation(820, 310);

    positionPlot.setDim(new float[] {400, 300});
    positionPlot.getXAxis().getAxisLabel().setText("X");
    positionPlot.getYAxis().getAxisLabel().setText("Y");
    positionPlot.getTitle().setText("Porter Location");
    positionPlot.setPoints(posPoints);
    //positionPlot.addLayer("pos",  posPoints);
    positionPlot.setLineColor(color(0, 0, 0));
    positionPlot.addLayer("tgt",  tgtPoints);
    positionPlot.getLayer("tgt").setPointColor(color(100, 255, 100));
    positionPlot.getLayer("tgt").setLineColor(color(50, 50, 50));
    //positionPlot.setPointColor(color(10, 10, 255));
    

//    targetPlot.setDim(new float[] {400, 400});
//    targetPlot.setPointColor(color(100, 255, 100));
//    targetPlot.setLineColor(color(50, 50, 50));
//    targetPlot.setPoints(tgtPoints);

    // Setup the mouse actions
    positionPlot.activateZooming();
    positionPlot.activatePanning();
  }


  public void draw() {
    background(255);

    // Add a new point to the second plot if the mouse moves significantly
    GPointsArray posPoints = positionPlot.getPoints();
    GPointsArray tgtPoints = positionPlot.getLayer("tgt").getPoints();

    //println(targetLocation[0], targetLocation[1]);
    if(posPoints.getNPoints() == 0){
      posPoints.add(porterLocation_Global[0], porterLocation_Global[1], "(" + str(porterLocation_Global[0]) + " , " + str(porterLocation_Global[1]) + ")");
      positionPlot.setPoints(posPoints);
    } else {
      GPoint lastPoint = posPoints.getLastPoint();
      if(sq(lastPoint.getX() - porterLocation_Global[0]) + sq(lastPoint.getY() - porterLocation_Global[1]) > 2500){
        posPoints.add(porterLocation_Global[0], porterLocation_Global[1], "(" + str(porterLocation_Global[0]) + " , " + str(porterLocation_Global[1]) + ")");
        positionPlot.setPoints(posPoints);
      }
    }

   if(tgtPoints.getNPoints() == 0){
      tgtPoints.add(targetLocation[0], targetLocation[1], "(" + str(targetLocation[0]) + " , " + str(targetLocation[1]) + ")");
      positionPlot.getLayer("tgt").setPoints(tgtPoints);
    } else {
      GPoint lastPoint = tgtPoints.getLastPoint();
      if(sq(lastPoint.getX() - targetLocation[0]) + sq(lastPoint.getY() - targetLocation[1]) > 2500){
        tgtPoints.add(targetLocation[0], targetLocation[1], "(" + str(targetLocation[0]) + " , " + str(targetLocation[1]) + ")");
        positionPlot.getLayer("tgt").setPoints(tgtPoints);
      }
    }

    // Reset the points if the user pressed the space bar
    if(keyPressed && key == ' '){
      positionPlot.setPoints(new GPointsArray(100));
    }

    // Draw the second plot
    positionPlot.beginDraw();
    positionPlot.drawBackground();
    positionPlot.drawBox();
    positionPlot.drawXAxis();
    positionPlot.drawYAxis();
    positionPlot.drawTitle();
    positionPlot.drawGridLines(GPlot.BOTH);
    positionPlot.drawLines();
    positionPlot.drawPoints();
    positionPlot.endDraw();

    //targetPlot.beginDraw();
    ////targetPlot.drawBackground();
    //// positionPlot.drawBox();
    //// positionPlot.drawXAxis();
    //// positionPlot.drawYAxis();
    //// positionPlot.drawTitle();
    //// positionPlot.drawGridLines(GPlot.BOTH);
    //targetPlot.drawLines();
    //targetPlot.drawPoints();
    //targetPlot.endDraw();
  }
}