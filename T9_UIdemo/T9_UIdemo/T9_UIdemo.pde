import org.gicentre.utils.stat.*;    // For chart classes.
import processing.serial.*;
Serial mPort;

// Displays a simple line chart representing a time series.
XYChart leftWheel;
XYChart rightWheel;
BarChart barChart;
int[] speeds = new int[2];
String inBuffer = "";
String delims = "[,]+";
float[] AvgSpeeds = new float[2];
int nAvg = 15;

float [] leftSpeeds = new float[40];
float [] rightSpeeds = new float[40];
float [] timeSeries = new float[40];

int tCount = 0;
// Loads data into the chart and customises its appearance.
void setup()
{

  try {
    mPort = new Serial(this, Serial.list()[0], 19200);
  } catch (IndexOutOfBoundsException e) {
    System.err.println("IndexOutOfBoundsException: " + e.getMessage());
  }
  size(600,280);
  textFont(createFont("Arial",10),10);

  //barChart = new BarChart(this);
  //barChart.setData(new float[] {70,10});

  //// Scaling
  //barChart.setMinValue(0);
  //barChart.setMaxValue(30);

  // Axis appearance
  //textFont(createFont("Serif",10),10);

  //barChart.showValueAxis(true);
  ////barChart.setValueFormat("#%");
  //barChart.setBarLabels(new String[] {"Left","Right"});
  //barChart.showCategoryAxis(true);

  // Both x and y data set here.
  leftWheel = new XYChart(this);
  rightWheel = new XYChart(this);
  rightWheel.setData(timeSeries, rightSpeeds);
  leftWheel.setData(timeSeries, leftSpeeds);
  
  // Axis formatting and labels.
  leftWheel.showXAxis(true);
  leftWheel.showYAxis(true);
  
  //rightWheel.showXAxis(true);
  //rightWheel.showYAxis(true);
  
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
  leftWheel.setPointSize(9);
  leftWheel.setLineWidth(3);
   //<>//
  rightWheel.setPointColour(color(50,50,180,100));
  rightWheel.setPointSize(9);
  rightWheel.setLineWidth(3);
}
 //<>//
void dataShiftBack(int left, int right){ 
  for (int i = 0; i < leftSpeeds.length-1; i++) { //<>//
    leftSpeeds[i] = leftSpeeds[i+1];
    rightSpeeds[i] = rightSpeeds[i+1]; 
    timeSeries[i] = timeSeries[i+1]; 
  }
  leftSpeeds[leftSpeeds.length-1] = left; //<>//
  rightSpeeds[rightSpeeds.length-1] = right;
  timeSeries[timeSeries.length-1] = timeSeries[timeSeries.length-2] + 1;
}


void mouseClicked() {
  //if (tCount ==1) {
  //  dataShiftBack(10,5);
  //  //leftWheel.setData(new float[] {1900,1910, 1920, 1930, 1940, 1950,
  //  //                              1960, 1970, 1980, 1990, 2000, 2010},
  //  //                  new float[] {6322,  6489,  6401, 7657, 9649, 9767,
  //  //                              12167, 15154, 18200, 23124, 28645, 18500});
  // } else if (tCount ==2){
  //   dataShiftBack(13,18);
  //   //leftWheel.setData(new float[] {1900,1910,1920, 1930, 1940, 1950,
  //   //                             1960, 1970, 1980, 1990, 2000, 2010,2020},
  //   //                 new float[] {  6322,  6489,  6401, 7657, 9649, 9767,
  //   //                             12167, 15154, 18200, 23124, 28645, 18500,15050});
  // }  else if (tCount == 3){
  //   dataShiftBack(6,20);
  //   //leftWheel.setData(new float[] {1900,1910, 1920,1930, 1940, 1950, //<>//
  //   //                             1960, 1970, 1980, 1990, 2000, 2010,2020,2030},
  //   //                 new float[] {  6322,  6489,  6401, 7657, 9649, 9767,
  //   //                             12167, 15154, 18200, 23124, 28645, 18500,15050,19320});
  // }else if (tCount == 20){
  //   dataShiftBack(15,13);
  //   //leftWheel.setData(new float[] {1900,1910, 1920,1930, 1940, 1950,
  //   //                             1960, 1970, 1980, 1990, 2000, 2010,2020,2030,2040}, //<>//
  //   //                 new float[] {  6322,  6489,  6401, 7657, 9649, 9767,
  //   //                             12167, 15154, 18200, 23124, 28645, 18500,15050,19320,22000});
  // }else if (tCount == 25){
  //   dataShiftBack(15,13);
  //   //leftWheel.setData(new float[] {1900,1910, 1920,1930, 1940, 1950,
  //   //                             1960, 1970, 1980, 1990, 2000, 2010,2020,2030,2040},
  //   //                 new float[] {  6322,  6489,  6401, 7657, 9649, 9767,
  //   //                             12167, 15154, 18200, 23124, 28645, 18500,15050,19320,22000});
  // }

  //leftWheel.setData(timeSeries, leftSpeeds);
  tCount ++;
}

// Draws the chart and a title.
void draw(){
    try{
      while (mPort.available() > 0) { //<>//
        inBuffer = mPort.readStringUntil('\n');
        if (inBuffer != null){
          inBuffer = inBuffer.trim();
          String[] mTokens = inBuffer.split(delims);
          //print(mTokens);
          for (int i = 0; i < mTokens.length; i++) { //<>//
             speeds[i] = Integer.parseInt(mTokens[i]); //<>//
             speeds[i] = (int)(speeds[i] * (60000/100)/192) ; 
             //barChart.setData(new float[] {speeds[1],speeds[0]});
          }
          dataShiftBack(speeds[1], speeds[0]);
          leftWheel.setData(timeSeries, leftSpeeds);
          leftWheel.setMinX(min(timeSeries));
          rightWheel.setMinX(min(timeSeries));
          leftWheel.setMaxX(max(timeSeries));
          rightWheel.setMaxX(max(timeSeries));
          rightWheel.setData(timeSeries, rightSpeeds);
        }
      }
    } catch (NullPointerException e){
      System.err.println("NullPointerException: " + e.getMessage());
      speeds[0] = 0;
      speeds[1] = 0;
    }

  background(255);
  textSize(12);
  //barChart.draw(15,15,width-30,height-30);
  leftWheel.draw(15,15,width-30,height-30);
  rightWheel.draw(30,5,width-60,height-30);

  //Draw a title over the top of the chart.
  fill(120);
  textSize(20);
  text("Motor Speeds", 70,30);
  textSize(11);
  text("(in RPM)",
        70,45);
}