import processing.serial.*;
Serial usPort;
String inBuffer = "";

void setup() {
  printArray(Serial.list());
    try{
      usPort = new Serial(this, "COM8", 19200);
    } catch (IndexOutOfBoundsException e) {
      System.err.println("IndexOutOfBoundsException: " + e.getMessage());
    }
}
void draw() {

  try {
    while (usPort.available() > 0) {
  
      inBuffer = usPort.readStringUntil('\n');
      if (inBuffer != null) {
        inBuffer = inBuffer.trim();
        println(inBuffer);
        //String[] tokens = inBuffer.split(delims);
        //for (int i = 0; i < tokens.length; i++) {
        //   distances[i] = Integer.parseInt(tokens[i]);
         }
      }
  } catch (NullPointerException e){
    System.err.println("NullPointerException: " + e.getMessage());
  }
}