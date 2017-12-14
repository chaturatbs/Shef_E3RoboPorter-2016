
char letter;


void setup() {

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0) {

    letter = Serial.read();
    
	if( letter == '#') {
		
		letter = Serial.read();
		
		if ( letter == 'F' ){
		Serial.print("Go forward ");
		Serial.println(Serial.parseInt());
		} else if ( letter == 'B' ) {
		Serial.print("Go backward ");
		Serial.println(Serial.parseInt());      
		} else if ( letter == 'R' ) {
		Serial.print("Turn right ");
		Serial.println(Serial.parseInt());      
		} else if ( letter == 'L' ) {
		Serial.print("Turn left ");
		Serial.println(Serial.parseInt());    
		} else if ( letter == 'S' ) {
		Serial.print("Set speed to ");
		Serial.println(Serial.parseInt());    
		} else if ( letter == 'X' ) {
		Serial.println("STOP!!!!!");
		} else {
		Serial.println("UWOTM8?");
		}
	}
    
  }

}
