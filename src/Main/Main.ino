#include <Servo.h>

// Variables
int Channel3 = 12;
int throttle = 0;
Servo ESC1;
int esc1 = 9; 

void setup() {
	// Setup ESC's 
	ESC1.attach(esc1);
	// Recieve signals from transmitter
	pinMode(Channel3, INPUT);
	
	// Baudrate
	Serial.begin(9600);
}

void loop() {
	listenForController();
	controlThrotle();
}

// Will listen for signals from the transmitter
// to control the drone
void listenForController(){
	throttle = map(pulseIn(Channel3, HIGH), 1005, 1980, 0, 179);
	Serial.println(throttle);
	delay(500);
}

void controlThrotle(){
	if(throttle > -1 && throttle < 181){
		Serial.println("Writing to esc");
		ESC1.writeMicroseconds(throttle);    
	}else{
		ESC1.writeMicroseconds(0);
	}
}





