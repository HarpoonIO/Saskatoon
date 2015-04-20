// Variables
int Channel3 = 12;
int throttle = 0;

void setup() {
	pinMode(Channel3, INPUT);
	Serial.begin(9600);
}

void loop() {

	listenForController();
	
}

// Will listen for signals from the transmitter
// to control the drone
void listenForController(){
	throttle = map(pulseIn(Channel3, HIGH), 1005, 1980, 0, 179);

	Serial.println(throttle);



	// ------
	Serial.println("---------------------");	

	delay(1000);
}
