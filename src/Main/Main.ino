// Variables
int motorPowers[4] = { 0, 0, 0, 0 };

const int esc_OutputPins[4] = { 9, 8, 7, 6 };
const int channel3_InputPin = 12;
const int channel2_InputPin = 11;
const int channel1_InputPin = 10; 
const int 6dof_InputPins[2] = { 5, 4 };


// Saskatone's attributes
int throttle = 0; // Z - channel 3
int pitch = 0; // Y // channel 2
int roll = 0; // X // channel 1

void setup() {
	pinMode(esc_OutputPins[0], OUTPUT);
	pinMode(esc_OutputPins[1], OUTPUT);
	pinMode(esc_OutputPins[2], OUTPUT);
	pinMode(esc_OutputPins[3], OUTPUT);

	pinMode(channel3_InputPin, INPUT);
	pinMode(channel2_InputPin, INPUT);
	pinMode(channel1_InputPin, INPUT);

	pinMode(6dof_InputPins[0], INPUT);
	pinMode(6dof_InputPins[1], INPUT);
	Serial.begin(9600);
}

void loop() {

	listenForController();
	
}

// Will listen for signals from the transmitter
// to control the drone
void listenForController(){
	throttle = map(pulseIn(channel3_InputPin, HIGH), 1005, 1980, 0, 179);
	pitch = map(pulseIn(channel2_InputPin, HIGH), 1005, 1980, 0, 179);
	roll = map(pulseIn(channel1_InputPin, HIGH), 1005, 1980, 0, 179);

	Serial.println(throttle);



	// ------
	Serial.println("---------------------");	

	delay(1000);
}
