#ifndef ESC_library_h
#define ESC_library_h

#include "Arduino.h"
#include "Servo.h"

class ESC_library
{
public:
	ESC_library(int _pin1, int _pin2, int _pin3, int _pin4);
	void calibrate();
	void controlMotor1(int throttle);
	void controlMotor2(int throttle);
	void controlMotor3(int throttle);
	void controlMotor4(int throttle);
private:
	int pin1;
	int pin2;
	int pin3;
	int pin4;
	Servo motor1;
	Servo motor2;
	Servo motor3;
	Servo motor4;
	const int MAX_SIGNAL = 2000;
	const int MIN_SIGNAL = 700;
};
#endif