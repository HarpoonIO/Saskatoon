/**
 * Custom library for controlling 4 ESC's that 
 * will control brushless motors
 */

#include "Arduino.h"
#include "Servo.h"
#include "ESC_library.h"

ESC_library::ESC_library(int _pin1, int _pin2, int _pin3, int _pin4)
{
	pin1 = _pin1;
	pin2 = _pin2;
	pin3 = _pin3;
	pin4 = _pin4;
}

void ESC_library::calibrate()
{
	motor1.attach(pin1);
	motor2.attach(pin2);
	motor3.attach(pin3);
	motor4.attach(pin4);
	motor1.writeMicroseconds(MAX_SIGNAL);
	motor2.writeMicroseconds(MAX_SIGNAL);
	motor3.writeMicroseconds(MAX_SIGNAL);
	motor4.writeMicroseconds(MAX_SIGNAL);
	delay(3000);
	motor1.writeMicroseconds(MIN_SIGNAL);
	motor2.writeMicroseconds(MIN_SIGNAL);
	motor3.writeMicroseconds(MIN_SIGNAL);
	motor4.writeMicroseconds(MIN_SIGNAL);
}

void ESC_library::controlMotor1(int throttle)
{
	motor1.writeMicroseconds(throttle);
}

void ESC_library::controlMotor2(int throttle)
{
	motor2.writeMicroseconds(throttle);
}

void ESC_library::controlMotor3(int throttle)
{
	motor3.writeMicroseconds(throttle);
}

void ESC_library::controlMotor4(int throttle)
{
	motor4.writeMicroseconds(throttle);
}