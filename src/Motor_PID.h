//author:: Natan :Lisowski
// this library let you control motor with pwm signal
//this works with HG7881 Hbridge  L9110S
//link to repository
//
// library inspired by https://github.com/curiores/ArduinoTutorials
//  encoder motor control
#pragma once
#ifndef Motor_PID_h
#define Motor_PID_h
#include <Arduino.h>
#include <math.h>
//#include <util/atomic.h> // For the ATOMIC_BLOCK macro

class motor

{
public:
	float kp, kd, ki;																											  //your pid variables
	bool motor_state;																											  //this variable let the pwm signal go
	motor(uint8_t ENCA, uint8_t ENCB, uint8_t IN1, uint8_t IN2, uint8_t pwmpin = 0, int lower_limit = 50, int upper_limit = 255); // constructor
	~motor(){};
	void start();							 // place this in loop without delays
	void init(float kp, float ki, float kd); // this function initializes pid regulator
	void turn_on();							 // changes motor_state variable to true
	void turn_off();						 // changes motor_state variable to false
	long posi;								 // position of rotary encoder \ number of pulses
	void set_position(float posi = 0);
	long get_position();
	void set_target(float);
	long get_target();
	void limit(int, int);
	bool target_reached(bool reset = false); // check if target position is reached by motor

private:
	uint8_t ENCA, ENCB, IN2, IN1;
	void setMotor(int dir, int pwmVal);
	void readEncoder();
	void RisingInterrupt(); // function for detecting rising edge
	bool buffer;
	uint8_t _pwmpin = 0;
	int _upper_limit = 0, _lower_limit = 0;
	long prevT;
	long target;
	float eprev, eintegral;
	bool target_is_reached = false;
};
#endif