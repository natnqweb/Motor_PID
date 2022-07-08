// author:: Natan :Lisowski
//  this library let you control motor with pwm signal
// this works with HG7881 Hbridge  L9110S
// link to repository
//
//  library inspired by https://github.com/curiores/ArduinoTutorials
//   encoder motor control
#pragma once
#include <Arduino.h>
#include <math.h>

class motor
{
public:
	float kp = 0.0f, kd = 0.0f, ki = 0.0f;																						  // your pid variables
	bool motor_state{false};																									  // this variable let the pwm signal go
	motor(uint8_t ENCA, uint8_t ENCB, uint8_t IN1, uint8_t IN2, uint8_t pwmpin = 0, int lower_limit = 50, int upper_limit = 255); // constructor
	~motor(){};
	void start();							 // place this in loop without delays
	void init(float kp, float ki, float kd); // this function initializes pid regulator
	void turn_on();							 // changes motor_state variable to true
	void turn_off();						 // changes motor_state variable to false
	long posi{0};							 // position of rotary encoder \ number of pulses
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
	bool buffer{false};
	uint8_t _pwmpin = 0;
	int _upper_limit = 0, _lower_limit = 0;
	long prevT{0};
	long target{0};
	float eprev = 0, eintegral = 0;
	bool target_is_reached = false;
};
