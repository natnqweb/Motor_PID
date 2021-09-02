//author:: Natan :Lisowski
// this library let you control motor with pwm signal 
//this works with HG7881 Hbridge  L9110S
//link to repository
//https://github.com/natnqweb/Motor_PID
// library inspired by curiores
// curiores encoder control
#ifndef Motor_PID_h
#define Motor_PID_h
#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

class motor

{
public:
	float kp, kd, ki;//your pid variables
	bool motor_on; //this variable let the pwm signal go
	motor(uint8_t ENCA, uint8_t ENCB, uint8_t IN1, uint8_t IN2);// constructor 
	~motor(){};
	void start(int target);// place this in loop without delays
	void init(float kp, float kd, float ki);// this function initializes pid regulator
	void turn_on(); // changes motor_on variable to true
	void turn_off();// changes motor_on variable to false
	volatile int posi; // position of rotary encoder \ number of pulses

private:
	uint8_t ENCA, ENCB, IN2, IN1;
	void setMotor(int dir, int pwmVal, int in1, int in2);
	void readEncoder();
	void RisingInterrupt();// function for detecting rising edge 
	bool buffer;

	long prevT;
	int target;
	float eprev, eintegral;
};
#endif
