#include "Motor_PID.h"
motor::motor(uint8_t ENCA, uint8_t ENCB, uint8_t IN1, uint8_t IN2)
{
  this->ENCA = ENCA;
  this->ENCB = ENCB;
  this->IN1 = IN1;
  this->IN2 = IN2;
}
void motor::init(float kp, float kd, float ki)
{
  this->kp = kp;
  this->kd = kd;
  this->ki = ki;
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motor_on = 1;
}
void motor::RisingInterrupt()
{
  bool reading = digitalRead(ENCA);
  if (reading != buffer)
  {
    buffer = reading;
    if (reading)
    {
      readEncoder();
    }
  }
}
void motor::setMotor(int dir, int pwmVal, int in1, int in2)
{
  pwmVal = constrain(pwmVal, 50, 255);
  if (dir == 1 && motor_on == 1)
  {
    analogWrite(in1, pwmVal);
    analogWrite(in2, 0);
  }
  else if (dir == -1 && motor_on == 1)
  {
    analogWrite(in1, 0);
    analogWrite(in2, pwmVal);
  }
  else
  {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}
void motor::start(int target)
{

  this->target = target;
  long currT = micros();
  RisingInterrupt();
  // time difference

  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    pos = posi;
  }

  // error
  int e = pos - target;

  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // motor power
  float pwr = fabs(u);
  if (pwr > 255)
  {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0)
  {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, IN1, IN2);

  // store previous error
  eprev = e;
}
void motor::readEncoder()
{
  int b = digitalRead(ENCB);
  if (b > 0)
  {
    posi++;
  }
  else
  {
    posi--;
  }
}
void motor::turn_on()
{
  motor_on = 1;
}
void motor::turn_off()
{
  motor_on = 0;
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
}