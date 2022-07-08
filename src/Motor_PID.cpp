#include "Motor_PID.h"

motor::motor(uint8_t ENCA, uint8_t ENCB, uint8_t IN1, uint8_t IN2, uint8_t pwmpin, int lower_limit, int upper_limit)
{
  this->ENCA = ENCA;
  this->ENCB = ENCB;
  this->IN1 = IN1;
  this->IN2 = IN2;
  _pwmpin = pwmpin;
  _upper_limit = upper_limit;
  _lower_limit = lower_limit;
}
void motor::init(float kp, float ki, float kd)
{
  this->kp = kp;
  this->kd = kd;
  this->ki = ki;
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  if (_pwmpin != 0)
    pinMode(_pwmpin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motor_state = 1;
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
void motor::setMotor(int dir, int pwmVal)
{
  pwmVal = constrain(pwmVal, _lower_limit, _upper_limit);
  if (_pwmpin != 0)
    analogWrite(_pwmpin, pwmVal);
  if (dir == 1 && motor_state == 1)
  {
    (_pwmpin != 0) ? digitalWrite(IN1, 1) : analogWrite(IN1, pwmVal);
    (_pwmpin != 0) ? digitalWrite(IN2, 0) : analogWrite(IN2, 0);
  }
  else if (dir == -1 && motor_state == 1)
  {
    (_pwmpin != 0) ? digitalWrite(IN1, 0) : analogWrite(IN1, 0);
    (_pwmpin != 0) ? digitalWrite(IN2, 1) : analogWrite(IN2, pwmVal);
  }
  else
  {
    (_pwmpin != 0) ? digitalWrite(IN1, 0) : analogWrite(IN1, 0);
    (_pwmpin != 0) ? digitalWrite(IN1, 0) : analogWrite(IN2, 0);
  }
}
void motor::start()
{

  long currT = micros();
  RisingInterrupt();

  // time difference
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // error
  long e = posi - target;

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
  if (e == 0)
  {
    turn_off();

    target_is_reached = true;
  }
  else
  {
    turn_on();
    target_is_reached = false;
  }

  // signal the motor
  setMotor(dir, pwr);

  // store previous error
  eprev = e;
}
void motor::readEncoder()
{
  bool b = digitalRead(ENCB);
  if (b)
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
  motor_state = 1;
}
void motor::turn_off()
{

  motor_state = 0;
  if (_pwmpin != 0)
    analogWrite(_pwmpin, 0);
  (_pwmpin != 0) ? digitalWrite(IN1, 0) : analogWrite(IN1, 0);
  (_pwmpin != 0) ? digitalWrite(IN2, 0) : analogWrite(IN2, 0);
}

void motor::set_position(float posi)
{
  this->posi = (long)round(posi);
}
long motor::get_position()
{
  return posi;
}
void motor::set_target(float target)
{
  this->target = (long)round(target);
}
long motor::get_target()
{
  return target;
}
void motor::limit(int lower_limit, int upper_limit)
{
  _lower_limit = lower_limit;
  _upper_limit = upper_limit;
}
bool motor::target_reached(bool reset)
{
  if (reset)
    target_is_reached = false;
  return target_is_reached;
}
