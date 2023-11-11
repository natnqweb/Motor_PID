#include "Motor_PID.h"

Motor::Motor(uint8_t enca, uint8_t encb, uint8_t in1, uint8_t in2, uint8_t pwmpin, int lower_limit, int upper_limit)
{
  this->enca = enca;
  this->encb = encb;
  this->in1 = in1;
  this->in2 = in2;
  _pwmpin = pwmpin;
  _upper_limit = upper_limit;
  _lower_limit = lower_limit;
}

void Motor::init(double kp, double ki, double kd)
{
  this->kp = kp;
  this->kd = kd;
  this->ki = ki;
  pinMode(enca, INPUT);
  pinMode(encb, INPUT);
  if (_pwmpin != 0)
    pinMode(_pwmpin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  motor_state = 1;
}

void Motor::rising_interrupt()
{
  bool reading = digitalRead(enca);
  if (reading != buffer)
  {
    buffer = reading;
    if (reading)
    {
      read_encoder();
    }
  }
}

void Motor::set_motor(int dir, int pwm_val)
{
  pwm_val = constrain(pwm_val, _lower_limit, _upper_limit);
  if (_pwmpin != 0)
    analogWrite(_pwmpin, pwm_val);
  if (dir == 1 && motor_state == 1)
  {
    (_pwmpin != 0) ? digitalWrite(in1, 1) : analogWrite(in1, pwm_val);
    (_pwmpin != 0) ? digitalWrite(in2, 0) : analogWrite(in2, 0);
  }
  else if (dir == -1 && motor_state == 1)
  {
    (_pwmpin != 0) ? digitalWrite(in1, 0) : analogWrite(in1, 0);
    (_pwmpin != 0) ? digitalWrite(in2, 1) : analogWrite(in2, pwm_val);
  }
  else
  {
    (_pwmpin != 0) ? digitalWrite(in1, 0) : analogWrite(in1, 0);
    (_pwmpin != 0) ? digitalWrite(in2, 0) : analogWrite(in2, 0);
  }
}

void Motor::start()
{
  long curr_t = micros();
  rising_interrupt();

  double delta_t = ((double)(curr_t - prev_t)) / (1.0e6);
  prev_t = curr_t;

  long e = posi - target;

  double dedt = ((double)e - eprev) / (delta_t);

  eintegral = eintegral + e * delta_t;

  double u = kp * (double)e + kd * dedt + ki * eintegral;

  double pwr = fabs(u);
  if (pwr > 255)
  {
    pwr = 255;
  }

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

  set_motor(dir, pwr);

  eprev = e;
}

void Motor::read_encoder()
{
  bool b = digitalRead(encb);
  if (b)
  {
    posi++;
  }
  else
  {
    posi--;
  }
}

void Motor::turn_on()
{
  motor_state = 1;
}

void Motor::turn_off()
{
  motor_state = 0;
  if (_pwmpin != 0)
    analogWrite(_pwmpin, 0);
  (_pwmpin != 0) ? digitalWrite(in1, 0) : analogWrite(in1, 0);
  (_pwmpin != 0) ? digitalWrite(in2, 0) : analogWrite(in2, 0);
}

void Motor::set_position(double position)
{
  posi = (long)round(position);
}

long Motor::get_position()
{
  return posi;
}

void Motor::set_target(double target)
{
  this->target = (long)round(target);
}

long Motor::get_target()
{
  return target;
}

void Motor::limit(int lower_limit, int upper_limit)
{
  _lower_limit = lower_limit;
  _upper_limit = upper_limit;
}

bool Motor::target_reached(bool reset)
{
  if (reset)
    target_is_reached = false;
  return target_is_reached;
}
