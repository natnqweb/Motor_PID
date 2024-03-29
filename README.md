# Motor_PID HG7881_L9110

this library let you control motor with pwm signal  

library works with HG7881 module double H bridge  L9110S

# usage

### EXAMPLE HG7881 module double H bridge  L9110S

```C++
#include <Motor_PID.h>
#define ENCA 33  // YELLOW from polulu
#define ENCB 34  // WHITE from polulu

#define IN2 11  //B1-A
#define IN1 10  //A1-A

int target = 100;

// PID constants
float kp = 8;
float kd = 1;
float ki = 0.01;

Motor motor1(ENCA, ENCB, IN1, IN2);

void setup() {
    motor1.init(kp, ki, kd);
    motor1.set_target(target);
}
void loop() {
    motor1.start();
}
```

### L298N driver example

 ```C++
 // remember that l298n need 3 volt over DC motor nominal voltage to run full speed 
 //you can supply even more voltage but then limit it with pwm like i did here
#include <Motor_PID.h>
#define ENCA 2 // YELLOW from polulu encoder
#define ENCB 3 // WHITE from polulu encoder

#define IN2 10 //in2 from driver // pins to control direction
#define IN1 11 //in1 from driver // pins to control direction
#define PWM_PIN 5 //pwm to control speed

int target = 900;

// PID constants
float kp = 1;
float kd = 0.1;
float ki = 0.02;

int pwm_lower_limit = 0;//full range
int pwm_upper_limit = 255;//full range

Motor motor1(ENCA, ENCB, IN2, IN1, PWM_PIN, pwm_lower_limit, pwm_upper_limit);
void setup()
{
    motor1.init(kp, kd, ki);
    motor1.set_target(target);
}
void loop()
{
    motor1.start();
    if (motor1.get_position() == target) motor1.turn_off();
}

 ```
