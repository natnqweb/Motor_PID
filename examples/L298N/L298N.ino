/********************************************************
 * PID Basic Example
 
 ********************************************************/
// remember that l298n need 3 volt over DC motor nominal voltage to run full speed 
//you can supply even more voltage but then limit it with pwm like i did here

#include <Motor_PID.h>
#define ENCA 2 // YELLOW from polulu encoder
#define ENCB 3 // WHITE from polulu encoder

#define IN2 10 //in2 from driver // pins to control direction
#define IN1 11 //in1 from driver // pins to control direction
#define pwmpin 5 //pwm to control speed
long prevT;
int target = 900;
//int target = 50*sin(prevT/1e6);
//int target=300;
//int target=analogRead(A0);
// PID constants
float kp = 1;
float kd = 0.1;
float ki = 0.02;
//int pwm_lower_limit=20;//to balance voltage if you are using too powerful power source  // limited
//int pwm_upper_limit=100;//to balance voltage if you are using too powerful power source  // limited
int pwm_lower_limit=0;//full range
int pwm_upper_limit=255;//full range

//int target = 50*sin(prevT/1e6);
motor motor1(ENCA, ENCB, IN2, IN1,pwmpin,pwm_lower_limit,pwm_upper_limit);
void setup()
{
    motor1.init(kp, kd, ki);
}
void loop()
{
    // long currT = micros();
    // float deltaT = ((float)(currT - prevT)) / (1.0e6);
    //prevT = currT;
    motor1.start(target);
    if(motor1.posi==target) motor1.turn_off();
   
    
}
