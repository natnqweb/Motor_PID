/********************************************************
 * PID Basic Example
 
 ********************************************************/
 //when your dc motor is not working properly try to sawp pins ENCA WITH ENCB or IN2 with IN1
#include <Motor_PID.h>
#define ENCA 2 // YELLOW from polulu
#define ENCB 3 // WHITE from polulu

#define IN2 11 //B1-A 
#define IN1 10 //A1-A 
long prevT;
int target = 900;
//int target = 50*sin(prevT/1e6);
//int target=300;
//int target=analogRead(A0);
// PID constants
float kp = 1;
float kd = 0.1;
float ki = 0.02;
//int target = 50*sin(prevT/1e6);
motor motor1(ENCA, ENCB, IN1, IN2);
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
}
