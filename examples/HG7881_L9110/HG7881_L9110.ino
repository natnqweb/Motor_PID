/********************************************************
 * PID Basic Example
 2
 ********************************************************/

#include <Motor_PID.h>
#define ENCA 33  // YELLOW from polulu
#define ENCB 34  // WHITE from polulu

#define IN2 11  //B1-A
#define IN1 10  //A1-A

 //long prevT;
int target = 100;
//int target = 50*sin(prevT/1e6);
//int target=300;
//int target=analogRead(A0);
// PID constants
float kp = 8;
float kd = 1;
float ki = 0.01;
//int target = 50*sin(prevT/1e6);
Motor motor1(ENCA, ENCB, IN1, IN2);
void setup() {
    motor1.init(kp, ki, kd);
    motor1.set_target(target);
}
void loop() {
    // long currT = micros();
    // float deltaT = ((float)(currT - prevT)) / (1.0e6);
    //prevT = currT;
    motor1.start();
}