# Motor_PID
//author:: Natan Lisowski // this library let you control motor with pwm signal  

this works with HG7881 module double H bridge  L9110S


library contain class motor

use of this class


#define ENCA 2 // YELLOW from polulu


#define ENCB 3 // WHITE from polulu

#define IN2 11 //B1-A  input of a motor driver


#define IN1 10 //A1-A  input of a motor driver


motor motor1(ENCA, ENCB, IN1, IN2);

