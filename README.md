# Motor_PID HG7881_L9110
# Natan Lisowski 

// this library let you control motor with pwm signal  

this works with HG7881 module double H bridge  L9110S


library contain class motor

use of this class


#define ENCA 2 // YELLOW from polulu


#define ENCB 3 // WHITE from polulu

#define IN2 11 //B1-A  input of a motor driver


#define IN1 10 //A1-A  input of a motor driver
//when  your dc motor is not working properly try to sawp pins ENCA WITH ENCB or IN2 with IN1

motor motor1(ENCA, ENCB, IN1, IN2);










Copyright (c) 2021 Natan Lisowski

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

### EXAMPLE
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


### L298N driver example
    /********************************************************
     * PID l298n

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

