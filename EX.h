#ifndef TX_h
#define TX_h

#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer 

class EX {

public:
    EX(){}
    ~EX(){}

    virtual void armServos() = 0;
    
    virtual void writeMotor1(uint8_t val) = 0;
    virtual void writeMotor2(uint8_t val) = 0;
    virtual void writeMotor3(uint8_t val) = 0;
    virtual void writeMotor4(uint8_t val) = 0;
    virtual void writeMotor5(uint8_t val) = 0;
    virtual void writeMotor6(uint8_t val) = 0;

    virtual void writeServo1(uint8_t val) = 0;
    virtual void writeServo2(uint8_t val) = 0;
    virtual void writeServo3(uint8_t val) = 0;
    virtual void writeServo4(uint8_t val) = 0;
    virtual void writeServo5(uint8_t val) = 0;
    virtual void writeServo6(uint8_t val) = 0;
    virtual void writeServo7(uint8_t val) = 0;
};


class EX_Teensy : public EX {   

public:
    EX_Teensy();
    ~EX_Teensy();

    void armServos();

    void writeMotor1(uint8_t val);
    void writeMotor2(uint8_t val);
    void writeMotor3(uint8_t val);
    void writeMotor4(uint8_t val);
    void writeMotor5(uint8_t val);
    void writeMotor6(uint8_t val);

    void writeServo1(uint8_t val);
    void writeServo2(uint8_t val);
    void writeServo3(uint8_t val);
    void writeServo4(uint8_t val);
    void writeServo5(uint8_t val);
    void writeServo6(uint8_t val);
    void writeServo7(uint8_t val);

private:
    //=============================================================================================//
    //  DECLARE PINS     
    //=============================================================================================//                                          

    //OneShot125 ESC pin outputs:
    const int m1Pin = 0;
    const int m2Pin = 1;
    const int m3Pin = 2;
    const int m4Pin = 3;
    const int m5Pin = 4;
    const int m6Pin = 5;

    //PWM servo or ESC outputs:
    const int servo1Pin = 6;
    const int servo2Pin = 7;
    const int servo3Pin = 8;
    const int servo4Pin = 9;
    const int servo5Pin = 10;
    const int servo6Pin = 11;
    const int servo7Pin = 12;

    PWMServo servo1;  //Create servo objects to control a servo or ESC with PWM
    PWMServo servo2;
    PWMServo servo3;
    PWMServo servo4;
    PWMServo servo5;
    PWMServo servo6;
    PWMServo servo7;
};

#endif
