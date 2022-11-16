#include "EX.h"

EX_Teensy::EX_Teensy() {
    //Initialize all pins
    pinMode(m1Pin, OUTPUT);
    pinMode(m2Pin, OUTPUT);
    pinMode(m3Pin, OUTPUT);
    pinMode(m4Pin, OUTPUT);
    pinMode(m5Pin, OUTPUT);
    pinMode(m6Pin, OUTPUT);

    servo1.attach(servo1Pin, 900, 2100); //Pin, min PWM value, max PWM value
    servo2.attach(servo2Pin, 900, 2100);
    servo3.attach(servo3Pin, 900, 2100);
    servo4.attach(servo4Pin, 900, 2100);
    servo5.attach(servo5Pin, 900, 2100);
    servo6.attach(servo6Pin, 900, 2100);
    servo7.attach(servo7Pin, 900, 2100);

    //Set built in LED to turn on to signal startup
    digitalWrite(13, HIGH);
}

void EX_Teensy::writeMotor1(uint8_t val) { digitalWrite(m1Pin, val); }
void EX_Teensy::writeMotor2(uint8_t val) { digitalWrite(m2Pin, val); }
void EX_Teensy::writeMotor3(uint8_t val) { digitalWrite(m3Pin, val); }
void EX_Teensy::writeMotor4(uint8_t val) { digitalWrite(m4Pin, val); }
void EX_Teensy::writeMotor5(uint8_t val) { digitalWrite(m5Pin, val); }
void EX_Teensy::writeMotor6(uint8_t val) { digitalWrite(m6Pin, val); }

void EX_Teensy::writeServo1(uint8_t val) { servo1.write(val); }
