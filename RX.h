#ifndef RX_h
#define RX_h

#include "Arduino.h"
#include "lib/SBUS/SBUS.h"
#include "lib/DSMRX/DSMRX.h" 

//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup
//Radio:
//Note: If using SBUS, connect to pin 21 (RX5), if using DSM, connect to pin 15 (RX3)
static const uint8_t CH1_PIN = 15; //throttle
static const uint8_t CH2_PIN = 16; //ail
static const uint8_t CH3_PIN = 17; //ele
static const uint8_t CH4_PIN = 20; //rudd
static const uint8_t CH5_PIN = 21; //gear (throttle cut)
static const uint8_t CH6_PIN = 22; //aux1 (free aux channel)
static const uint8_t PPM_PIN = 23;
static const uint8_t NUM_DSM_CHANNELS = 6; //If using DSM RX, change this to match the number of transmitter channels you have


// C++ functions

class RX {
public:
    RX(){}
    ~RX(){}
    virtual void getCommands(unsigned long channels[])=0;
};


class RX_PPM : public RX {
public:
    RX_PPM();
    ~RX_PPM();
    void getCommands(unsigned long channels[]);
};
    

class RX_PWM : public RX {
private:
    RX_PPM *rx_ppm;

public:
    RX_PWM();
    ~RX_PWM();
    void getCommands(unsigned long channels[]);
};
    

class RX_SBUS : public RX {
private:
    uint16_t sbusChannels[16];
    bool sbusFailSafe;
    bool sbusLostFrame;
    SBUS *sb;
      
public:
    RX_SBUS(HardwareSerial& serial);
    ~RX_SBUS();
    void getCommands(unsigned long channels[]);
};


class RX_DSM : public RX {
public:
    HardwareSerial* serial;
    DSM1024 DSM;

    RX_DSM(HardwareSerial& bus);
    ~RX_DSM();
    void getCommands(unsigned long channels[]);

private:

    void serialEvent3(void);
};


// C functions

void getPPM();

void getCh1();

void getCh2();

void getCh3();

void getCh4();

void getCh5();

void getCh6();

unsigned long getRadioPWM(int ch_num);


#endif
