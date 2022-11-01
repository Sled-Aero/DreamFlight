#ifndef MX_h
#define MX_h

#include "Arduino.h"
#include "SPI.h"

#ifdef MACOS
  #include "HardwareSerial.h"
  extern HardwareSerial Serial;
#else 
  #include "usb_serial.h"
#endif

#define USE_MPU6050_I2C
#define GYRO_250DPS
#define ACCEL_2G

class MX {
    //========================================================================================================================//
    //Setup gyro and accel full scale value selection and scale factor
    //========================================================================================================================//

    #if defined GYRO_250DPS
        #define GYRO_SCALE GYRO_FS_SEL_250
        #define GYRO_SCALE_FACTOR 131.0
    #elif defined GYRO_500DPS
        #define GYRO_SCALE GYRO_FS_SEL_500
        #define GYRO_SCALE_FACTOR 65.5
    #elif defined GYRO_1000DPS
        #define GYRO_SCALE GYRO_FS_SEL_1000
        #define GYRO_SCALE_FACTOR 32.8
    #elif defined GYRO_2000DPS
        #define GYRO_SCALE GYRO_FS_SEL_2000
        #define GYRO_SCALE_FACTOR 16.4
    #endif

    #if defined ACCEL_2G
        #define ACCEL_SCALE ACCEL_FS_SEL_2
        #define ACCEL_SCALE_FACTOR 16384.0
    #elif defined ACCEL_4G
        #define ACCEL_SCALE ACCEL_FS_SEL_4
        #define ACCEL_SCALE_FACTOR 8192.0
    #elif defined ACCEL_8G
        efine ACCEL_SCALE ACCEL_FS_SEL_8
        #define ACCEL_SCALE_FACTOR 4096.0
    #elif defined ACCEL_16G
        #define ACCEL_SCALE ACCEL_FS_SEL_16
        #define ACCEL_SCALE_FACTOR 2048.0
    #endif

public:
    MX(){}
    ~MX(){}
    virtual void getMotion(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz)=0;
};


class MX_6050 : public MX {
    #include "lib/MPU6050/MPU6050.h"

    #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
    #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
    #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
    #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
    #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
    #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
    #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
    #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16

public:
    MX_6050();
    ~MX_6050();
    void getMotion(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);

private:    
    MPU6050 mpu6050;
};

#ifdef USE_MPU9250_SPI 
class MX_9250 : public MX {
    #include "lib/MPU9250/MPU9250.h"

    #define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
    #define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
    #define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS                                                        
    #define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
    #define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
    #define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
    #define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
    #define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G

public:
    MX_9250();
    ~MX_9250();
    void getMotion(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);

private:
    MPU9250 mpu9250(SPI2, 36);
};
#endif // USE_MPU9250_SPI

#endif
