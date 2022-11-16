#ifndef FlightController_h
#define FlightController_h

//REQUIRED LIBRARIES (included with download in main sketch folder)

#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer

#ifdef MACOS
  #include "HardwareSerial.h"
  extern HardwareSerial Serial;
#else 
  #include "usb_serial.h"
#endif

#include "RX.h"
#include "MX.h"
#include "EX.h"


//Debug print controls
#define PRINT_RADIO_DATA     0x001
#define PRINT_DESIRED_STATE  0x002
#define PRINT_GYRO_DATA      0x004
#define PRINT_ACCEL_DATA     0x008
#define PRINT_MAG_DATA       0x010
#define PRINT_ROLL_PITCH_YAW 0x020
#define PRINT_PID_OUTPUT     0x040
#define PRINT_MOTOR_COMMANDS 0x080
#define PRINT_SERVO_COMMANDS 0x100
#define PRINT_LOOP_RATE      0x200

class FlightController {
private:
    //========================================================================================================================//
    //  USER-SPECIFIED VARIABLES                     
    //========================================================================================================================//

    //Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
    const float B_madgwick = 0.04;  //Madgwick filter parameter
    const float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    const float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
    const float B_mag = 1.0;        //Magnetometer LP filter parameter

    //Controller parameters (take note of defaults before modifying!): 
    float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
    float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
    float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
    float maxYaw = 160.0;     //Max yaw rate in deg/sec

    float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
    float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
    float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
    float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
    float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
    float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
    float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
    float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

    float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
    float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
    float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
    float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
    float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
    float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

    float Kp_yaw = 0.3;           //Yaw P-gain
    float Ki_yaw = 0.05;          //Yaw I-gain
    float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)


    //========================================================================================================================//
    //DECLARE GLOBAL VARIABLES

    //Radio communications
    unsigned long pwm_channels[6];
    unsigned long pwm_channels_prev[6];
    RX* rx;
    MX* mx;
    EX* ex;

    //IMU
    float AccX, AccY, AccZ;
    float AccX_prev, AccY_prev, AccZ_prev;
    float GyroX, GyroY, GyroZ;
    float GyroX_prev, GyroY_prev, GyroZ_prev;
    float MagX, MagY, MagZ;
    float MagX_prev, MagY_prev, MagZ_prev;

    float roll_IMU, pitch_IMU, yaw_IMU;
    float roll_IMU_prev, pitch_IMU_prev;
    float q0 = 1.0f; //Initialize quaternion for madgwick filter
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    //Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
    const float MagErrorX = 0.0;
    const float MagErrorY = 0.0; 
    const float MagErrorZ = 0.0;
    const float MagScaleX = 1.0;
    const float MagScaleY = 1.0;
    const float MagScaleZ = 1.0;

    //IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
    float AccErrorX = 0.0;
    float AccErrorY = 0.0;
    float AccErrorZ = 0.0;
    float GyroErrorX = 0.0;
    float GyroErrorY= 0.0;
    float GyroErrorZ = 0.0;

    //Normalized desired state:
    float thro_des, roll_des, pitch_des, yaw_des;
    float roll_passthru, pitch_passthru, yaw_passthru;

    //Controller:
    float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
    float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
    float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

    //Mixer
    float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
    int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
    float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
    int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

    //Functions
    void controlMixer();
    // void IMUinit();
    void getIMUdata();
    void calculate_IMU_error();
    void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);
    void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
    void getDesState();
    void controlANGLE(float dt);
    void controlANGLE2(float dt);
    void controlRATE(float dt);
    void scaleCommands();
    void failSafe();
    void armMotors();
    void commandMotors();
    void armServos();
    void commandServos();
    float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq);
    float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq);
    void switchRollYaw(int reverseRoll, int reverseYaw);
    void throttleCut();
    float invSqrt(float x);

protected:
    FlightController() = default;

public:
    FlightController(MX* mx, RX* rx, EX* ex);
    ~FlightController();

    void mainLoop(float dt);
    void calibrateAttitude(float dt);
    void calibrateESCs(float dt);
    void calibrateMagnetometer();
    void printDebug(uint16_t flags, float dt);
};

#endif