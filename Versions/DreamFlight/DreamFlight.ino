#include "Arduino.h"
#include "FlightController.h"

#ifndef HAVE_HWSERIAL0
  #include "HardwareSerial.h"
  extern HardwareSerial Serial;
#endif

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
//printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
//printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
//printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
//printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
//printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
//printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
//printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
//printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
//printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
//printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)
uint16_t debug_flags = 0x0;

RX *rx;
MX *mx;
FlightController* fc;

void setup() {
  Serial.begin(500000); //USB serial
  delay(500);
  pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify 

  rx = new RX_PPM();
  //rx = new RX_PWM();
  //rx = new RX_SBUS(Serial3);
  //rx = new RX_DSM(Serial3);

  mx = new MX_6050();
  //mx = new MX_9250();

  fc = new FlightController(mx, rx);

  //Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); //numBlinks, upTime (ms), downTime (ms)
}

//========================================================================================================================//
//  MAIN LOOP  
//========================================================================================================================//
                                                  
void loop() {
    // Keep track of what time it is and how much time has elapsed since the last loop
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0;

    loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

    // Debug
    if ((debug_flags) && (current_time - print_counter > 10000)) {
        print_counter = micros();
        fc->printDebug(debug_flags, dt);
    }

    // Flight Controller main loop
    fc->mainLoop(dt);

    // Regulate loop rate
    loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

void calibrateAttitude() {
    //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
    //Assuming vehicle is powered up on level surface!
    /*
      * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
      * to boot. 
      */

    // Warm up IMU and madgwick filter in simulated main loop
    for (int i = 0; i <= 10000; i++) {
        prev_time = current_time;      
        current_time = micros();      
        dt = (current_time - prev_time)/1000000.0; 

        fc->calibrateAttitude(dt);
        
        loopRate(2000); //do not exceed 2000Hz
    }
}

void calibrateESCs() {
  //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*  
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
   *  uncommented when performing an ESC calibration.
   */
   while (true) {
      prev_time = current_time;      
      current_time = micros();      
      dt = (current_time - prev_time)/1000000.0;
    
      digitalWrite(13, HIGH); //LED on to indicate we are not in main loop

      fc->calibrateESCs(dt);

      loopRate(2000); //do not exceed 2000Hz
   }
}

void loopRate(int freq) {
    //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
    /*
      * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
      * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
      * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
      * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
      * and remain above 2kHz, without needing to retune all of our filtering parameters.
      */
    float invFreq = 1.0/freq*1000000.0;
    unsigned long checker = micros();

    //Sit in loop until appropriate time has passed
    while (invFreq > (checker - current_time)) {
        checker = micros();
    }
}

void loopBlink() {
    //DESCRIPTION: Blink LED on board to indicate main loop is running
    /*
      * It looks cool.
      */
    if (current_time - blink_counter > blink_delay) {
        blink_counter = micros();
        digitalWrite(13, blinkAlternate); //Pin 13 is built in LED
        
        if (blinkAlternate == 1) {
            blinkAlternate = 0;
            blink_delay = 100000;
        }
        else if (blinkAlternate == 0) {
            blinkAlternate = 1;
            blink_delay = 2000000;
        }
    }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
    //DESCRIPTION: Simple function to make LED on board blink as desired
    for (int j = 1; j<= numBlinks; j++) {
        digitalWrite(13, LOW);
        delay(downTime);
        digitalWrite(13, HIGH);
        delay(upTime);
    }
}
