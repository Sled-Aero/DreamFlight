#include "RX.h"

//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup
//Radio:
//Note: If using SBUS, connect to pin 21 (RX5), if using DSM, connect to pin 15 (RX3)


RX_PPM::RX_PPM() {
    // Declare interrupt pin
    pinMode(PPM_PIN, INPUT_PULLUP);
    delay(20);

    // Attach interrupt and point to corresponding ISR function
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), getPPM, CHANGE);
}

void RX_PPM::getCommands(unsigned long channels[]) {
    channels[0] = getRadioPWM(1);
    channels[1] = getRadioPWM(2);
    channels[2] = getRadioPWM(3);
    channels[3] = getRadioPWM(4);
    channels[4] = getRadioPWM(5);
    channels[5] = getRadioPWM(6);
}

    
RX_PWM::RX_PWM() {
    //Declare interrupt pins 
    pinMode(CH1_PIN, INPUT_PULLUP);
    pinMode(CH2_PIN, INPUT_PULLUP);
    pinMode(CH3_PIN, INPUT_PULLUP);
    pinMode(CH4_PIN, INPUT_PULLUP);
    pinMode(CH5_PIN, INPUT_PULLUP);
    pinMode(CH6_PIN, INPUT_PULLUP);
    delay(20);

    //Attach interrupt and point to corresponding ISR functions
    attachInterrupt(digitalPinToInterrupt(CH1_PIN), getCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH2_PIN), getCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH3_PIN), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH4_PIN), getCh4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH5_PIN), getCh5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH6_PIN), getCh6, CHANGE);
    delay(20);

    rx_ppm = new RX_PPM();
}

void RX_PWM::getCommands(unsigned long channels[]) {
    channels[0] = getRadioPWM(1);
    channels[1] = getRadioPWM(2);
    channels[2] = getRadioPWM(3);
    channels[3] = getRadioPWM(4);
    channels[4] = getRadioPWM(5);
    channels[5] = getRadioPWM(6);
}


RX_SBUS::RX_SBUS(HardwareSerial& serial) {
    SBUS sbus(serial);
    sb = &sbus;
    sb->begin();
}

void RX_SBUS::getCommands(unsigned long channels[]) {
    if (sb->read(sbusChannels, &sbusFailSafe, &sbusLostFrame)) {
        //sBus scaling below is for Taranis-Plus and X4R-SB
        float scale = 0.615;  
        float bias  = 895.0; 
        channels[0] = sbusChannels[0] * scale + bias;
        channels[1] = sbusChannels[1] * scale + bias;
        channels[2] = sbusChannels[2] * scale + bias;
        channels[3] = sbusChannels[3] * scale + bias;
        channels[4] = sbusChannels[4] * scale + bias;
        channels[5] = sbusChannels[5] * scale + bias; 
    }
}


RX_DSM::RX_DSM(HardwareSerial& bus) {
    serial = &bus;
    serial->begin(115000);
}

void RX_DSM::getCommands(unsigned long channels[]) {
    if (DSM.timedOut(micros())) {
        //Serial.println("*** DSM RX TIMED OUT ***");
    }
    else if (DSM.gotNewFrame()) {
        uint16_t values[NUM_DSM_CHANNELS];
        DSM.getChannelValues(values, NUM_DSM_CHANNELS);

        channels[0] = values[0];
        channels[1] = values[1];
        channels[2] = values[2];
        channels[3] = values[3];
        channels[4] = values[4];
        channels[5] = values[5];
    }
}

void RX_DSM::serialEvent3(void) {
    while (serial->available()) {
        DSM.handleSerialEvent(serial->read(), micros());
    }
}


/*
 * Interrupt Service Routines are *way* too hard to do in C++. (It's a terrible language)
 * https://www.drdobbs.com/implementing-interrupt-service-routines/184401485
 */

unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

void getPPM() {
    unsigned long dt_ppm;
    int trig = digitalRead(PPM_PIN);
    if (trig==1) { //Only care about rising edge
        dt_ppm = micros() - time_ms;
        time_ms = micros();

        if (dt_ppm > 5000) { //Waiting for long pulse to indicate a new pulse train has arrived
            ppm_counter = 0;
        }
      
        if (ppm_counter == 1) { //First pulse
            channel_1_raw = dt_ppm;
        }
      
        if (ppm_counter == 2) { //Second pulse
            channel_2_raw = dt_ppm;
        }
      
        if (ppm_counter == 3) { //Third pulse
            channel_3_raw = dt_ppm;
        }
      
        if (ppm_counter == 4) { //Fourth pulse
            channel_4_raw = dt_ppm;
        }
      
        if (ppm_counter == 5) { //Fifth pulse
            channel_5_raw = dt_ppm;
        }
      
        if (ppm_counter == 6) { //Sixth pulse
            channel_6_raw = dt_ppm;
        }
        
        ppm_counter = ppm_counter + 1;
    }
}

void getCh1() {
    int trigger = digitalRead(CH1_PIN);
    if (trigger == 1) {
        rising_edge_start_1 = micros();
    }
    else if (trigger == 0) {
        channel_1_raw = micros() - rising_edge_start_1;
    }
}

void getCh2() {
    int trigger = digitalRead(CH2_PIN);
    if (trigger == 1) {
        rising_edge_start_2 = micros();
    }
    else if (trigger == 0) {
        channel_2_raw = micros() - rising_edge_start_2;
    }
}

void getCh3() {
    int trigger = digitalRead(CH3_PIN);
    if (trigger == 1) {
        rising_edge_start_3 = micros();
    }
    else if (trigger == 0) {
        channel_3_raw = micros() - rising_edge_start_3;
    }
}

void getCh4() {
    int trigger = digitalRead(CH4_PIN);
    if (trigger == 1) {
        rising_edge_start_4 = micros();
    }
    else if (trigger == 0) {
        channel_4_raw = micros() - rising_edge_start_4;
    }
}

void getCh5() {
    int trigger = digitalRead(CH5_PIN);
    if (trigger == 1) {
        rising_edge_start_5 = micros();
    }
    else if (trigger == 0) {
        channel_5_raw = micros() - rising_edge_start_5;
    }
}

void getCh6() {
    int trigger = digitalRead(CH6_PIN);
    if (trigger == 1) {
        rising_edge_start_6 = micros();
    }
    else if (trigger == 0) {
        channel_6_raw = micros() - rising_edge_start_6;
    }
}

// Get current radio commands from interrupt routines 
unsigned long getRadioPWM(int ch_num) {
    unsigned long returnPWM = 0;
    
    if (ch_num == 1) {
        returnPWM = channel_1_raw;
    }
    else if (ch_num == 2) {
        returnPWM = channel_2_raw;
    }
    else if (ch_num == 3) {
        returnPWM = channel_3_raw;
    }
    else if (ch_num == 4) {
        returnPWM = channel_4_raw;
    }
    else if (ch_num == 5) {
        returnPWM = channel_5_raw;
    }
    else if (ch_num == 6) {
        returnPWM = channel_6_raw;
    }
    
    return returnPWM;
}
