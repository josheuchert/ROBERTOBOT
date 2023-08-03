#include <Arduino.h>
#include <Wire.h>
#include <scissorLift.h>
#include <diffsteering.h>
int encoderPosition;
bool go;
bool extending;
volatile bool calibrateStatus;


void initSL() {

    pinMode(LIMIT_SWITCH_EXT, INPUT);
    pinMode(LIMIT_SWITCH_RET, INPUT);
    pinMode(SL_ENCODER, INPUT);
    pinMode(GO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_EXT), ext_limit_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_RET), ret_limit_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(SL_ENCODER), encoder_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(GO_PIN), go_handler, RISING);

    go = false;

}

void ext_limit_handler() {
    Serial3.println("Top Limit Switch Triggered");
    pwm_start(SCISSOR_MOTOR_UP, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(SCISSOR_MOTOR_DOWN, 50, 0, RESOLUTION_12B_COMPARE_FORMAT);
    extending = 0;
}


void ret_limit_handler() {
    Serial3.println("Bottom Limit Switch Triggered");
    pwm_start(SCISSOR_MOTOR_UP, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(SCISSOR_MOTOR_DOWN, 50, 0, RESOLUTION_12B_COMPARE_FORMAT);
    encoderPosition = 0;
    calibrateStatus = 1;
    extending = 0;
    
    // extending = 1;
    // delay(1000);
    // pwm_start(SCISSOR_MOTOR_UP, 75, 4000, RESOLUTION_12B_COMPARE_FORMAT);
}

void encoder_handler() {
    encoderPosition++;
    //Serial3.println(encoderPosition);
}

void go_handler() {
    go = true;
}

void extend() {
    if(digitalRead(LIMIT_SWITCH_EXT) == HIGH) {
        Serial3.println("Detected High Extentsion switch and tried to extend!");
    }
    else {
        Serial3.println("Extending");
        pwm_start(SCISSOR_MOTOR_DOWN, 50, 0, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(SCISSOR_MOTOR_UP, 75, 4000, RESOLUTION_12B_COMPARE_FORMAT);
        extending = 1;
    }
}

void retract() {
    if(digitalRead(LIMIT_SWITCH_RET) == HIGH){
        Serial3.println("Detected High Bottom switch and tried to retract!");
    }
    else {
        Serial3.println("Retracting");
        pwm_start(SCISSOR_MOTOR_UP, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(SCISSOR_MOTOR_DOWN, 50, 4000, RESOLUTION_12B_COMPARE_FORMAT);
        extending = 0;
    }
}

void stopScissor() {
    Serial3.println("Stop Scissor fcn");
    pwm_start(SCISSOR_MOTOR_UP, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(SCISSOR_MOTOR_DOWN, 50, 0, RESOLUTION_12B_COMPARE_FORMAT);
    extending = 0;
}

void calibrateSL() {
    Serial3.println("Calibrating SL");
    calibrateStatus = 0;

    if(digitalRead(LIMIT_SWITCH_EXT) == HIGH) {
        Serial3.println("Detected High Extentsion switch");
        retract();
    }
    else if(digitalRead(LIMIT_SWITCH_RET) == HIGH){
        Serial3.println("Detected High Bottom switch");
        int startTime = millis();
        extend();
        delay(1200);
        calibrateStatus = 0;
        retract();
    }
    else {
        Serial3.println("Detected Neither Switch");
        retract();
    }
    
    while(calibrateStatus == 0){
        delay(500);
        Serial3.println("Waiting...");
    }
    Serial3.println("Done");
}

long getDistanceFromFloor() {
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);

    return duration / 29 / 2;

}

void mountingDrivingRoutine(){
    Serial3.println("Performing Mounting Driving Routine");
    pwm_start(RMOTORBACK, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(LMOTORBACK, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RMOTORFORWARD, 75, 1000, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(LMOTORFORWARD, 75, 1000, RESOLUTION_12B_COMPARE_FORMAT);
}

void dismountDrivingRoutine(){
    Serial3.println("Performing Dismount Driving Routine");
    delay(1000);
    pwm_start(RMOTORFORWARD, 75, 2000, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(LMOTORFORWARD, 75, 600, RESOLUTION_12B_COMPARE_FORMAT);
    delay(1000);
    pwm_start(RMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(LMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    delay(1000);
    calibrateStatus = 0;
    retract();
    while (calibrateStatus == 0){
      delay(200);
    }
}




