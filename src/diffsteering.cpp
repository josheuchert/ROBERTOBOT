#include <Arduino.h>
#include <Wire.h>
#include <diffsteering.h>


#define THRESHOLD 500
#define LMARKERTHRESHOLD 800
#define RMARKERTHRESHOLD 900
#define ZEROERRORVAL 600
#define LED_BUILTIN PC13
#define PWM_PERCENTAGE 30 //Update this to change motor speed (0-100)
#define STRAIGHTLINE_PWM_PERCENT 30
#define PWM_MAX 4095
#define PWM_DEADZONE 650 //PWM from 0-650 do not result in rotation of the motor
const int DRIVE_MOTOR_DUTY_CYCLE (((PWM_MAX-PWM_DEADZONE)*PWM_PERCENTAGE/100) + PWM_DEADZONE);
const int STRAIGHTLINE_DUTY_CYCLE = (((PWM_MAX-PWM_DEADZONE)*PWM_PERCENTAGE/100)+ PWM_DEADZONE);
const PinName DRIVE_MOTORS[2][2] = {{RMOTORFORWARD, RMOTORBACK},{LMOTORFORWARD, LMOTORBACK}};
bool bottomRampGoingUp;
bool bottomRampGoingDown;
bool topOfRamp;


void drivePinInit(){
   pinMode(LEFTSENSE, INPUT);  //for diffSteering.h
  pinMode(MIDLEFTSENSE, INPUT);
  pinMode(MIDRIGHTSENSE, INPUT);
  pinMode(RIGHTSENSE, INPUT);
}

int getErrorState(int previousState) {
  int currentStateLeft = analogRead(LEFTSENSE);
  int currentStateMidLeft = analogRead(MIDLEFTSENSE);
  int currentStateMidRight = analogRead(MIDRIGHTSENSE);
  int currentStateRight = analogRead(RIGHTSENSE);
  int leftMarkerReading = analogRead(LMARKERSENSE);
  int rightMarkerReading = analogRead(RMARKERSENSE);

  //If we need to turn right (tape is to right of sensors) error is positive
  //If we need to turn left, error is negative

if(leftMarkerReading > LMARKERTHRESHOLD || rightMarkerReading > RMARKERTHRESHOLD) {

if(leftMarkerReading > LMARKERTHRESHOLD && rightMarkerReading < RMARKERTHRESHOLD) {
  topOfRamp = false;
  bottomRampGoingDown = false;
  bottomRampGoingUp = true;
}

if(leftMarkerReading < LMARKERTHRESHOLD && rightMarkerReading > RMARKERTHRESHOLD) {
  topOfRamp = false;
  bottomRampGoingUp = false;
  bottomRampGoingDown = true;
}

if(leftMarkerReading > LMARKERTHRESHOLD && rightMarkerReading > RMARKERTHRESHOLD) {
  bottomRampGoingDown = false;
  bottomRampGoingUp = false;
  topOfRamp = true;
}

} else {

if(currentStateMidRight > THRESHOLD && currentStateMidLeft > THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD) {
  return 0;
}
else if(currentStateMidLeft > THRESHOLD && currentStateMidRight < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD) {
  return -1;
}
else if(currentStateMidRight > THRESHOLD && currentStateMidLeft < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD) {
  return 1;
}
else if(currentStateMidLeft > THRESHOLD && currentStateMidRight < THRESHOLD && currentStateLeft > THRESHOLD && currentStateRight < THRESHOLD) {
  return -2;
}

else if(currentStateMidRight > THRESHOLD && currentStateMidLeft < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight > THRESHOLD) {
return 2;
}

else if(currentStateMidLeft < THRESHOLD && currentStateMidRight < THRESHOLD && currentStateLeft > THRESHOLD && currentStateRight < THRESHOLD) {
return -3;
}

else if(currentStateMidRight < THRESHOLD && currentStateMidLeft < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight > THRESHOLD) {
  return 3;
}

else if(currentStateMidLeft < THRESHOLD && currentStateMidRight < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD && previousState < 0) {
return -4;
}

else if(currentStateMidRight < THRESHOLD && currentStateMidLeft < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD && previousState > 0) {
  return 4;
 }
}



return previousState;
}

int getSteeringVal(int currentErrorState, int previousState) {
  
  double kp = 0.48;
  double kd = 0.3;
  double ki = 0;
  double scaleFactor = 4*((1/(1 + exp(kp*currentErrorState + kd*(currentErrorState - previousState)))) - 0.5); //-2 to 2

  double steeringVal = (DRIVE_MOTOR_DUTY_CYCLE-PWM_DEADZONE)*scaleFactor; //range: (-2*dutyCycle, 2*dutyCycle)
  return (int) steeringVal;
  }

  void startDriveMotors(int steeringVal){
  
  if(steeringVal != 0){ 
    int i;
    int j;//right = 0, left = 1
    if(steeringVal<0){
      i = 0;
      j = 1;
    } else {
      i = 1;
      j = 0;
    }
    if(i == 1){
      steeringVal = steeringVal*1.5;
    }
    pwm_start(DRIVE_MOTORS[j][1], 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(DRIVE_MOTORS[j][0], 75, DRIVE_MOTOR_DUTY_CYCLE, RESOLUTION_12B_COMPARE_FORMAT);
    if(abs(steeringVal) <= DRIVE_MOTOR_DUTY_CYCLE-PWM_DEADZONE){
      pwm_start(DRIVE_MOTORS[i][1], 75, 0 ,RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(DRIVE_MOTORS[i][0], 75, DRIVE_MOTOR_DUTY_CYCLE - abs(steeringVal), RESOLUTION_12B_COMPARE_FORMAT);
    } else{
      pwm_start(DRIVE_MOTORS[i][0], 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(DRIVE_MOTORS[i][1], 75, abs(steeringVal)-DRIVE_MOTOR_DUTY_CYCLE+PWM_DEADZONE, RESOLUTION_12B_COMPARE_FORMAT);
    }
  }
  if(steeringVal == 0){
      for (int i = 0; i<2; i++) {
    pwm_start(DRIVE_MOTORS[i][1], 75, 0 , RESOLUTION_12B_COMPARE_FORMAT);
     pwm_start(DRIVE_MOTORS[i][0], 75, STRAIGHTLINE_DUTY_CYCLE, RESOLUTION_12B_COMPARE_FORMAT);
  }
  }
}

int getGryoFromSerial(){
  if(Serial3.available()){ //checks serial port to see if there is a message from gyro
        int val = Serial3.readString().toInt();
        if(val == 1){ //1 over serial = negative transition (flat to down or up to flat)
          return -1;
        } else if (val ==2){ //2 over serial = positive transition (flat to up or )
          return 1;
        }
  }
        return 0;
}

