#include <Arduino.h>
#include <Wire.h>
#include <diffsteering.h>


#define PWM_PERCENTAGE 40 // Update this to change motor speed (0-100)
#define STRAIGHTLINE_PWM_PERCENT 40
#define PWM_MAX 4095
#define PWM_DEADZONE 650 // PWM from 0-650 do not result in rotation of the motor
const int DRIVE_MOTOR_DUTY_CYCLE(((PWM_MAX - PWM_DEADZONE) * PWM_PERCENTAGE / 100) + PWM_DEADZONE);
const int STRAIGHTLINE_DUTY_CYCLE = (((PWM_MAX - PWM_DEADZONE) * PWM_PERCENTAGE / 100) + PWM_DEADZONE);
const PinName DRIVE_MOTORS[2][2] = {{RMOTORFORWARD, RMOTORBACK}, {LMOTORFORWARD, LMOTORBACK}};
bool topOfRamp;

void drivePinInit()
{
  pinMode(LEFTSENSE, INPUT); // for diffSteering.h
  pinMode(MIDLEFTSENSE, INPUT);
  pinMode(MIDRIGHTSENSE, INPUT);
  pinMode(RIGHTSENSE, INPUT);
  pinMode(LMARKERSENSE, INPUT);
  pinMode(RMARKERSENSE, INPUT);
}

int getErrorState(int previousState)
{
  int currentStateLeft = analogRead(LEFTSENSE);
  int currentStateMidLeft = analogRead(MIDLEFTSENSE);
  int currentStateMidRight = analogRead(MIDRIGHTSENSE);
  int currentStateRight = analogRead(RIGHTSENSE);
  int leftMarkerReading = analogRead(LMARKERSENSE);
  int rightMarkerReading = analogRead(RMARKERSENSE);


  // If we need to turn right (tape is to right of sensors) error is positive
  // If we need to turn left, error is negative

  if (currentStateMidRight > TAPE_THRESHOLD && currentStateMidLeft > TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 0;
  }
  if(currentStateMidRight > TAPE_THRESHOLD && currentStateMidLeft > TAPE_THRESHOLD && currentStateLeft > TAPE_THRESHOLD && currentStateRight > TAPE_THRESHOLD && leftMarkerReading < TAPE_THRESHOLD && rightMarkerReading <TAPE_THRESHOLD){
    return 0;
  }
  else if (currentStateMidLeft > TAPE_THRESHOLD && currentStateMidRight < TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return -1;
  }
  else if(currentStateMidLeft > TAPE_THRESHOLD && currentStateLeft > TAPE_THRESHOLD && currentStateMidRight > TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD && leftMarkerReading < TAPE_THRESHOLD && rightMarkerReading < TAPE_THRESHOLD){
    digitalWrite(PA3, LOW);
    return -1;
  }
  else if (currentStateMidRight > TAPE_THRESHOLD && currentStateMidLeft < TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 1;
  }
  else if (currentStateMidRight > TAPE_THRESHOLD && currentStateMidLeft > TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight > TAPE_THRESHOLD  && leftMarkerReading < TAPE_THRESHOLD && rightMarkerReading < TAPE_THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 1;
  }
  else if (currentStateMidLeft > TAPE_THRESHOLD && currentStateMidRight < TAPE_THRESHOLD && currentStateLeft > TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return -2;
  }

  else if (currentStateMidRight > TAPE_THRESHOLD && currentStateMidLeft < TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight > TAPE_THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 2;
  }

  else if (currentStateMidLeft < TAPE_THRESHOLD && currentStateMidRight < TAPE_THRESHOLD && currentStateLeft > TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return -3;
  }

  else if (currentStateMidRight < TAPE_THRESHOLD && currentStateMidLeft < TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight > TAPE_THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 3;
  }

  else if (currentStateMidLeft < TAPE_THRESHOLD && currentStateMidRight < TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD && previousState < 0 && previousState > -5)
  {
    digitalWrite(PA3, LOW);
    return -4;
  }

  else if (currentStateMidRight < TAPE_THRESHOLD && currentStateMidLeft < TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD && previousState > 0 && previousState < 5)
  {
    digitalWrite(PA3, LOW);
    return 4;
   } else if(currentStateMidRight < TAPE_THRESHOLD && currentStateMidLeft < TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD && previousState >=4 &&rightMarkerReading > TAPE_THRESHOLD && leftMarkerReading < TAPE_THRESHOLD){
    digitalWrite(PA3, LOW);
    return 6;
  }  else if(currentStateMidRight < TAPE_THRESHOLD && currentStateMidLeft < TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD && rightMarkerReading < TAPE_THRESHOLD && leftMarkerReading > TAPE_THRESHOLD && previousState <= -4 ){
    digitalWrite(PA3, LOW);
    return -6;
  }
  else if (currentStateMidLeft < TAPE_THRESHOLD && currentStateMidRight < TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD && previousState == -5 && leftMarkerReading < TAPE_THRESHOLD && rightMarkerReading < TAPE_THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return -10;
  }
  else if (currentStateMidLeft < TAPE_THRESHOLD && currentStateMidRight < TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD && previousState == 5 && leftMarkerReading < TAPE_THRESHOLD && rightMarkerReading < TAPE_THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 10;
  } else if(currentStateMidRight > TAPE_THRESHOLD && currentStateMidLeft > TAPE_THRESHOLD && currentStateLeft > TAPE_THRESHOLD && currentStateRight > TAPE_THRESHOLD && leftMarkerReading > TAPE_THRESHOLD && rightMarkerReading > TAPE_THRESHOLD) {
    return 50;
  } else if(currentStateMidRight > TAPE_THRESHOLD && currentStateMidLeft > TAPE_THRESHOLD && currentStateLeft > TAPE_THRESHOLD && currentStateRight > TAPE_THRESHOLD && leftMarkerReading > TAPE_THRESHOLD && rightMarkerReading < TAPE_THRESHOLD) {
    return 50;
  }
  else if(currentStateMidRight > TAPE_THRESHOLD && currentStateMidLeft > TAPE_THRESHOLD && currentStateLeft > TAPE_THRESHOLD && currentStateRight > TAPE_THRESHOLD && leftMarkerReading < TAPE_THRESHOLD && rightMarkerReading > TAPE_THRESHOLD) {
    return 50;
  } else if(currentStateMidRight > TAPE_THRESHOLD && currentStateMidLeft > TAPE_THRESHOLD && currentStateLeft < TAPE_THRESHOLD && currentStateRight > TAPE_THRESHOLD && leftMarkerReading < TAPE_THRESHOLD && rightMarkerReading > TAPE_THRESHOLD) {
    return 50;
  }else if(currentStateMidRight > TAPE_THRESHOLD && currentStateMidLeft > TAPE_THRESHOLD && currentStateLeft > TAPE_THRESHOLD && currentStateRight < TAPE_THRESHOLD && leftMarkerReading > TAPE_THRESHOLD && rightMarkerReading < TAPE_THRESHOLD) {
    return 50;
  }
  
  digitalWrite(PA3, HIGH);
  return previousState;
}

int getSteeringVal(int currentErrorState, double previousState)
{

  double kp = 0.35;
  double kd = 0.35;
  double ki = 0;
  double scaleFactor = 4 * ((1 / (1 + exp(kp * currentErrorState + kd * (currentErrorState - previousState)))) - 0.5); //-2 to 2

  double steeringVal = (DRIVE_MOTOR_DUTY_CYCLE - PWM_DEADZONE) * scaleFactor; // range: (-2*dutyCycle, 2*dutyCycle)
  return (int)steeringVal;
}

void startDriveMotors(int steeringVal)
{

  if (steeringVal != 0)
  {
    int i;
    int j; // right = 0, left = 1
    if (steeringVal < 0)
    {
      i = 0;
      j = 1;
    }
    else
    {
      i = 1;
      j = 0;
    }
    if (i == 1)
    {
      steeringVal = steeringVal * 1.75;
    }
    pwm_start(DRIVE_MOTORS[j][1], 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(DRIVE_MOTORS[j][0], 75, DRIVE_MOTOR_DUTY_CYCLE, RESOLUTION_12B_COMPARE_FORMAT);
    if (abs(steeringVal) <= DRIVE_MOTOR_DUTY_CYCLE - PWM_DEADZONE)
    {
      pwm_start(DRIVE_MOTORS[i][1], 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(DRIVE_MOTORS[i][0], 75, DRIVE_MOTOR_DUTY_CYCLE - abs(steeringVal), RESOLUTION_12B_COMPARE_FORMAT);
    }
    else
    {
      pwm_start(DRIVE_MOTORS[i][0], 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
      if(abs(steeringVal) - DRIVE_MOTOR_DUTY_CYCLE + PWM_DEADZONE > 4095){
        pwm_start(DRIVE_MOTORS[i][1], 75, 4095, RESOLUTION_12B_COMPARE_FORMAT);
      }
      else{
      pwm_start(DRIVE_MOTORS[i][1], 75, abs(steeringVal) - DRIVE_MOTOR_DUTY_CYCLE + PWM_DEADZONE, RESOLUTION_12B_COMPARE_FORMAT);
      }
    }
  }
  if (steeringVal == 0)
  {
    for (int i = 0; i < 2; i++)
    {
      pwm_start(DRIVE_MOTORS[i][1], 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(DRIVE_MOTORS[i][0], 75, STRAIGHTLINE_DUTY_CYCLE, RESOLUTION_12B_COMPARE_FORMAT);
    }
  }
}

void stopDriveMotors()
{
  pwm_start(RMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RMOTORBACK, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LMOTORBACK, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
}
