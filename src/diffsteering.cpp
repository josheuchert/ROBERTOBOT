#include <Arduino.h>
#include <Wire.h>
#include <diffsteering.h>

#define THRESHOLD 500
#define LMARKERTHRESHOLD 1050
#define RMARKERTHRESHOLD 1050
#define ZEROERRORVAL 600
#define LED_BUILTIN PC13
#define PWM_PERCENTAGE 50 // Update this to change motor speed (0-100)
#define STRAIGHTLINE_PWM_PERCENT 50
#define PWM_MAX 4095
#define PWM_DEADZONE 650 // PWM from 0-650 do not result in rotation of the motor
const int DRIVE_MOTOR_DUTY_CYCLE(((PWM_MAX - PWM_DEADZONE) * PWM_PERCENTAGE / 100) + PWM_DEADZONE);
const int STRAIGHTLINE_DUTY_CYCLE = (((PWM_MAX - PWM_DEADZONE) * PWM_PERCENTAGE / 100) + PWM_DEADZONE);
const PinName DRIVE_MOTORS[2][2] = {{RMOTORFORWARD, RMOTORBACK}, {LMOTORFORWARD, LMOTORBACK}};
bool bottomRampGoingUp;
bool bottomRampGoingDown;
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

  if (currentStateMidRight > THRESHOLD && currentStateMidLeft > THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 0;
  }
  if(currentStateMidRight > THRESHOLD && currentStateMidLeft > THRESHOLD && currentStateLeft > THRESHOLD && currentStateRight > THRESHOLD && leftMarkerReading < THRESHOLD && rightMarkerReading <THRESHOLD){
    return 0;
  }
  else if (currentStateMidLeft > THRESHOLD && currentStateMidRight < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return -1;
  }
  else if(currentStateMidLeft > THRESHOLD && currentStateLeft > THRESHOLD && currentStateMidRight > THRESHOLD && currentStateRight < THRESHOLD && leftMarkerReading < THRESHOLD && rightMarkerReading < THRESHOLD){
    digitalWrite(PA3, LOW);
    return -1;
  }
  else if (currentStateMidRight > THRESHOLD && currentStateMidLeft < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 1;
  }
  else if (currentStateMidRight > THRESHOLD && currentStateMidLeft > THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight > THRESHOLD  && leftMarkerReading < THRESHOLD && rightMarkerReading < THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 1;
  }
  else if (currentStateMidLeft > THRESHOLD && currentStateMidRight < THRESHOLD && currentStateLeft > THRESHOLD && currentStateRight < THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return -2;
  }

  else if (currentStateMidRight > THRESHOLD && currentStateMidLeft < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight > THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 2;
  }

  else if (currentStateMidLeft < THRESHOLD && currentStateMidRight < THRESHOLD && currentStateLeft > THRESHOLD && currentStateRight < THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return -3;
  }

  else if (currentStateMidRight < THRESHOLD && currentStateMidLeft < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight > THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 3;
  }

  else if (currentStateMidLeft < THRESHOLD && currentStateMidRight < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD && previousState < 0 && previousState > -5)
  {
    digitalWrite(PA3, LOW);
    return -4;
  }

  else if (currentStateMidRight < THRESHOLD && currentStateMidLeft < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD && previousState > 0 && previousState < 5)
  {
    digitalWrite(PA3, LOW);
    return 4;
   } else if(currentStateMidRight < THRESHOLD && currentStateMidLeft < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD && previousState >=4 &&rightMarkerReading > THRESHOLD && leftMarkerReading < THRESHOLD){
    digitalWrite(PA3, LOW);
    return 6;
  }  else if(currentStateMidRight < THRESHOLD && currentStateMidLeft < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD && rightMarkerReading < THRESHOLD && leftMarkerReading > THRESHOLD && previousState <= -4 ){
    digitalWrite(PA3, LOW);
    return -6;
  }
  else if (currentStateMidLeft < THRESHOLD && currentStateMidRight < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD && previousState == -5 && leftMarkerReading < THRESHOLD && rightMarkerReading < THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return -10;
  }
  else if (currentStateMidLeft < THRESHOLD && currentStateMidRight < THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight < THRESHOLD && previousState == 5 && leftMarkerReading < THRESHOLD && rightMarkerReading < THRESHOLD)
  {
    digitalWrite(PA3, LOW);
    return 10;
  } else if(currentStateMidRight > THRESHOLD && currentStateMidLeft > THRESHOLD && currentStateLeft > THRESHOLD && currentStateRight > THRESHOLD && leftMarkerReading > THRESHOLD && rightMarkerReading > THRESHOLD) {
    return 50;
  } else if(currentStateMidRight > THRESHOLD && currentStateMidLeft > THRESHOLD && currentStateLeft > THRESHOLD && currentStateRight > THRESHOLD && leftMarkerReading > THRESHOLD && rightMarkerReading < THRESHOLD) {
    return 50;
  }
  else if(currentStateMidRight > THRESHOLD && currentStateMidLeft > THRESHOLD && currentStateLeft > THRESHOLD && currentStateRight > THRESHOLD && leftMarkerReading < THRESHOLD && rightMarkerReading > THRESHOLD) {
    return 50;
  } else if(currentStateMidRight > THRESHOLD && currentStateMidLeft > THRESHOLD && currentStateLeft < THRESHOLD && currentStateRight > THRESHOLD && leftMarkerReading < THRESHOLD && rightMarkerReading > THRESHOLD) {
    return 50;
  }else if(currentStateMidRight > THRESHOLD && currentStateMidLeft > THRESHOLD && currentStateLeft > THRESHOLD && currentStateRight < THRESHOLD && leftMarkerReading > THRESHOLD && rightMarkerReading < THRESHOLD) {
    return 50;
  }
  digitalWrite(PA3, HIGH);
    // Serial3.print(analogRead(LMARKERSENSE));
    // Serial3.print(" ");
    // Serial3.print(analogRead(LEFTSENSE));
    // Serial3.print(" ");
    // Serial3.print(analogRead(MIDLEFTSENSE));
    // Serial3.print(" ");
    // Serial3.print(analogRead(MIDRIGHTSENSE));
    // Serial3.print(" ");
    // Serial3.print(analogRead(RIGHTSENSE));
    // Serial3.print(" ");
    // Serial3.print(analogRead(RMARKERSENSE));
    // Serial3.println();
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
      pwm_start(DRIVE_MOTORS[i][1], 75, abs(steeringVal) - DRIVE_MOTOR_DUTY_CYCLE + PWM_DEADZONE, RESOLUTION_12B_COMPARE_FORMAT);
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

int getGryoFromSerial()
{
  if (Serial3.available())
  { // checks serial port to see if there is a message from gyro
    int val = Serial3.readString().toInt();
    if (val == 1)
    { // 1 over serial = negative transition (flat to down or up to flat)
      return -1;
    }
    else if (val == 2)
    { // 2 over serial = positive transition (flat to up or )
      return 1;
    }
  }
  return 0;
}

void stopDriveMotors()
{
  pwm_start(RMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RMOTORBACK, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LMOTORBACK, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
}
