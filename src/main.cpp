#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "diffsteering.h" //differential steering funcitons
#include "objectcollection.h" //object collection funcutions
#include "scissorLift.h"
using namespace std;

#define DERIV_OVER_MS 100

#define CALIBRATE_STATE 0
#define POLL_GO_STATE 1
#define START_L 3
#define START_R 4
#define TAPE_FOLLOW_STATE 2
#define MOUNT_SL 5
#define ON_ZIPLINE 6
#define EXTENDING_SL 7
const int ZIPLINE_LAPS[1] = {1};

// Variable Declaration
HardwareSerial Serial3(USART3);
#define AVERAGE_OVER 1000
unsigned long tLastDeriv = millis();

volatile int derivState = 0;
volatile int previousState = 0;
int loopCount = 0;
unsigned long tLastLPSCalc = millis();
unsigned long tBombDetected = millis();
bool bombDetected = false;
int rampState = 0; //-1 for downhill, 0 for flat, 1 for up
int lapNum = 0;

int prev_time = 0;
bool state = 0;
long distanceCM;
int currentStateMachine;


void loopRate();

// Setup (unchanged)
void setup() {
  Serial3.begin(9600); //BLUEPILL TX: B10, RX:B11

  drivePinInit();
  //objCollectionInit();
  pinMode(LED_BUILTIN, OUTPUT);
  initSL(); //scissorlift init

  currentStateMachine = TAPE_FOLLOW_STATE;
  pwm_start(ELASTIFORWARD,75, 2000, RESOLUTION_12B_COMPARE_FORMAT);
}


void loop() {

  switch(currentStateMachine) {
    
    case CALIBRATE_STATE: {
      delay(500);
      calibrateSL();
      Serial3.println("Done Calibrating!");
      
      currentStateMachine = MOUNT_SL;
      Serial3.println("ENTER MOUNT SL STATE");
      delay(1000);}
      break;

    case POLL_GO_STATE:{
      delay(500);
      //if go switch is flipped 
      currentStateMachine = TAPE_FOLLOW_STATE;
      Serial3.println("ENTER TAPE FOLLOW STATE");
      //pwm_start(ELASTIFORWARD,75,2000, RESOLUTION_12B_COMPARE_FORMAT);
    }
      break;
   
    
    case TAPE_FOLLOW_STATE: {
      // ---- DRIVING -----
      if(millis()-tLastDeriv > DERIV_OVER_MS){
        tLastDeriv = millis();
        derivState = previousState;
      }
      int currentState = getErrorState(previousState);
      int steeringVal = getSteeringVal(currentState, derivState);
      startDriveMotors(steeringVal);
      previousState = currentState;
      int newRampState = getGryoFromSerial(); //might want to make it check this less than every loop, could be slow
      rampState += newRampState;
      if(newRampState == 1 && rampState == 1){
        lapNum +=1;
         for(int i = 0; i<sizeof(ZIPLINE_LAPS); i++){
              if(ZIPLINE_LAPS[i]==lapNum){
                currentStateMachine = MOUNT_SL;
              }
            }
      }
      //if at bottom of ramp change state to Mount SL
    }
      break;
    
    case MOUNT_SL: {
      if(rampState == 1) {
        if(millis()-tLastDeriv > DERIV_OVER_MS){
        tLastDeriv = millis();
        derivState = previousState;
      }
      int currentState = getErrorState(previousState);
      int steeringVal = getSteeringVal(currentState, derivState);
      startDriveMotors(steeringVal);
      previousState = currentState;
        int newRampState = getGryoFromSerial(); //might want to make it check this less than every loop, could be slow
      rampState += newRampState;
      } else{
        stopDriveMotors();
        stopElasti();
      }
      if(extending == 0 && encoderPosition <= MOUNTPOSITION) {
        extend();
      }
      else if(encoderPosition >= MOUNTPOSITION) {
        // maybe add an extending == 1 ^^
        stopScissor();
        Serial3.println("In Mount Position!");
        delay(1000);

        //move down to if at tape marker --> exclude check if extending to test before that
        mountingDrivingRoutine();
       
        //ignore
        distanceCM = getDistanceFromFloor();
        if (distanceCM >= SONAR_CLIFF_HEIGHT){
          extend();
          Serial3.println("Setting to on zipline state");
          currentStateMachine = ON_ZIPLINE;
          pwm_start(RMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(LMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
        }
        // if (distanceCM >= SONAR_CLIFF_HEIGHT) {
        //   extend();
        //   currentState = ON_ZIPLINE;
        //   pwm_start(RMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
        //   pwm_start(LMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
      }
      
      // distanceCM = getDistanceFromFloor();
      // if (distanceCM >= SONAR_CLIFF_HEIGHT){
      //   extend();
      //   Serial3.println("Setting to on zipline state");
      //   currentState = ON_ZIPLINE;
      //   pwm_start(RMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
      //   pwm_start(LMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
      // }
    }  
    break;


      //ignore for now
      // delay(1500);
      // pwm_start(DRIVE_RF, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
      // pwm_start(DRIVE_LF, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
      
       // Add an if for if at top of ramp and on zipline lap
      // pwm_start(DRIVE_RF, 75, 2000, RESOLUTION_12B_COMPARE_FORMAT);
      // pwm_start(DRIVE_LF, 75, 2000, RESOLUTION_12B_COMPARE_FORMAT);
      
      // distanceCM = getDistanceFromFloor();
      // if (distanceCM > SONAR_CLIFF_HEIGHT) {
      //   extend();
        //state is on zipline

    
    case ON_ZIPLINE:{
      distanceCM = getDistanceFromFloor();
      if (distanceCM <= SONAR_GROUND) {
        delay(2000);
        stopScissor();
        dismountDrivingRoutine();
        Serial3.println("Entering Tape Follow State (DONE)!");
        
        currentStateMachine = TAPE_FOLLOW_STATE;
      }
      
    }
    break;
  }
 
  // BLUEPILL HEART BEAT
  if (millis() - prev_time > 500) {
    if (state == 0) {
      digitalWrite(PC13, LOW);
      state = 1;
    }
    else {
      digitalWrite(PC13, HIGH);
      state = 0;
    }
    prev_time = millis();
  }
 }


// //Can print rate of loops, also can use to decrease number of serial prints to make them easier to read

// void loopRate(){
//   long curTime = millis();
//   unsigned long unitConversion = (AVERAGE_OVER*1000);
//   double lps = unitConversion / static_cast<long double>(curTime - tLastLPSCalc);
//   //Serial.print("Loops per second: ");
//   //Serial.println(lps);

//     Serial3.print(analogRead(LEFTSENSE));
//     Serial3.print(" ");
//     Serial3.print(analogRead(MIDLEFTSENSE));
//     Serial3.print(" ");
//     Serial3.print(analogRead(MIDRIGHTSENSE));
//     Serial3.print(" ");
//     Serial3.print(analogRead(RIGHTSENSE));
//     Serial3.println();
  
  
//   // Serial.print("CurrentTime: ");
//   // Serial.println(curTime);
//   // Serial.print("StartTime:  ");
//   // Serial.println(tStart);
//   tLastLPSCalc = millis();
//}

