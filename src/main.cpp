#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "diffsteering.h"     //differential steering funcitons
#include "objectcollection.h" //object collection funcutions
#include "scissorLift.h"
#include "MovingAverage.h"

// State definitions
#define CALIBRATE_STATE 0
#define POLL_GO_STATE 1
#define START_L 3
#define START_R 4
#define TAPE_FOLLOW_STATE 2
#define MOUNT_SL 5
#define ON_ZIPLINE 6
#define BETWEEN_LAPS_ZIPLINE_TIMER_MS 1000
#define IGNORE_GYRO_OFF_START_MS 7000

// Other definitions
#define GO_SWITCH PB14
#define RESET PB12

#define AVERAGE_OVER 1000
#define TAPE_MARKER_STATE_DELAY_MS 50


// Variable Declaration
HardwareSerial Serial3(USART3);
MovingAverage movingAverage(0.02); //exponential moving average using approx 100 terms

volatile int previousState = 0;

int loopCount = 0;
unsigned long tLastLPSCalc = millis();
unsigned long tBombDetected = millis();
//bool bombDetected = false;
bool atTopOfRamp = false;
int upTransitionCounter;
long lastMarker = millis();
long tStart = 0;
long tLastUp = 0;
long bomb_time;
bool bomb_routine = false;

long distanceCM;
int currentStateMachine;

long prev_checkStall = 0;
long zipline_time;


void loopRate();

//Strategy Information
int ZIPLINE_LAPS[1] = {1};


// Setup
void setup() {
  Serial3.begin(9600); 

  // Inits
  drivePinInit();
  objCollectionInit();
  initSL();

  pinMode(GO_SWITCH, INPUT_PULLUP);
  pinMode(RESET, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ELASTIENCODER), elastiEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_EXT), ext_limit_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_RET), ret_limit_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(SL_ENCODER), encoder_handler, RISING);
  normalObjRoutine();
  currentStateMachine = CALIBRATE_STATE;
}

void loop() {

  switch (currentStateMachine) {

    case CALIBRATE_STATE:
    {
      calibrateSL();
      //Serial3.println("Done Calibrating!");

      currentStateMachine = POLL_GO_STATE;
      //Serial3.println("POLL_GO_STATE");
      topOfRamp = false;
    }
    break;

    case POLL_GO_STATE:
    {
      if (digitalRead(GO_SWITCH) == LOW) {
        tStart = millis();
        currentStateMachine = TAPE_FOLLOW_STATE;
        //Serial3.println("ENTER TAPE FOLLOW STATE");
        normalObjRoutine();
      } 
      else if (digitalRead(RESET) == LOW) {
        tStart = millis();
        currentStateMachine = TAPE_FOLLOW_STATE;
        //Serial3.println("ENTER TAPE FOLLOW STATE");
        normalObjRoutine();
        for (int i = 0; i < sizeof(ZIPLINE_LAPS)/sizeof(ZIPLINE_LAPS[0]); i++) {
          ZIPLINE_LAPS[i] = -1;
        }
      }
     }
    break;
        
    case TAPE_FOLLOW_STATE:
    {
      // ---- DRIVING -----
      int currentState = previousState;
      movingAverage.update(previousState);
      if (millis() - lastMarker >= TAPE_MARKER_STATE_DELAY_MS) {
        currentState = getErrorState(previousState);
        if (currentState == 50) {
          currentState = previousState;
          lastMarker = millis();
        }
      }
      int steeringVal = getSteeringVal(currentState, movingAverage.get());
      startDriveMotors(steeringVal);
      previousState = currentState;
      

      //Check Stall Code
      if (millis() - prev_checkStall >= 300 && bomb_routine == false ){
        checkStall();
        prev_checkStall = millis();
      }

      
      // Bomb routine
      checkBomb();
      if (bomb_routine == false) {
        if(bombDetected) {
            bombRoutine();
            bomb_time = millis();
            bomb_routine = true;
        }
      }
      else {
        if(!bombDetected){
          if(millis() - bomb_time > 1000) {
              normalObjRoutine();
              bomb_routine = false;
            }
        }
        else {
          bomb_time = millis();
        }
      }      

      
      // Check if changed height
      if(millis() - tStart > IGNORE_GYRO_OFF_START_MS && millis() - tLastUp > BETWEEN_LAPS_ZIPLINE_TIMER_MS) {
        if (digitalRead(UP_RAMP) == HIGH) {
        upTransitionCounter++;
          for (int i = 0; i < sizeof(ZIPLINE_LAPS)/sizeof(ZIPLINE_LAPS[0]); i++) {
            if (ZIPLINE_LAPS[i] == upTransitionCounter) {
              currentStateMachine = MOUNT_SL;
            }
          }
        }
      }
    }
    break;
        
    case MOUNT_SL: {
        if(!topOfRamp) {
          if(!((analogRead(LEFTSENSE) > 500 && analogRead(MIDLEFTSENSE) > 500 && analogRead(MIDRIGHTSENSE) > 500 && analogRead(RIGHTSENSE) > 500))){
            movingAverage.update(previousState);
            int currentState = getErrorState(previousState);
            int steeringVal = getSteeringVal(currentState, movingAverage.get());
            startDriveMotors(steeringVal);
            previousState = currentState;

            //Check Stall Code
            if (millis() - prev_checkStall >= 300 && bomb_routine == false ){
              checkStall();
              prev_checkStall = millis();
            }

            
            // Bomb routine
            checkBomb();
            if (bomb_routine == false) {
              if(bombDetected) {
                  bombRoutine();
                  bomb_time = millis();
                  bomb_routine = true;
              }
            }
            else {
              if(!bombDetected){
                if(millis() - bomb_time > 1000) {
                    normalObjRoutine();
                    bomb_routine = false;
                  }
              }
              else {
                bomb_time = millis();
              }
            } 
          } 
          else {
            topOfRamp = true;
            stopElasti();
            mountingDrivingRoutine();
          }
        }

        if(extending == 0 && encoderPosition <= MOUNTPOSITION) {
          extend();
        }
        else if(encoderPosition >= MOUNTPOSITION) {
          stopScissor();
          
          if(topOfRamp == true) {
            distanceCM = getDistanceFromFloor();
            Serial3.println(distanceCM); 
            if (distanceCM >= SONAR_CLIFF_HEIGHT) {
              //Serial3.println("Over the cliff");
              //Serial3.println(distanceCM);
              extend();
              //Serial3.println("Setting to on zipline state");
              currentStateMachine = ON_ZIPLINE;
              zipline_time = millis();

              topOfRamp = false;
              pwm_start(RMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
              pwm_start(LMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
            }
          }
        }
      }  
      break;
      
      case ON_ZIPLINE: {
        distanceCM = getDistanceFromFloor();
        if (distanceCM <= SONAR_GROUND) {
          dismountRoutine();
          //Serial3.println("Entering Tape Follow State (DONE)!");
          previousState = 0;
          currentStateMachine = TAPE_FOLLOW_STATE;
          break;
        }

        if (millis() - zipline_time > 7000) {
          dismountRoutine();
          previousState = 0;
          currentStateMachine = TAPE_FOLLOW_STATE;
          break;
        }
      }
    }
}

// //Can print rate of loops, also can use to decrease number of serial prints to make them easier to read

// void loopRate()
// {
//   long curTime = millis();
//   unsigned long unitConversion = (AVERAGE_OVER * 1000);
//   double lps = unitConversion / static_cast<long double>(curTime - tLastLPSCalc);
//   Serial3.print("Loops per second: ");
//   Serial3.println(lps);
//   Serial3.print(analogRead(LMARKERSENSE));
//   Serial3.print(" ");
//   Serial3.print(analogRead(LEFTSENSE));
//   Serial3.print(" ");
//   Serial3.print(analogRead(MIDLEFTSENSE));
//   Serial3.print(" ");
//   Serial3.print(analogRead(MIDRIGHTSENSE));
//   Serial3.print(" ");
//   Serial3.print(analogRead(RIGHTSENSE));
//   Serial3.print(" ");
//   Serial3.print(analogRead(RMARKERSENSE));
//   Serial3.print(" ");
//   Serial3.print(previousState);
//   Serial3.println();

//   // Serial.print("CurrentTime: ");
//   // Serial.println(curTime);
//   // Serial.print("StartTime:  ");
//   // Serial.println(tStart);
//   tLastLPSCalc = millis();
// }
