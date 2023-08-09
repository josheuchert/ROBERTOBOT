#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "diffsteering.h"     //differential steering funcitons
#include "objectcollection.h" //object collection funcutions
#include "scissorLift.h"
#include "MovingAverage.h"

#define DERIV_OVER_MS 100

#define CALIBRATE_STATE 0
#define POLL_GO_STATE 1
#define START_L 3
#define START_R 4
#define TAPE_FOLLOW_STATE 2
#define MOUNT_SL 5
#define ON_ZIPLINE 6
#define EXTENDING_SL 7
#define ERROR_LED PA3
#define GO_SWITCH PB12
#define AVERAGE_OVER 1000
const int ZIPLINE_LAPS[1] = {1};
bool atTopOfRamp = false;
int upTransitionCounter;
#define TAPE_MARKER_STATE_DELAY_MS 50
long lastMarker = millis();

// Variable Declaration
HardwareSerial Serial3(USART3);
MovingAverage movingAverage(0.02); //exponential moving average using approx 100 terms
#define AVERAGE_OVER 1000

volatile int previousState = 0;

int loopCount = 0;
unsigned long tLastLPSCalc = millis();
unsigned long tBombDetected = millis();
bool bombDetected = false;

long distanceCM;
int currentStateMachine;

void loopRate();

// Setup (unchanged)
void setup()
{
  Serial3.begin(9600); // BLUEPILL TX: B10, RX:B11

  drivePinInit();
  objCollectionInit();
  initSL(); //scissorlift init
  
  currentStateMachine = CALIBRATE_STATE;
  attachInterrupt(digitalPinToInterrupt(ELASTIENCODER), elastiEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_EXT), ext_limit_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_RET), ret_limit_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(SL_ENCODER), encoder_handler, RISING);
}

void loop()
{

  switch (currentStateMachine)
  {

  case CALIBRATE_STATE:
  {
    calibrateSL();
    Serial3.println("Done Calibrating!");

    currentStateMachine = POLL_GO_STATE;
    Serial3.println("POLL_GO_STATE");
    topOfRamp = false;
  }

  break;

  case POLL_GO_STATE:
  {
    while (digitalRead(GO_SWITCH))
    {
      delay(1);
    }
    currentStateMachine = TAPE_FOLLOW_STATE;
    Serial3.println("ENTER TAPE FOLLOW STATE");
    normalObjRoutine();
  }
  break;
      
  case TAPE_FOLLOW_STATE:
  {
    // ---- DRIVING -----
    int currentState = previousState;
    movingAverage.update(previousState);
    if (millis() - lastMarker >= TAPE_MARKER_STATE_DELAY_MS)
    {
      currentState = getErrorState(previousState);
      if (currentState == 50)
      {
        currentState = previousState;
      }
       }
    int steeringVal = getSteeringVal(currentState, movingAverage.get());
    startDriveMotors(steeringVal);
    previousState = currentState;
    if (digitalRead(UP_RAMP) == HIGH)
    {
      upTransitionCounter++;
      for (int i = 0; i < sizeof(ZIPLINE_LAPS); i++)
      {
        if (ZIPLINE_LAPS[i] == (upTransitionCounter + 1) / 2);
        currentStateMachine = MOUNT_SL;
        upTransitionCounter++;
      }
      //if at bottom of ramp change state to Mount SL
    }
  }
  // if at bottom of ramp change state to Mount SL
  break;
      
  case MOUNT_SL: {
      if(!topOfRamp) {
        if(!((analogRead(LEFTSENSE) > 500 && analogRead(MIDLEFTSENSE) > 500 && analogRead(MIDRIGHTSENSE) > 500 && analogRead(RIGHTSENSE) > 500))){
          movingAverage.update(previousState);
          int currentState = getErrorState(previousState);
          int steeringVal = getSteeringVal(currentState, movingAverage.get());
          startDriveMotors(steeringVal);
          previousState = currentState;
        } 
        else {
          topOfRamp = true;
          mountingDrivingRoutine();
          stopElasti();
        }
      }

      if(extending == 0 && encoderPosition <= MOUNTPOSITION) {
        extend();
      }
      else if(encoderPosition >= MOUNTPOSITION) {
        stopScissor();
        //Serial3.println("In Mount Position!");
        //delay(1000);

        //move down to if at tape marker --> exclude check if extending to test before that
        if(topOfRamp == true){
          distanceCM = getDistanceFromFloor();
          Serial3.println(distanceCM);
          if (distanceCM >= SONAR_CLIFF_HEIGHT) {
            Serial3.println("over the cliff");
            Serial3.println(distanceCM);
            extend();
            Serial3.println("Setting to on zipline state");
            currentStateMachine = ON_ZIPLINE;
            topOfRamp = false;
            pwm_start(RMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(LMOTORFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
          }
        }
      }
    }  
    break;
    
    case ON_ZIPLINE:{
      distanceCM = getDistanceFromFloor();
      if (distanceCM <= SONAR_GROUND) {
        delay(200);
        stopScissor();
        dismountDrivingRoutine();
        Serial3.println("Entering Tape Follow State (DONE)!");
        previousState = 0;
        currentStateMachine = TAPE_FOLLOW_STATE;
      }
      
    }
    break;
  }
}

// //Can print rate of loops, also can use to decrease number of serial prints to make them easier to read

void loopRate()
{
  long curTime = millis();
  unsigned long unitConversion = (AVERAGE_OVER * 1000);
  double lps = unitConversion / static_cast<long double>(curTime - tLastLPSCalc);
  Serial3.print("Loops per second: ");
  Serial3.println(lps);
  Serial3.print(analogRead(LMARKERSENSE));
  Serial3.print(" ");
  Serial3.print(analogRead(LEFTSENSE));
  Serial3.print(" ");
  Serial3.print(analogRead(MIDLEFTSENSE));
  Serial3.print(" ");
  Serial3.print(analogRead(MIDRIGHTSENSE));
  Serial3.print(" ");
  Serial3.print(analogRead(RIGHTSENSE));
  Serial3.print(" ");
  Serial3.print(analogRead(RMARKERSENSE));
  Serial3.print(" ");
  Serial3.print(previousState);
  Serial3.println();

  // Serial.print("CurrentTime: ");
  // Serial.println(curTime);
  // Serial.print("StartTime:  ");
  // Serial.println(tStart);
  tLastLPSCalc = millis();
}
