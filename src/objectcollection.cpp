#include <Arduino.h>
#include <Wire.h>
#include <objectcollection.h>

#define ELASTIPWM 2000//Powered at 12V so keep under ~3000
#define REVERSEPWM 450

volatile int elastiClicks = 0;
volatile int prevElastiClicks = 0;
bool stallState = 0;

int elastiSpeed = 0;
int prevTime = 0;
bool bombDetected = false;


void objCollectionInit(){
    stallState = 0;
    elastiClicks = 0;
    prevElastiClicks = 0;

    pinMode(PIPIN, INPUT_PULLDOWN);
    pinMode(ELASTIENCODER, INPUT);
   

}

void bombRoutine(){ //should be in bomb routine while bomb is being detected
    Serial3.println("Bomb Routine!");
    pwm_start(ELASTIFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(ELASTIREVERSE, 75, REVERSEPWM, RESOLUTION_12B_COMPARE_FORMAT);
}

void normalObjRoutine(){
    pwm_start(ELASTIREVERSE, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(ELASTIFORWARD, 75, ELASTIPWM, RESOLUTION_12B_COMPARE_FORMAT);
}

void stopElasti(){
    pwm_start(ELASTIREVERSE, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(ELASTIFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
}

void elastiEncoder(){
    elastiClicks++;
}


void checkStall(){

    // Serial3.println("Check Stall:");
    // Serial3.println(elastiClicks);
    // Serial3.println(prevElastiClicks);
    // Serial3.println(elastiClicks);
    // Serial3.println(stallState);
    
    if (stallState == 1) {
        normalObjRoutine();
        stallState = 0;
    }
    else {
        if (elastiClicks - prevElastiClicks <= 5) {
            //Stall Detected
            pwm_start(ELASTIFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(ELASTIREVERSE, 75, 3000, RESOLUTION_12B_COMPARE_FORMAT);
            stallState = 1;
        }
    }
    prevElastiClicks = elastiClicks;

}

void checkBomb() {

    if(digitalRead(PIPIN) == HIGH) {
        bombDetected = true;
    }
    else {
        bombDetected = false;
    }
}


