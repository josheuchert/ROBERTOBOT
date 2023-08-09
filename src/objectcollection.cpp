#include <Arduino.h>
#include <Wire.h>
#include <objectcollection.h>

#define ELASTIPWM 2000//Powered at 12V so keep under ~3000
#define REVERSEPWM 500

volatile int elastiClicks = 0;
volatile int prevElastiClicks = 0;
bool stallState = 0;
#define ELASTIPWM 3000//Powered at 12V so keep under ~3000
#define ELASTIPWM 2000//Powered at 12V so keep under ~3000
#define REVERSEPWM 500

volatile int elastiClicks = 0;
volatile int prevElastiClicks = 0;
bool stallState = 0;

int elastiSpeed = 0;
int prevTime = 0;
int elastiClicks = 0;
int prevElastiClicks = 0;
bool stallState = 0;


void objCollectionInit(){
    stallState = 0;
    elastiClicks = 0;
    prevElastiClicks = 0;

    pinMode(PIPIN, INPUT_PULLDOWN);
    pinMode(ELASTIENCODER, INPUT);
   

}

void bombRoutine(){ //should be in bomb routine while bomb is being detected

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
    if (stallState == 1) {
        normalObjRoutine();
        stallState = 0;
    }
    else {
        if (elastiClicks - prevElastiClicks <= 5) {
            //Stall Detected
            pwm_start(ELASTIFORWARD, 75, 0, RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(ELASTIREVERSE, 75, ELASTIFORWARD, RESOLUTION_12B_COMPARE_FORMAT);
            stallState = 1;
        }
    } 
}


