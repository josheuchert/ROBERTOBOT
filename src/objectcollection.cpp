#include <Arduino.h>
#include <Wire.h>
#include <objectcollection.h>

#define ELASTIPWM 3000//Powered at 12V so keep under ~3000
#define REVERSEPWM 500



void objCollectionInit(){
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


