#ifndef OBJECT_COLLECTION
#define OBJECT_COLLECTION

#define PIPIN PB5 //not sure if this is worth having as a header file or not (interupts with headers?)
#define ELASTIFORWARD PA_0
#define ELASTIREVERSE PA_1
#define ELASTIENCODER PA3

#define BOMB_ROUTINE_MS 3000
#define NORMAL_SPEED 2000

volatile extern int elastiClicks;
extern bool bombDetected;



/*
Initialize the raspberry pi pin as an interrupt*/
void objCollectionInit();

/*Bomb detection routine on interrupt*/
void bombRoutine();

/*Normal state for object collection*/
void normalObjRoutine();

/*Stops Elasti grab motor*/
void stopElasti();

/*Tracks elasti clicks*/
void elastiEncoder();

/*checks for a stall*/
void checkStall();

/*Checks for Bomb*/
void checkBomb();
#endif