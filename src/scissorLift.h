#ifndef SCISSOR_LIFT
#define SCISSOR_LIFT

#define SCISSOR_MOTOR_UP PA_10
#define SCISSOR_MOTOR_DOWN PA_9
#define LIMIT_SWITCH_EXT PA12
#define LIMIT_SWITCH_RET PA15
#define SL_ENCODER PA11
#define TRIG_PIN PB3
#define ECHO_PIN PB4
#define GO_PIN PB14
#define SL_EXTEND_SPEED 3000
#define SL_RETRACT_SPEED 3000
#define MOUNTPOSITION 2400
#define DRIVE_PWM_FOR_MOUNTING 2000
#define SONAR_CLIFF_HEIGHT 7
#define SONAR_GROUND 3
#define DOWN_RAMP PB12
#define UP_RAMP PB13

 //0 when scissorlift is fully retracted
extern int encoderPosition;
extern bool go;
extern bool extending;
extern int lapCount;
extern int rampState;
//Initialize switches and encoder pins
void initSL();

//Called when scissorlift reaches maximum height
void ext_limit_handler();

//Called when scissorlift reaches minimum height
void ret_limit_handler();

//Increments encoder position/count
void encoder_handler();

//starts race
void go_handler();

//Raises scissorlift
void extend();

//Lowers scissorlift
void retract();

//Raises scissorlift to mounting position
void readyMount();

//resets SL to rest position and calibrates position tracking
void calibrateSL();

//Returns distanceCM of ultrasonic sensor from floor
long getDistanceFromFloor();

//Stops scissor lift
void stopScissor();

//Driving at the top of the ramp to mount
void mountingDrivingRoutine();

//Driving off the zipline and retracting SL
void dismountDrivingRoutine();







#endif