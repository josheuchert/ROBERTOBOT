
#ifndef DIFF_STEERING
#define DIFF_STEERING

#define LEFTSENSE PA7
#define MIDLEFTSENSE PA6
#define MIDRIGHTSENSE PB1
#define RIGHTSENSE PB0
#define LMOTORFORWARD PB_6
#define LMOTORBACK PB_7
#define RMOTORFORWARD PB_9
#define RMOTORBACK PB_8
#define LMARKERSENSE PA4
#define RMARKERSENSE PA3

extern bool bottomRampGoingUp;
extern bool bottomRampGoingDown;
extern bool topOfRamp;

/*Assigns pins to their required states for the steering code to work (Use only in setup) */
void drivePinInit();
/*Returns an error state based on the readings on the tape sensors (and previous state if necessary)*/
int getErrorState(int previousState);
/*Outputs a steering value from -2*duty cycle to 2*duty cycle that represents the steering angle desired
depends on values of P and D in PID (I term not implemented yet)*/
int getSteeringVal(int currentErrorState, int previousErrorState);
/*starts pwm for each wheel based on the steering value
*/
void startDriveMotors(int steeringVal);


#endif
