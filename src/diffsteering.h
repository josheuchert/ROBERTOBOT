
#ifndef DIFF_STEERING
#define DIFF_STEERING

#define LEFTSENSE PA6
#define MIDLEFTSENSE PA7
#define MIDRIGHTSENSE PB0
#define RIGHTSENSE PB1     
#define LMOTORFORWARD PB_9
#define LMOTORBACK PB_8
#define RMOTORFORWARD PB_7
#define RMOTORBACK PB_6
#define LMARKERSENSE PA4
#define RMARKERSENSE PA5
#define TAPE_THRESHOLD 650

extern bool topOfRamp;

/*Assigns pins to their required states for the steering code to work (Use only in setup) */
void drivePinInit();
/*Returns an error state based on the readings on the tape sensors (and previous state if necessary)*/
int getErrorState(int previousState);
/*Outputs a steering value from -2*duty cycle to 2*duty cycle that represents the steering angle desired
depends on values of P and D in PID (I term not implemented yet)*/
int getSteeringVal(int currentErrorState, double previousErrorState);
/*starts pwm for each wheel based on the steering value
*/
void startDriveMotors(int steeringVal);

/*Stops motors*/
void stopDriveMotors();

#endif
