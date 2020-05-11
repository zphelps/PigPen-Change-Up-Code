#include "main.h"

extern pros::Motor frontRight;
extern pros::Motor backLeft;
extern pros::Motor frontLeft;
extern pros::Motor backLeft;

extern double MOVE_KP;
extern double MOVE_KI;
extern double MOVE_KD;
extern double MOVE_ACCEL_STEP;
extern int MOVE_MIN_SPEED;
extern int START_SPEED;

extern double TURN_KP;
extern double TURN_KI;
extern double TURN_KD;
extern int POINT_TURN_MIN_SPEED;

extern const double WHEEL_DIAMETER;
extern const int TICS_PER_REVOLUTION;
extern const double LEFT_OFFSET;
extern const double RIGHT_OFFSET;
extern const double REAR_OFFSET;

void left(int speed);
void right(int speed);

void brake();
void coast();

void timedDrive(int time, int speed);

void driveOP();

void xDriveOP();

void initializeInertialSensor();

void move(int distance, int heading, bool fluid = false);

void moveUltra(int distance, int heading, bool fluid = false);

void moveBack(int distance, int heading, bool fluid = false);

void turn(int degrees);

void correct(int degrees);

void turnInertial(int degrees);

void sweepRight(int degrees, int rightSideSpeed);

void sweepRight(int degrees, int rightSideSpeed, int errorThreshhold);

void sweepLeft(int degrees, int leftSideSpeed);

void sweepLeft(int degrees, int leftSideSpeed, int errorThreshhold);
