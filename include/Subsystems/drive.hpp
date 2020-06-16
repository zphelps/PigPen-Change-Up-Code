#include "main.h"

extern pros::Motor frontRight;
extern pros::Motor backLeft;
extern pros::Motor frontLeft;
extern pros::Motor backLeft;

extern pros::Imu inertial;

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

void stopDrive();

void brake();
void coast();

void timedDrive(int time, int speed);

void driveOP();

void xdriveOP();

void checkInertial(int expectedAngle);

void initializeInertialSensor();

void move(int distance, int heading, int accelStep, bool fluid = false);

void move(int distance, int heading, int accelStep, double kP, int minSpeed, double correction);

void moveFluid(int distance, int heading, int accelStep);

void moveToYCoord(int distance, int heading, int accelStep, bool fluid);

void moveToYCoord(int distance, int heading, int accelStep, double kP, int minSpeed, double correction);

void moveToYCoordFluid(int distance, int heading, int accelStep);

void moveAboslute(double x, double y, int accelStep);

void moveBack(int distance, int heading, int accelStep, bool fluid = false);

void moveBack(int distance, int heading, int accelStep, double kP, int minSpeed, double correction);

void moveBackFluid(int distance, int heading, int accelStep);

void moveBackToYCoord(int distance, int heading, int accelStep, bool fluid);

void moveBackToYCoord(int distance, int heading, int accelStep, double kP, int minSpeed, double correction);

void moveBackToYCoordFluid(int distance, int heading, int accelStep);

void turn(int degrees);

void turn(int degrees, int kP, double minSpeed);

void correct(int degrees, int minSpeed);

void turnInertial(int degrees);

void sweepRight(int degrees, int rightSideSpeed);

void sweepRight(int degrees, int rightSideSpeed, int errorThreshhold);

void sweepRight(int degrees, int rightSideSpeed, double kP, int minspeed);

void sweepRight(int degrees, int rightSideSpeed, int errorThreshhold, double kP, int minSpeed);

void sweepLeft(int degrees, int leftSideSpeed);

void sweepLeft(int degrees, int leftSideSpeed, int errorThreshhold);

void sweepLeft(int degrees, int leftSideSpeed, int errorThreshhold, double kP, int minSpeed);

void sweepRightBack(int degrees, int leftSideSpeed);

void sweepRightBack(int degrees, int leftSideSpeed, int errorThreshhold);

void sweepRightBack(int degrees, int leftSideSpeed, int errorThreshhold, double kP, int minSpeed);

void sweepLeftBack(int degrees, int rightSideSpeed, double kP, int minSpeed);

void sweepLeftBack(int degrees, int rightSideSpeed);

void sweepLeftBack(int degrees, int rightSideSpeed, int errorThreshhold);

void sweepLeftBack(int degrees, int rightSideSpeed, int errorThreshhold, double kP, int minSpeed);
