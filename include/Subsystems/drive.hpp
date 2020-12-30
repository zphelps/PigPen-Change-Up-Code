
//Motors
extern pros::Motor frontRight;
extern pros::Motor backLeft;
extern pros::Motor frontLeft;
extern pros::Motor backLeft;

//Line Sensors
extern pros::ADILineSensor rightDriveLine;
extern pros::ADILineSensor leftDriveLine;

//Externalize line sensor constant
extern int LINE_DETECTED;

#define RIGHT_SIDE 0;
#define LEFT_SIDE 1;
#define BOTH_SIDES 2;

#define MOVE_FOR_DISTANCE 0
#define MOVE_TO_X_COORD 1
#define MOVE_BACK_TO_X_COORD 2
#define MOVE_TO_Y_COORD 3
#define MOVE_BACK_TO_Y_COORD 4

#define TURN 0
#define SWEEP_RIGHT 1
#define SWEEP_RIGHT_WITH_THRESHHOLD 2
#define SWEEP_LEFT 3
#define SWEEP_LEFT_WITH_THRESHHOLD 4
#define SWEEP_RIGHT_BACK 5
#define SWEEP_RIGHT_BACK_WITH_THRESHHOLD 6
#define SWEEP_LEFT_BACK 7
#define SWEEP_LEFT_BACK_WITH_THRESHHOLD 8

#define NORMAL_DRIVE 0
#define SLOW_DRIVE 1

/**
 * @brief Move targets structure (see moveTask())
 * 
 */
struct MoveTargets
{
    int targetDistance;
    int targetHeading;
    int accelStep;
    bool fluid;
    int moveType;
};

/**
 * @brief Turn targets structure (see turnTask())
 * 
 */
struct TurnTargets
{
    int degrees;
    int leftSideSpeed;
    int rightSideSpeed;
    int errorThreshhold;
    int turnType;
};

extern MoveTargets moveTargets;
extern TurnTargets turnTargets;

/**
 * @brief Drive Class Header
 * 
 */
class Drive
{

public:
    static void left(int l);
    static void right(int r);
    static void slewRight(int speed, int accelStep);
    static void slewLeft(int speed, int accelStep);
    static void slewRightBack(int speed, int accelStep);
    static void slewLeftBack(int speed, int accelStep);
    static void drivePower(int l, int r);
    static void timedDrive(int time, int l, int r);
    static void untilLineDetected(int speed, int timeout = 2000);
    static void brake();
    static void coast();
    static void driveOP(int driveMode = 0);

    static void moveStartTask();
    static void moveStopTask();

    static void turnStartTask();
    static void turnStopTask();

    Drive &withCorrection(double cM);
    Drive &withGains(double kP, double kI, double kD, int minSpeed);
    Drive &withTurnGains(double kP, double kI, double kD, int minSpeed);

    //static void moveHeadingCorrection(int heading, double correctionMultiplier, double PIDSpeed, int accelStep, bool backward);
    static void moveHeadingCorrection(int heading, double correctionMultiplier, double PIDSpeed, int accelStep, bool backward, bool vision = false);

    Drive &move(int distance, int heading, int accelStep, bool async = false, bool fluid = false);
    Drive &moveToYCoord(int distance, int heading, int accelStep, bool async = false, bool fluid = false);
    Drive &moveToXCoord(int distance, int heading, int accelStep, bool async = false, bool fluid = false);
    Drive &moveBackToYCoord(int distance, int heading, int accelStep, bool async = false, bool fluid = false);
    Drive &moveBackToXCoord(int distance, int heading, int accelStep, bool async = false, bool fluid = false);

    Drive &turn(int degrees);
    Drive &sweepRight(int degrees, int rightSideSpeed);
    Drive &sweepRight(int degrees, int rightSideSpeed, int errorThreshhold);
    Drive &sweepLeft(int degrees, int leftSideSpeed);
    Drive &sweepLeft(int degrees, int leftSideSpeed, int errorThreshhold);
    Drive &sweepRightBack(int degrees, int leftSideSpeed);
    Drive &sweepRightBack(int degrees, int rightSideSpeed, int errorThreshhold);
    Drive &sweepLeftBack(int degrees, int rightSideSpeed);
    Drive &sweepLeftBack(int degrees, int leftSideSpeed, int errorThreshhold);

    static void moveTask(void *parameter);
    static void turnTask(void *parameter);
    void waitForComplete();
};

extern Drive drive;