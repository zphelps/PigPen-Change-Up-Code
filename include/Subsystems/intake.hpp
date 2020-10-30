
extern pros::ADIDigitalIn intakeLimit;
extern pros::ADIDigitalIn indexerLimit;
extern pros::ADILineSensor frontRollerLine;
extern pros::ADILineSensor topRollerLine;

//Vision Sensor
#define BLUE_ID 1
#define RED_ID 2
#define ANY_COLOR 3

extern pros::Vision vision2;
extern pros::Vision vision;

#define ONE_BALL 0
#define TWO_BALLS 1
#define THREE_BALLS 2

#define FRONT_ROLLERS 0
#define INTAKE 1
#define INDEXER 2

void initVision();

bool topBallDetected(int color);
bool intakeBallDetected(int color);

void visionTest();

extern int BALL_DETECTED_SIGNATURE;

class Intake
{
public:
    void frontRollers(int speed, int time = 0);
    void intakeRollers(int speed, int time = 0);
    void indexer(int speed, int time = 0);
    void intake(int FRSpeed, int IRSpeed, int ISpeed);
    void stop();

    void brake(int subsystem);
    void coast(int subsystem);

    void logic();

    void manager(void *parameter);

    void reverseFor(int timeout, bool auton = false);

    void intakeOP();

    void score(int numBalls, int timeout = 2000);
    void scoreWithVision(int timeout = 2000);

    void twoBlueCycleThreeRed();
    void twoBlueCycleTwoRed();
    void twoBlueCycleOneRed();

    void oneBlueCycleOneRed();
    void oneBlueCycleTwoRed();
};

extern Intake intake;
