
extern pros::ADIDigitalIn intakeLimit;
extern pros::ADIDigitalIn indexerLimit;
extern pros::ADILineSensor frontRollerLine;
extern pros::ADILineSensor topRollerLine;

extern pros::Vision vision2;
extern pros::Vision vision;

//Vision Methods
void descoreBottomBall(int timeout = 2000);

//Currently Used
void frontRollers(int speed);
void intake(int speed);
void indexer(int speed);
void intakeFullReverse();

void initVision();
void ejectBalls(void *parameter);
void ejectOneBall(void *parameter);

//Not Sure
void loadOneBall(void *parameter);
void scoreOneBall(int timeout = 2000);
void scoreOneBallWithVision(int timeout = 2000);
void scoreOneBallInCenterGoal();
void scoreBalls(int time);
void intakeFullStop();
void intakeOP();
void fullIntake(int speed);
void scoreOneBallWithFrontRollers();

void oneBlueCycleOneRed();
void twoBlueCycleTwoRed();
void twoBlueCycleTwoRedAuton();

void intakeManager(void *parameter);
