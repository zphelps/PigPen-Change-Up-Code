
extern pros::ADIDigitalIn intakeLimit;
extern pros::ADIDigitalIn indexerLimit;
extern pros::ADILineSensor frontRollerLine;
extern pros::ADILineSensor topRollerLine;

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
void scoreOneBall();
void scoreOneBall(int timeout);
void scoreOneBallInCenterGoal();
void scoreOneBallWithWait();
void scoreBalls(int time);
void intakeFullStop();
void intakeOP();
void fullIntake(int speed);
void scoreOneBallWithFrontRollers();

void oneBlueCycleOneRed();
void twoBlueCycleTwoRed();
void twoBlueCycleTwoRedAuton();

void intakeManager(void *parameter);

void setIntakeMode(int mode);