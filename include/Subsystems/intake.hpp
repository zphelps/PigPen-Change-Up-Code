
extern pros::ADIDigitalIn intakeLimit;

//Currently Used
void frontRollers(int speed);
void intake(int speed);
void indexer(int speed);
void intakeFullReverse();
void loadOneBallAndReverse(void *parameter);

//Not Sure
void loadOneBall(void *parameter);
void scoreOneBall();
void scoreOneBallWithWait();
void scoreBalls(int time);
void topBallSwitch(int timeout);
void intakeFullStop();
void intakeOP();
void countThreeBlue();
void oneRedOneBlue();
void fullIntake(int speed);
void cycleBlueRed();
void scoreOneBallWithFrontRollers();
void intakeTests();

void oneBlueCycleOneRed();
void twoBlueCycleTwoRed();
void twoBlueCycleTwoRedAuton();