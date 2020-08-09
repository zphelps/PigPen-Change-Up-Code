#include "main.h"

std::shared_ptr<ChassisController> okapiChassis = ChassisControllerBuilder()
                                                      .withMotors({9, 2}, {5, 1})
                                                      .withGains(
                                                          {0.01, 0, 0},
                                                          {0.1, 0, 0})
                                                      .withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 12.5_in}, imev5BlueTPR *(36 / 60)})
                                                      .build();

std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
        .withLimits({
            1.0, //Maximum linear velocity of the chassis
            1.0, //Maximum linear acceleration
            5.0  //Maximum linear jerk of the chassis
        })
        .withOutput(okapiChassis)
        .buildMotionProfileController();

void test_sTurn()
{
    profileController->generatePath({{0_ft, 0_ft, 0_deg},
                                     {2_ft, 2_ft, 0_deg}},
                                    "A");
    profileController->setTarget("A");
    profileController->waitUntilSettled();
}

/**
 * @brief 
 * 
 * @param speed 
 */
void testMinSpeed(int speed)
{
    left(speed);
    right(speed);
    wait(10000);
}

void twoGoalAutonRight()
{
    //turn(90, TURN_KP, 50);
    //move(15, 0, 2, true);
    sweepRight(130, 0);
    frontRollers(127);
    //fullIntake(127);
    //pros::Task load1(loadIntake);
    //fullIntake(127);
    move(8, 130, 2);
    timedDrive(300, 30);
    intakeFullStop();
    scoreOneBall();
    frontRollers(127);
    wait(200);
    intakeFullStop();
    oneRedOneBlue();

    intakeFullReverse();
    wait(500);
    //moveBack(20, 130, 3);
    //intakeFullStop();
    // sweepLeftBack(90, 0, 3.5, 30);
    // sweepLeftBack(90, 0, 15);
    // moveBack(51, 90, 0);
    // sweepRight(180, 0, 3.0, 30);
    moveBack(68, 90, 1, MOVE_KP, 20, 0.25);
    turn(180);
    move(15, 180, 0, true);
    //timedDrive(300, 30);
    //scoreOneBall();
    cycleBlueRed();
    moveBack(10, 180, 2);
    /*
    turn(-90);
    move(30, -90, 2);
    turn(-145, 2.5, MOVE_MIN_SPEED);
    pros::Task load3(loadIntake);
    move(41, -145, 2);
    turn(-140);
    timedDrive(500, 30);
    wait(2000);
    intakeFullStop();
    //move(15, 130, 0);
    // correct(20, 20);*/
}

void twoGoalAutonLeft()
{
    //turn(90, TURN_KP, 50);
    //move(15, 0, 2, true);
    sweepLeft(-130, 0);
    //fullIntake(127);
    //pros::Task load1(loadIntake);
    //fullIntake(127);
    frontRollers(127);
    move(8, -130, 2);
    timedDrive(300, 30);
    intakeFullStop();
    scoreOneBall();
    frontRollers(127);
    wait(200);
    intakeFullStop();
    oneRedOneBlue();

    intakeFullReverse();
    wait(500);
    //moveBack(20, 130, 3);
    //intakeFullStop();
    // sweepLeftBack(90, 0, 3.5, 30);
    // sweepLeftBack(90, 0, 15);
    // moveBack(51, 90, 0);
    // sweepRight(180, 0, 3.0, 30);
    moveBack(60, -90, 1, MOVE_KP, 20, 0.25);
    turn(-180);
    move(15, -180, 0, true);
    //timedDrive(300, 30);
    //scoreOneBall();
    cycleBlueRed();
    moveBack(10, -180, 2);
    /*
    turn(-90);
    move(30, -90, 2);
    turn(-145, 2.5, MOVE_MIN_SPEED);
    pros::Task load3(loadIntake);
    move(41, -145, 2);
    turn(-140);
    timedDrive(500, 30);
    wait(2000);
    intakeFullStop();
    //move(15, 130, 0);
    // correct(20, 20);*/
}

void home_row_right()
{
    setCoordinates(0, 24, 0);
    sweepRight(135, 0);
    pros::Task task1(loadOneBallAndReverse);
    move(8, 135, 0);
    timedDrive(300, 30);
    scoreOneBall();

    fullIntake(127);
    moveBack(10, 135, 0, true);
    sweepLeftBack(90, 0, 15, 5, 100);
    moveBackToYCoord(56, 90, 0, 6, 30, 0.8);
    //58

    turn(180);
    intakeFullStop();
    move(8, 180, 0, 0.3, 40, 0.2);
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, 180, 0, 0.25, 35, 0.8);
    turn(270);
    move(16, 270, 2, true);
    //14
    sweepLeft(225, 0, 10, 2, 100);
    pros::Task task3(loadOneBallAndReverse);
    move(10, 225, 0, true);
    timedDrive(500, 40);
    scoreOneBall();
    intakeFullReverse();
    moveBack(24, 225, 1);
}

void home_row_left()
{
    setCoordinates(0, 125, 0);
    sweepLeft(-135, 0);
    pros::Task task1(loadOneBallAndReverse);
    move(8, -135, 0);
    timedDrive(300, 30);
    scoreOneBall();

    fullIntake(127);
    moveBack(10, -135, 0, true);
    //wait(100000);
    sweepRightBack(-90, 0, 20, 5, 100);
    moveBackToYCoord(90, -90, 0, 6, 30, 0.8);

    turn(-180);
    intakeFullStop();
    move(8, -180, 0, 0.3, 40, 0.2);
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, -180, 0, 0.25, 35, 0.8);
    turn(-270);
    move(16, -270, 2, true);
    sweepRight(-225, 0, 10, 2, 100);
    pros::Task task3(loadOneBallAndReverse);
    move(10, -225, 0, true);
    timedDrive(500, 40);
    scoreOneBall();
    intakeFullReverse();
    moveBack(24, -225, 1);
}

void programming_skills()
{
    setCoordinates(0, 24, 0);
    pros::Task task1(loadOneBall);
    move(5, 0, 0, 0.2, 40, 0.8);
    sweepRight(135, -5);
    move(10, 135, 0, true);
    timedDrive(200, 35);
    intakeFullStop();
    scoreOneBall();
    wait(100);
    fullIntake(127);
    indexer(-127);

    moveBack(12, 135, 0, true);
    intakeFullStop();
    sweepLeftBack(90, 0, 2.75, 40);
    pros::Task load1(loadOneBall);
    move(24, 90, 2);
    timedDrive(500, 35);
    moveBack(12, 90, 1);
    //sweepLeft(-90, -5);
    turn(-45, 1, 20);

    frontRollers(127);
    move(5, -45, 1, true);
    sweepLeft(-90, 0, 10);
    moveToYCoord(56, -90, 0, 4, 25, 0.4);

    turn(-180);
    move(26, -180, 0, true);
    intakeFullStop();
    timedDrive(300, 40);
    scoreBalls(500);

    moveBack(6, -180, 0, MOVE_KP + 0.05, 25, 0.5);
    fullIntake(127);
    indexer(-127);
    turn(-90);
    pros::Task load3(loadOneBall);
    move(40, -90, 1);
    sweepLeft(-135, 0, 10);
    move(8, -135, 0);
    intakeFullStop();
    timedDrive(200, 35);
    scoreOneBall();
    wait(100);
    fullIntake(127);
    indexer(-127);

    moveBack(12, -135, 0, true);
    intakeFullStop();
    sweepRightBack(-90, 0, 2.75, 40);
    pros::Task loadBall(loadOneBall);
    move(24, -90, 2);
    timedDrive(500, 35);
    moveBack(12, -90, 1);

    wait(1000000);

    sweepLeftBack(90, 1);
    timedDrive(500, -50);
    wait(200);
    setTheta(90);
    pros::Task load4(loadOneBall);

    move(2, 90, 1);
    sweepLeft(0, 1, 10);
    move(29, 0, 0, 0.15, 30, 0.6);
    turn(-90);
    intakeFullStop();
    timedDrive(350, 30);
    scoreOneBall();
    moveBack(8, -90, 1);
    turn(0);
    pros::Task load5(loadOneBall);

    move(38, 15, 2, MOVE_KP, 20, 0.3); //0.3
    turn(-45, 1.5, 25);
    move(26, -45, 0);
    timedDrive(300, 30);
    topBallSwitch(1000);
    intakeFullReverse();

    moveBack(32, 0, 1, MOVE_KP, 20, 0.3);
    wait(300);
    turn(90);
    moveBack(14, 90, 1);
    timedDrive(750, -35);
    setTheta(90);
    wait(300);
    pros::Task load6(loadOneBall);
    move(58, 90, 1);
    turn(0);
    move(24, 0, 1, true);
    intakeFullStop();
    timedDrive(450, 40);
    topBallSwitch(1500);
    intakeFullReverse();

    moveBack(28, 0, 0, 1.35, 40, 0.2);
    wait(300);
    turn(90);
    move(24, 90, 0);
    pros::Task load7(loadOneBall);
    turn(45, 1.5, 25);
    move(42, 40, 2, true);
    intakeFullStop();
    timedDrive(750, 40);
    topBallSwitch(2500);
    intakeFullReverse();

    sweepLeftBack(-90, 1);
    moveBack(3, -90, 0);
    timedDrive(600, -40);
    setTheta(-90);
    wait(200);

    move(3, -90, 1);
    sweepLeft(-180, 1, 10);
    move(25, -180, 0, 0.1, 25, 0.6);
    turn(-270);
    intakeFullStop();
    timedDrive(350, 30);
    scoreOneBall();
    moveBack(8, -270, 1);

    turn(-90, 1.15, POINT_TURN_MIN_SPEED);
}

void programming_skills_new()
{
    setCoordinates(1, 24, 0);
    frontRollers(127);
    intake(127);
    indexer(-127);
    move(8, 0, 0, 0.2, 40, 0.8);
    //sweepRight(90, 0, 1.35, 30);
    sweepRight(90, 0, 10, 1.35, 35);
    move(15, 90, 0, true);
    timedDrive(250, 35); //350
    moveBack(15, 90, 0, true);
    sweepRightBack(135, 5, 1.6, 35);
    move(20, 135, 0, 0.25, 35, 0.2);
    timedDrive(200, 35);
    frontRollers(0);
    twoBlueCycleTwoRedAuton();
    //scoreOneBall();

    moveBackToYCoord(10, 135, 0, 8.5, MOVE_MIN_SPEED, 0.2);

    frontRollers(127);
    intake(127);
    indexer(-127);
    turn(0, 1.25, 20); //1.15 25
    moveToXCoord(60, 0, 1, 4, 30, 0.2);
    turn(90);
    frontRollers(0);
    //move(5, 90, 0, 0.25, 35, 0.2);
    timedDrive(100, 30); //200
    scoreOneBall();
    //wait(200);
    scoreOneBall();
    moveBack(10, 90, 0);

    frontRollers(127);
    intake(127);
    indexer(-127);
    turn(0);
    moveToXCoord(78, 0, 0, 8, 45, 0.2);
    sweepRight(90, 0, 1.2, 35);
    timedDrive(800, 35); //800
    wait(50);            //250
    if (getTheta() > 86 && getTheta() < 94)
    {
        setTheta(90);
    }
    setCoordinates(getX(), -3, getTheta());
    moveBackToYCoord(5, 90, 0);
    turn(5, 1.25, 20);
    //move(1, 5, 0, true);
    sweepRight(45, 35, 5, 2.75, 50);
    frontRollers(0);
    timedDrive(100, 35); //500
    scoreOneBall();
    sweepLeftBack(0, 0, 10, 5, 90);
    moveBackToXCoord(88, 0, 0, 6, 30, 0.5);

    indexer(-127);
    intake(127);
    frontRollers(127);
    turn(-90);
    moveToYCoord(56, -90, 1, 4.5, 30, 0.2);
    //move(48, -90, 1, 0.1, 30, 0.2);
    turn(0);
    move(24, 0, 0);
    frontRollers(0);
    timedDrive(100, 30); //300
    scoreOneBall();
    intake(127);
    //wait(200);
    scoreOneBall();
    setCoordinates(132, getY(), getTheta());

    frontRollers(-127);
    moveBack(8, 0, 0);
    turn(-90);
    intake(127);
    indexer(-127);
    frontRollers(127);
    move(28, -90, 1, true);
    sweepRight(-45, 0, 10, 2, 75);
    frontRollers(0);
    //move(2, -45, 0, true);
    timedDrive(250, 30);
    scoreOneBall();
    wait(200);
    frontRollers(-127);
    moveBack(28, -45, 1);
    intakeFullStop();

    turn(-90, 2.15, 15); //2, 20
    intake(127);
    indexer(-127);
    frontRollers(127);
    move(16, -90, 0);
    timedDrive(300, 30);
    wait(250);
    frontRollers(0);
    timedDrive(300, 30);
    wait(250);
    setTheta(-90);
    wait(50);

    frontRollers(127);
    moveBack(35, -90, 2); //36
    turn(-180);
    move(32, -180, 0);
    turn(-90);

    timedDrive(1750, -40);
    move(8, -90, 0);
    timedDrive(1250, -40);

    move(32, -90, 0);
    frontRollers(0);
    timedDrive(300, 40);
    scoreOneBall();
    wait(200);
    scoreOneBall();
    moveBack(22, -88, 0, 0.1, 30, 0.3);
    frontRollers(127);

    turn(90, 1.35, 15);
    move(14, 90, 1);
    intakeFullReverse();
    wait(1000);
    timedDrive(500, 35);
    moveBack(22, 90, 1);

    turn(180);
    frontRollers(127);
    move(46, 180, 1);
    sweepRightBack(232, 0, 1.5, 35);
    move(24, 232, 0);
    timedDrive(300, 30);
    scoreOneBall();
    moveBack(30, 232, 0);

    sweepLeftBack(180, 0, 10, 5, 90);
    moveBackToXCoord(48, 180, 0, 6, 30, 0.5);

    indexer(-127);
    intake(127);
    frontRollers(127);
    turn(90);
    moveToYCoord(60, 90, 2, 4.5, 30, 0.2);
    //move(48, -90, 1, 0.1, 30, 0.2);
    turn(180);
    move(24, 180, 1);
    frontRollers(0);
    timedDrive(300, 30);
    scoreOneBall();
    intake(127);
    wait(200);
}

void autonomous()
{
    programming_skills_new();

    //twoGoalAutonLeft();
    // switch (autonIndex)
    // {
    // case 0:
    //     home_row_right();
    //     break;
    // case 1:
    //     home_row_left();
    //     break;
    // case 2:
    //     twoGoalAutonRight();
    //     break;
    // case 3:
    //     //Red Back
    //     twoGoalAutonLeft();
    //     break;
    // case 4:
    //     programming_skills_new();
    //     break;
    // }
}
