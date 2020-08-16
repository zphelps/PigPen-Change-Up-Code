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
    setCoordinates(0, 24, 0);
    sweepRight(135, 0, 1.5, 25);
    frontRollers(127);
    move(8, 135, 0, 0.2, 35, 0.2);
    timedDrive(300, 35);
    // scoreOneBall();
    // intake(90);
    // frontRollers(-127);
    scoreOneBall();
    wait(200);
    //frontRollers(0);
    scoreOneBall();
    intake(-127);
    frontRollers(-127);

    moveBack(8, 135, 0, true); //10
    intake(127);
    sweepLeftBack(90, 0, 15, 5, 100);
    moveBackToYCoord(56, 90, 0, 6, 30, 0.8);

    turn(180);
    intakeFullStop();
    frontRollers(127);
    move(10, 180, 0, 0.3, 40, 0.2); //8
    timedDrive(500, 40);
    scoreOneBall();
    wait(250);
    scoreOneBall();
    moveBack(14, 180, 2, 0.15, 30, 0.2);
    intakeFullReverse();
}

void twoGoalAutonLeft()
{
    setCoordinates(0, 125, 0);
    sweepLeft(-135, 0);
    frontRollers(127);
    move(8, -135, 0);
    timedDrive(300, 30);
    frontRollers(0);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(10, -135, 0, true);
    fullIntake(127);
    sweepRightBack(-90, 0, 20, 5, 100);
    moveBackToYCoord(90, -90, 0, 6, 30, 0.8);

    turn(-180);
    intakeFullStop();
    move(10, -180, 0, 0.3, 40, 0.2); //8
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, -180, 0, 0.25, 35, 0.8);
}

void home_row_right_cycle()
{
    setCoordinates(0, 24, 0);
    sweepRight(135, 0, 1.5, 25);
    frontRollers(127);
    move(8, 135, 0, 0.2, 35, 0.2);
    timedDrive(100, 35);
    frontRollers(0);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(8, 135, 0, true); //10
    fullIntake(127);
    sweepLeftBack(90, 0, 15, 5, 100);
    moveBackToYCoord(56, 90, 0, 6, 30, 0.8);
    //58

    // turn(180);
    // intakeFullStop();
    // move(10, 180, 0, 0.3, 40, 0.2); //8
    // timedDrive(300, 30);
    // scoreOneBall();
    // intakeFullReverse();

    turn(180);
    intakeFullStop();
    frontRollers(127);
    move(10, 180, 0, 0.3, 40, 0.2); //8
    timedDrive(500, 35);
    scoreOneBall();
    wait(250);
    scoreOneBall();
    intakeFullReverse();
    // move(10, 180, 0, 0.3, 40, 0.2); //8
    // timedDrive(300, 30);
    // scoreOneBall();
    // intakeFullReverse();

    moveBack(14, 180, 2, 0.15, 30, 0.2);
    turn(270);
    move(23, 270, 1, true);
    sweepLeft(225, 30, 15, 1.75, 100);
    frontRollers(127);
    move(10, 225, 0, 0.2, 40, 0.5);
    timedDrive(500, 40);
    scoreOneBall();
    wait(200);
    //frontRollers(0);
    scoreOneBall();
    intakeFullReverse();
    moveBack(24, 225, 1);
}

void home_row_right_no_cycle()
{
    setCoordinates(0, 24, 0);
    sweepRight(135, 0);
    frontRollers(127);
    move(8, 135, 0);
    timedDrive(300, 30);
    frontRollers(0);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(8, 135, 0, true); //10
    fullIntake(127);
    sweepLeftBack(90, 0, 15, 5, 100);
    moveBackToYCoord(56, 90, 0, 6, 30, 0.8);
    //58

    // turn(180);
    // intakeFullStop();
    // move(10, 180, 0, 0.3, 40, 0.2); //8
    // timedDrive(300, 30);
    // scoreOneBall();
    // intakeFullReverse();

    turn(180);
    intakeFullStop();
    // frontRollers(127);
    // move(10, 180, 0, 0.3, 40, 0.2); //8
    // timedDrive(500, 40);
    // scoreOneBall();
    // wait(250);
    // scoreOneBall();
    // intakeFullReverse();
    move(10, 180, 0, 0.3, 40, 0.2); //8
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, 180, 2, 0.15, 30, 0.2);
    turn(270);
    move(23, 270, 1, true);
    sweepLeft(225, 30, 15, 1.75, 100);
    frontRollers(127);
    move(10, 225, 0, 0.2, 40, 0.5);
    timedDrive(500, 40);
    scoreOneBall();
    wait(200);
    scoreOneBall();
    intakeFullReverse();
    moveBack(24, 225, 1);
}

void home_row_left_no_cycle()
{
    setCoordinates(0, 125, 0);
    sweepLeft(-135, 0);
    // pros::Task task1(loadOneBallAndReverse);
    frontRollers(127);
    move(8, -135, 0);
    frontRollers(0);
    timedDrive(300, 30);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(8, -135, 0, true);
    fullIntake(127);
    //wait(100000);
    sweepRightBack(-90, 0, 20, 5, 100);
    moveBackToYCoord(92, -90, 0, 6, 30, 0.8);

    turn(-180);
    intakeFullStop();
    // frontRollers(127);
    move(10, -180, 0, 0.3, 40, 0.2); //8
    // move(8, -180, 0, 0.3, 40, 0.2);
    // timedDrive(500, 40);
    // scoreOneBall();
    // wait(350);
    // scoreOneBall();
    // intakeFullReverse();
    // move(8, -180, 0, 0.3, 40, 0.2);
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, -180, 0, 0.25, 35, 0.8);
    turn(-270);
    move(18, -270, 2, true);
    sweepRight(-225, 0, 10, 2, 100);
    //pros::Task task3(loadOneBallAndReverse);
    frontRollers(127);
    move(10, -225, 0, 0.2, 40, 0.5);
    timedDrive(500, 40);
    scoreOneBall();
    wait(200);
    scoreOneBall();
    intakeFullReverse();
    moveBack(24, -225, 1);
    // move(10, -225, 0, true);
    // timedDrive(500, 40);
    // scoreOneBall();
    // intakeFullReverse();
    // moveBack(24, -225, 1);
}

void programming_skills()
{
    setCoordinates(1, 24, 0);
    pros::Task intake_manager(intakeManager);
    frontRollers(127);
    intake(90);
    move(8, 0, 0, 0.2, 40, 0.8);
    sweepRight(90, 0, 10, 1.75, 25); //1.35, 35
    move(15, 90, 0, true);
    timedDrive(350, 35);       //350
    moveBack(16, 90, 1, true); //15
    frontRollers(0);
    sweepRightBack(135, 5, 1.45, 35);
    move(20, 135, 0, 0.4, 35, 0.2); //0.3
    timedDrive(300, 35);
    wait(200);
    scoreOneBall();

    moveBackToYCoord(10, 135, 1, 8.5, MOVE_MIN_SPEED, 0.2); //10

    intake_manager.resume();
    intake(90);
    frontRollers(127);
    turn(0, 1.25, 20);                  //1.15 25
    moveToXCoord(60, 0, 0, 5, 30, 0.2); //4, 30
    turn(90);
    frontRollers(0);
    move(6, 90, 0, 0.2, 35, 0.2);
    intake_manager.suspend();
    timedDrive(100, 30);
    scoreOneBall();
    wait(250);
    scoreOneBall();
    moveBack(10, 90, 0);

    intake_manager.resume();
    frontRollers(127);
    intake(90);
    turn(0);
    moveToXCoord(76, 0, 0, 10, 45, 0.2); //76, 0, 0,8
    sweepRight(90, 0, 1.3, 35);          //1.2
    timedDrive(1000, 35);                //800
    if (getTheta() > 86 && getTheta() < 94)
    {
        setTheta(90);
    }
    setCoordinates(getX(), -3, getTheta());
    moveBackToYCoord(5, 90, 0);
    turn(0, 1.4, 25); //5
    //move(1, 5, 0, true);
    sweepRight(45, 30, 5, 2.75, 50); //45, 35
    frontRollers(0);
    timedDrive(200, 35); //500
    scoreOneBall();
    sweepLeftBack(0, 0, 5, 5, 100);
    moveBackToXCoord(89, 0, 0, 10, 25, 0.1); //6, 30, 0.3

    intake_manager.resume();
    intake(127);
    frontRollers(127);
    turn(-90);
    intake(0);
    moveToYCoord(54, -90, 1, 4.5, 30, 0.2); //56
    turn(0);
    move(25, 0, 0);
    frontRollers(0);
    intake_manager.suspend();
    timedDrive(100, 30); //300
    wait(200);
    scoreOneBall();
    wait(250);
    scoreOneBall();
    setCoordinates(132, getY(), getTheta());

    frontRollers(-127);
    moveBack(8, 0, 2);
    turn(-90);
    intake_manager.resume();
    intake(127);
    frontRollers(127);
    move(24, -90, 1, true);
    sweepRight(-45, 0, 15, 2, 75);
    move(3, -25, 0);
    frontRollers(0);
    timedDrive(350, 35);
    scoreOneBall();
    wait(250);
    frontRollers(-127);
    moveBack(26, -45, 2); //26
    intakeFullStop();

    turn(-90, 2.15, 15); //2, 20
    intake_manager.resume();
    intake(90);
    frontRollers(127);
    move(16, -90, 0);
    timedDrive(300, 35);
    wait(250);
    frontRollers(0);
    timedDrive(250, 50);
    wait(250);
    setTheta(-90);

    frontRollers(127);
    moveBack(35, -90, 1); //36
    turn(-180);
    move(34, -180, 0); //32
    turn(-90);

    move(28, -90, 0);
    intake_manager.suspend();
    frontRollers(0);
    timedDrive(100, 30);
    scoreOneBall();
    wait(300);
    scoreOneBall();
    moveBack(22, -88, 2, 0.1, 30, 0.3);
    frontRollers(127);

    turn(90, 1.35, 15);
    move(14, 90, 1);
    left(100);
    right(100);
    intake(127);
    frontRollers(127);
    scoreOneBallInCenterGoal();
    indexer(-127);
    wait(350);

    frontRollers(-127);
    moveBack(28, 90, 1);

    indexer(-127);
    intake(127);
    turn(180);
    indexer(0);
    intake(127);
    frontRollers(127);
    move(56, 180, 1, 0.1, 35, 0.2);
    timedDrive(300, 35);
    wait(100);
    moveBack(16, 180, 0, true); //16
    frontRollers(0);
    sweepRightBack(225, 10, 1.45, 35); //1.75, 40
    move(24, 225, 0);                  //24
    timedDrive(250, 35);
    scoreOneBall();
    intakeFullReverse();
    moveBack(25, 225, 0);
    intakeFullStop();

    sweepLeftBack(180, 0, 10, 5, 90);
    moveBackToXCoord(46, 180, 0, 6, 30, 0.5);

    intake(127);
    frontRollers(127);
    turn(90);
    move(28, 90, 0, 0.1, 30, 0.2); //26
    turn(180);
    move(26, 180, 0, 0.3, 30, 0.2);
    frontRollers(0);
    timedDrive(250, 40);
    scoreOneBall();
    intake(127);
    wait(200);
}

void autonomous()
{
    // turn(90);
    // wait(250);
    // turn(180);
    // wait(250);
    // turn(270);
    // wait(250);
    // turn(360);
    //programming_skills();

    switch (autonIndex)
    {
    case 0:
        home_row_right_no_cycle();
        break;
    case 1:
        home_row_right_cycle();
        break;
    case 2:
        home_row_left_no_cycle();
        break;
    case 3:
        //home_row_left_cycle();
        break;
    case 4:
        programming_skills();
        break;
    }
}
