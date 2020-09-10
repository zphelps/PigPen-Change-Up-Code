#include "main.h"

//Ready for Michigan tournament!

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
    timedDrive(300, 30); //300
    frontRollers(0);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(8, 135, 0, true); //10
    fullIntake(127);
    sweepLeftBack(90, 0, 15, 5, 100);
    moveBackToYCoord(56, 90, 0, 6, 30, 0.6); //0.8
    //58

    indexer(-127);
    turn(180);
    indexer(0);
    intakeFullStop();
    move(10, 180, 0, 0.3, 40, 0.2); //8
    timedDrive(100, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, 180, 2, 0.15, 30, 0.2);
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
    timedDrive(300, 35);
    frontRollers(0);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(8, 135, 0, true); //10
    fullIntake(127);
    sweepLeftBack(90, 0, 15, 5, 100);
    moveBackToYCoord(56, 90, 0, 6, 30, 0.6);
    //58

    indexer(-127);
    turn(180);
    indexer(0);
    intakeFullStop();
    frontRollers(127);
    move(10, 180, 0, 0.3, 40, 0.2); //8
    timedDrive(500, 35);
    scoreOneBall(2000);
    indexer(-127);
    wait(250);
    indexer(0);
    scoreOneBall(3000);
    intakeFullReverse();

    moveBack(14, 180, 2, 0.15, 30, 0.2);
    turn(270);
    move(28, 270, 1, true); //26
    sweepLeft(225, 30, 20, 1.6, 80);
    frontRollers(127);
    move(10, 225, 0, 0.2, 40, 0.5);
    timedDrive(500, 40);
    scoreOneBall();
    wait(200);
    frontRollers(0);
    scoreOneBall();
    intakeFullReverse();
    moveBack(24, 225, 1);
}

void home_row_right_no_cycle()
{
    setCoordinates(0, 24, 0);
    sweepRight(135, 0, 1.5, 25);
    frontRollers(127);
    move(8, 135, 0, 0.2, 35, 0.2);
    timedDrive(300, 30); //300
    frontRollers(0);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(8, 135, 0, true); //10
    fullIntake(127);
    sweepLeftBack(90, 0, 15, 5, 100);
    moveBackToYCoord(56, 90, 0, 6, 30, 0.6); //0.8
    //58

    indexer(-127);
    turn(180);
    indexer(0);
    intakeFullStop();
    move(10, 180, 0, 0.3, 40, 0.2); //8
    timedDrive(100, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, 180, 2, 0.15, 30, 0.2);
    turn(270);
    move(26, 270, 1, true);          //23
    sweepLeft(225, 30, 20, 1.6, 80); //1.75
    frontRollers(127);
    move(10, 225, 0, 0.2, 40, 0.5);
    timedDrive(500, 40);
    scoreOneBall();
    wait(200);
    frontRollers(0);
    scoreOneBall();
    intakeFullReverse();
    moveBack(24, 225, 1);
}

void home_row_left_no_cycle()
{
    setCoordinates(0, 125, 0);
    sweepLeft(-135, 0);
    frontRollers(127);
    move(8, -135, 0, 0.2, 35, 0.2);
    frontRollers(0);
    timedDrive(300, 30);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(8, -135, 0, true);
    fullIntake(127);
    sweepRightBack(-90, 0, 20, 5, 100);
    moveBackToYCoord(90, -90, 0, 6, 30, 0.8);

    turn(-180);
    intakeFullStop();
    move(10, -180, 0, 0.3, 40, 0.2); //8
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, -180, 2, 0.25, 35, 0.8);
    turn(-270);
    move(16, -270, 2, true); //18
    sweepRight(-225, 0, 20, 1.6, 80);
    frontRollers(127);
    move(10, -225, 0, 0.2, 40, 0.5);
    timedDrive(500, 40);
    scoreOneBall();
    wait(200);
    scoreOneBall();
    intakeFullReverse();
    moveBack(24, -225, 1);
}

void programming_skills()
{
    setCoordinates(1, 24, 0);
    pros::Task intake_manager(intakeManager);
    frontRollers(127);
    intake(127);
    move(8, 0, 0, 0.2, 40, 0.8);
    sweepRight(90, 0, 10, 1.75, 25); //1.35, 35
    intake(90);
    move(15, 90, 0, true);
    timedDrive(350, 35);       //350
    moveBack(16, 90, 1, true); //15
    frontRollers(0);
    sweepRightBack(135, 5, 1.5, 30);
    intake_manager.suspend();
    move(20, 135, 1, 0.4, 35, 0.2); //0.3
    timedDrive(300, 35);
    scoreOneBall(2000);

    moveBackToYCoord(10, 135, 1, 8.5, MOVE_MIN_SPEED, 0.2); //10

    //intake_manager.resume();
    frontRollers(127);
    turn(0, 1.25, 20); //1.15 25
    intake(90);
    intake_manager.resume();
    moveToXCoord(60, 0, 0, 5, 30, 0.2); //4, 30
    turn(90);
    frontRollers(0);
    move(6, 90, 0, 0.2, 35, 0.2);
    intake_manager.suspend();
    frontRollers(0);
    timedDrive(250, 35); //100, 30
    scoreOneBall(2000);
    wait(250);
    scoreOneBall(2000);
    moveBack(10, 90, 0);

    intake_manager.resume();
    frontRollers(127);
    intake(90);
    turn(0);
    moveToXCoord(77, 0, 0, 10, 45, 0.2); //76, 0, 0,8
    sweepRight(90, 0, 1.3, 35);          //1.2
    timedDrive(350, 40);                 //800
    frontRollers(0);
    wait(250);
    timedDrive(750, 60);
    setTheta(90);
    setCoordinates(getX(), -3, getTheta());
    wait(100);
    frontRollers(127);
    moveBackToYCoord(5, 90, 0);
    turn(0, 1.4, 25);                //5
    sweepRight(45, 30, 5, 2.75, 50); //45, 35
    intake_manager.suspend();
    frontRollers(0);
    timedDrive(200, 35); //500
    scoreOneBall(2000);
    sweepLeftBack(0, 0, 5, 5, 100);
    moveBackToXCoord(89, 0, 0, 10, 25, 0.4); //6, 30, 0.3

    intake_manager.resume();
    intake(127);
    frontRollers(127);
    turn(-90);
    intake(0);
    moveToYCoord(54, -90, 1, 4.5, 30, 0.2); //56
    turn(0);
    move(28, 0, 0); //25
    frontRollers(0);
    intake_manager.suspend();
    timedDrive(250, 35); //100
    scoreOneBall(2000);
    wait(250);
    scoreOneBall(2000);
    setCoordinates(132, getY(), getTheta());

    frontRollers(-127);
    moveBack(8, 0, 2);
    turn(-90);
    intake(127);
    frontRollers(127);
    move(28, -90, 1, true); //24
    intake_manager.resume();
    sweepRight(-45, 0, 15, 2, 75);
    move(3, -25, 0);
    intake_manager.suspend();
    frontRollers(0);
    timedDrive(350, 35);
    scoreOneBall(2000);
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
    timedDrive(250, 75);
    wait(250);
    setTheta(-90);

    frontRollers(127);
    moveBack(36, -90, 1); //35
    turn(-180);
    move(35, -180, 0); //34
    turn(-90);

    move(28, -90, 0);
    intake_manager.suspend();
    frontRollers(0);
    timedDrive(100, 30);
    scoreOneBall();
    wait(250);
    scoreOneBall();
    moveBack(22, -88, 2, 0.1, 30, 0.6); //0.8
    frontRollers(127);

    turn(90, 1.35, 15);
    move(14, 90, 1);
    left(100);
    right(100);
    intake(127);
    frontRollers(127);
    wait(1000);
    scoreOneBallInCenterGoal();
    indexer(-127);
    intake(127);
    frontRollers(127);
    wait(350);

    frontRollers(-127);

    moveBack(16, 90, 1);

    pros::Task eject(ejectBalls);
    turn(135, 1.5, 25);
    move(25, 135, 0, true);
    sweepRight(180, -5, 1.75, 35);
    eject.suspend();
    move(24, 180, 1, 0.2, 30, 0.2); //24
    frontRollers(127);
    timedDrive(400, 35);
    scoreOneBall();
    frontRollers(0);

    moveBack(5, 180, 2, 0.15, 30, 0.2);
    frontRollers(127);
    intake(127);
    indexer(-127);
    turn(270);
    indexer(0);
    move(32, 270, 1, true); //28
    sweepLeft(225, 30, 15, 1.75, 80);
    move(10, 225, 0, 0.2, 40, 0.5);
    timedDrive(500, 40);
    scoreOneBall();
    moveBack(24, 225, 0);
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
    //scoreOneBall();
    //programming_skills();
    //home_row_right_no_cycle();
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
        twoGoalAutonRight();
        break;
    case 4:
        programming_skills();
        break;
    }
}
