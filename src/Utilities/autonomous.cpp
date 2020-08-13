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
    // //turn(90, TURN_KP, 50);
    // //move(15, 0, 2, true);
    // sweepRight(130, 0);
    // frontRollers(127);
    // //fullIntake(127);
    // //pros::Task load1(loadIntake);
    // //fullIntake(127);
    // move(8, 130, 2);
    // timedDrive(300, 30);
    // intakeFullStop();
    // scoreOneBall();
    // frontRollers(127);
    // wait(200);
    // intakeFullStop();
    // oneRedOneBlue();

    // intakeFullReverse();
    // wait(500);
    // //moveBack(20, 130, 3);
    // //intakeFullStop();
    // // sweepLeftBack(90, 0, 3.5, 30);
    // // sweepLeftBack(90, 0, 15);
    // // moveBack(51, 90, 0);
    // // sweepRight(180, 0, 3.0, 30);
    // moveBack(68, 90, 1, MOVE_KP, 20, 0.25);
    // turn(180);
    // move(15, 180, 0, true);
    // //timedDrive(300, 30);
    // //scoreOneBall();
    // cycleBlueRed();
    // moveBack(10, 180, 2);
    // /*
    // turn(-90);
    // move(30, -90, 2);
    // turn(-145, 2.5, MOVE_MIN_SPEED);
    // pros::Task load3(loadIntake);
    // move(41, -145, 2);
    // turn(-140);
    // timedDrive(500, 30);
    // wait(2000);
    // intakeFullStop();
    // //move(15, 130, 0);
    // // correct(20, 20);*/
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
    frontRollers(127);
    move(10, 180, 0, 0.3, 40, 0.2); //8
    timedDrive(500, 40);
    scoreOneBall();
    frontRollers(0);
    wait(250);
    scoreOneBall();
    moveBack(14, 180, 2, 0.15, 30, 0.2);
    intakeFullReverse();
    // move(10, 180, 0, 0.3, 40, 0.2); //8
    // timedDrive(300, 30);
    // scoreOneBall();
    // intakeFullReverse();
}

void twoGoalAutonLeft()
{
    setCoordinates(0, 125, 0);
    sweepLeft(-135, 0);
    // pros::Task task1(loadOneBallAndReverse);
    frontRollers(127);
    move(8, -135, 0);
    timedDrive(300, 30);
    frontRollers(0);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(10, -135, 0, true);
    fullIntake(127);
    //wait(100000);
    sweepRightBack(-90, 0, 20, 5, 100);
    moveBackToYCoord(90, -90, 0, 6, 30, 0.8);

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
    // //turn(90, TURN_KP, 50);
    // //move(15, 0, 2, true);
    // sweepLeft(-130, 0);
    // //fullIntake(127);
    // //pros::Task load1(loadIntake);
    // //fullIntake(127);
    // frontRollers(127);
    // move(8, -130, 2);
    // timedDrive(300, 30);
    // intakeFullStop();
    // scoreOneBall();
    // frontRollers(127);
    // wait(200);
    // intakeFullStop();
    // oneRedOneBlue();

    // intakeFullReverse();
    // wait(500);
    // //moveBack(20, 130, 3);
    // //intakeFullStop();
    // // sweepLeftBack(90, 0, 3.5, 30);
    // // sweepLeftBack(90, 0, 15);
    // // moveBack(51, 90, 0);
    // // sweepRight(180, 0, 3.0, 30);
    // moveBack(60, -90, 1, MOVE_KP, 20, 0.25);
    // turn(-180);
    // move(15, -180, 0, true);
    // //timedDrive(300, 30);
    // //scoreOneBall();
    // cycleBlueRed();
    // moveBack(10, -180, 2);
    // /*
    // turn(-90);
    // move(30, -90, 2);
    // turn(-145, 2.5, MOVE_MIN_SPEED);
    // pros::Task load3(loadIntake);
    // move(41, -145, 2);
    // turn(-140);
    // timedDrive(500, 30);
    // wait(2000);
    // intakeFullStop();
    // //move(15, 130, 0);
    // // correct(20, 20);*/
}

void home_row_right()
{
    setCoordinates(0, 24, 0);
    sweepRight(135, 0);
    frontRollers(127);
    move(8, 135, 0);
    frontRollers(0);
    timedDrive(300, 30);
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
    frontRollers(0);
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

void home_row_left()
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

    moveBack(10, -135, 0, true);
    fullIntake(127);
    //wait(100000);
    sweepRightBack(-90, 0, 20, 5, 100);
    moveBackToYCoord(90, -90, 0, 6, 30, 0.8);

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
    move(16, -270, 2, true);
    sweepRight(-225, 0, 10, 2, 100);
    //pros::Task task3(loadOneBallAndReverse);
    frontRollers(127);
    move(10, -225, 0, 0.2, 40, 0.5);
    timedDrive(500, 40);
    scoreOneBall();
    wait(200);
    scoreOneBall();
    frontRollers(0);
    moveBack(24, -225, 1);
    intakeFullReverse();
    // move(10, -225, 0, true);
    // timedDrive(500, 40);
    // scoreOneBall();
    // intakeFullReverse();
    // moveBack(24, -225, 1);
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
    pros::Task intake_manager(intakeManager);
    frontRollers(127);
    intake(90);
    move(8, 0, 0, 0.2, 40, 0.8);
    sweepRight(90, 0, 10, 1.5, 35); //1.35, 35
    move(15, 90, 0, true);
    timedDrive(250, 35);       //350
    moveBack(16, 90, 1, true); //15
    frontRollers(0);
    sweepRightBack(135, 5, 1.45, 35);
    move(20, 135, 1, 0.3, 35, 0.2); //0.25
    timedDrive(300, 35);
    wait(200);
    scoreOneBall();

    moveBackToYCoord(9, 135, 1, 8.5, MOVE_MIN_SPEED, 0.2); //10

    intake_manager.resume();
    intake(90);
    frontRollers(127);
    turn(0, 1.25, 20);                  //1.15 25
    moveToXCoord(60, 0, 1, 4, 30, 0.2); //4, 30
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
    turn(5, 1.4, 25);
    //move(1, 5, 0, true);
    sweepRight(45, 30, 5, 2.75, 50); //45, 35
    frontRollers(0);
    timedDrive(200, 35); //500
    scoreOneBall();
    sweepLeftBack(0, 0, 5, 5, 100);
    moveBackToXCoord(89, 0, 0, 10, 25, 0.3); //6, 30

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
    timedDrive(250, 45);
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
    moveBack(30, 225, 0);
    intakeFullStop();

    sweepLeftBack(180, 0, 10, 5, 90);
    moveBackToXCoord(50, 180, 0, 6, 30, 0.5);

    intake(127);
    frontRollers(127);
    turn(90);
    move(26, 90, 0, 0.1, 30, 0.2);
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
    //programming_skills_new();
    //home_row_right_no_cycle();
    //home_row_left();
    // timedDrive(300, 35);
    // scoreOneBall();
    // wait(200);
    // scoreOneBall();
    //home_row_left();
    twoGoalAutonLeft();
    // frontRollers(127);
    // intake(90);
    // pros::Task test(intakeManager);
    // wait(1000000);
    // turn(90);
    // wait(200);
    // turn(180);
    // wait(200);
    // turn(270);
    // wait(200);
    // turn(360);
    //turn(90);
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
