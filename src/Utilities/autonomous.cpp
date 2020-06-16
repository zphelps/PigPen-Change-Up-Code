#include "main.h"

auto okapiChassis = ChassisControllerBuilder()
                        .withMotors({9, 2}, {5, 1})
                        .withGains(
                            {0.01, 0, 0},
                            {0.1, 0, 0})
                        .withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 12.5_in}, imev5BlueTPR / 0.6})
                        .build();

auto profileController =
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
                                     {5_ft, 5_ft, 0_deg}},
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
    //scoreOneBall();
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
    //intakeFullStop();
    //scoreOneBall();
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

void home_row_right_no_intake_button()
{
    setCoordinates(0, 24, 0);
    sweepRight(135, 0);
    frontRollers(127);
    move(8, 135, 0);
    timedDrive(300, 30);
    wait(300);
    frontRollers(-127);
    scoreOneBall();

    fullIntake(127);
    moveBack(10, 135, 0, true);
    sweepLeftBack(90, 0, 15, 5, 100);
    moveBackToYCoord(58, 90, 0, 6, 30, 0.8);

    turn(180);
    intakeFullStop();
    move(8, 180, 0, 0.3, 40, 0.2);
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, 180, 0, 0.25, 35, 0.8);
    turn(270);
    move(22, 270, 2, true);
    sweepLeft(225, 0, 10, 2, 100);
    frontRollers(127);
    move(10, 225, 0, true);
    timedDrive(500, 40);
    wait(500);
    frontRollers(-127);
    scoreOneBall();
    intakeFullReverse();
    wait(100);
    moveBack(24, 225, 1);
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
    moveBackToYCoord(58, 90, 0, 6, 30, 0.8);

    turn(180);
    intakeFullStop();
    move(8, 180, 0, 0.3, 40, 0.2);
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, 180, 0, 0.25, 35, 0.8);
    turn(270);
    move(12, 270, 2, true);
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

void one_goal_left_no_intake_button()
{
    setCoordinates(0, 125, 0);
    sweepLeft(-135, 0);
    frontRollers(127);
    move(8, -135, 0);
    timedDrive(300, 30);
    wait(300);
    frontRollers(-127);
    scoreOneBall();
    moveBack(10, -135, 2);
}

void home_row_left_no_intake_button()
{
    setCoordinates(0, 125, 0);
    sweepLeft(-135, 0);
    frontRollers(127);
    move(8, -135, 0);
    timedDrive(300, 30);
    wait(300);
    frontRollers(-127);
    scoreOneBall();

    fullIntake(127);
    moveBack(10, -135, 0, true);
    sweepRightBack(-90, 0, 15, 5, 100);
    moveBackToYCoord(90, -90, 0, 6, 30, 0.8);

    turn(-180);
    intakeFullStop();
    move(8, -180, 0, 0.3, 40, 0.2);
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, -180, 0, 0.25, 35, 0.8);
    turn(-270);
    move(22, -270, 2, true);
    sweepRight(-225, 0, 10, 2, 100);
    frontRollers(127);
    move(10, -225, 0, true);
    timedDrive(500, 40);
    wait(500);
    frontRollers(-127);
    scoreOneBall();
    intakeFullReverse();
    wait(100);
    moveBack(24, -225, 1);
}

void programming_skills_57()
{
    pros::Task grab_first_goal_ball(loadOneBall);
    sweepRight(135, 0);
    move(3, 135, 0, true);
    timedDrive(250, 40);
    intakeFullStop();
    scoreOneBall();
    frontRollers(-127);

    moveBack(32, 180, 1, MOVE_KP, 20, 0.3);
    turn(270);
    pros::Task load1(loadOneBall);
    move(40, 270, 2);
    turn(180);
    move(22, 180, 0, true);
    intakeFullStop();
    timedDrive(100, 30);
    stopDrive();
    scoreBalls(500);

    moveBack(28, 180, 0);
    turn(270);
    pros::Task load2(loadOneBall);
    move(22, 270, 1);
    turn(225, 1.5, 25);
    move(30, 220, 2, true);
    intakeFullStop();
    timedDrive(500, 40);
    scoreOneBall();
    sweepLeftBack(90, 1);
    timedDrive(500, -50);
    wait(200);
    setTheta(90);
    pros::Task load3(loadOneBall);

    move(2, 90, 1);
    sweepLeft(0, 1, 10);
    move(29, 0, 0, 0.15, 30, 0.6);
    turn(-90);
    intakeFullStop();
    timedDrive(350, 30);
    scoreOneBall();
    moveBack(8, -90, 1);
    turn(0);
    pros::Task load4(loadOneBall);

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
    pros::Task load5(loadOneBall);
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
    pros::Task load6(loadOneBall);
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

void autonomous()
{
    //scoreOneBallWithFrontRollers();
    //scoreOneBall();
    //home_row_right();
    one_goal_left_no_intake_button();
    //home_row_left_no_intake_button();
    //home_row_right_no_intake_button();
    //home_row_left();
    //intakeTests();
    //testMinSpeed(25);
    // turn(-90);
    // turn(90, 1, 15);
    //programming_skills_57();
    //twoGoalAutonRight();
    //twoGoalAutonLeft();
    //oneRedOneBlue();
    //test_sTurn();
    // turn(90);
    // wait(500);
    // turn(180);
    // wait(500);
    // turn(270);
    // wait(500);
    // turn(360);
    //turn(-45, TURN_KP, 30);
    //moveAboslute(24, 24, 1);
}
