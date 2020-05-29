#include "main.h"
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

void red_home_row()
{
    pros::Task grab_first_goal_ball(loadIntake);
    sweepRight(135, 0);
    move(6, 135, 0);
    timedDrive(300, 30);
    intakeFullStop();
    scoreOneBall();
    frontRollers(-127);

    moveBack(66, 90, 1, MOVE_KP, 20, 0.3);
    turn(180);
    pros::Task load1(loadIntake);
    move(12, 180, 1, true);
    intakeFullStop();
    timedDrive(300, 30);
    scoreOneBall();

    moveBack(12, 180, 0);
    turn(270);
    pros::Task load2(loadIntake);
    move(50, 225, 5, MOVE_KP, 15, 0.325);
    intakeFullStop();
    timedDrive(500, 40);
    scoreOneBall();
    moveBack(24, 225, 1);
}

void programming_skills_57()
{
    pros::Task grab_first_goal_ball(loadIntake);
    sweepRight(135, 0);
    move(3, 135, 0, true);
    timedDrive(250, 40);
    intakeFullStop();
    scoreOneBall();
    frontRollers(-127);

    moveBack(32, 180, 1, MOVE_KP, 20, 0.3);
    turn(270);
    pros::Task load1(loadIntake);
    move(40, 270, 2);
    turn(180);
    move(22, 180, 0, true);
    intakeFullStop();
    timedDrive(100, 30);
    stopDrive();
    scoreBalls(500);

    moveBack(28, 180, 0);
    turn(270);
    pros::Task load2(loadIntake);
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
    pros::Task load3(loadIntake);

    move(2, 90, 1);
    sweepLeft(0, 1, 10);
    move(29, 0, 0, 0.15, 30, 0.6);
    turn(-90);
    intakeFullStop();
    timedDrive(350, 30);
    scoreOneBall();
    moveBack(8, -90, 1);
    turn(0);
    pros::Task load4(loadIntake);

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
    pros::Task load5(loadIntake);
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
    pros::Task load6(loadIntake);
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
    //testMinSpeed(25);
    // turn(-90);
    // turn(90, 1, 15);
    //red_home_row();
    programming_skills_57();
}
