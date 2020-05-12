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

void autonomous()
{
    //testMinSpeed(25);
    // turn(-90);
    // turn(90, 1, 15);

    red_home_row();
}
