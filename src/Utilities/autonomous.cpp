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
}

void twoGoalAutonLeft()
{
}

void home_row_right_no_intake_button()
{
}

void home_row_right()
{
}

void home_row_left()
{
}

void one_goal_left_no_intake_button()
{
}

void home_row_left_no_intake_button()
{
}

void programming_skills_57()
{
}

void autonomous()
{
    //scoreOneBallWithFrontRollers();
    //scoreOneBall();
    //home_row_right();

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
