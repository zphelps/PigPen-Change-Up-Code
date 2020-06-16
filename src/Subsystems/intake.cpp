#include "main.h"

//Motors - 10
pros::Motor rightRollerMotor(10, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); //Orange - 17
pros::Motor leftRollerMotor(18, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor mainRollerMotor(19, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor indexerMotor(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

//Sensors
pros::ADIDigitalIn indexerLimit('G');
pros::ADIDigitalIn intakeLimit('H');
pros::Vision vision_sensor(16);

int stopLoading = 0;

void frontRollers(int speed)
{
    rightRollerMotor.move(speed);
    leftRollerMotor.move(speed);
}

void intake(int speed)
{
    mainRollerMotor.move(speed);
}

void fullIntake(int speed)
{
    frontRollers(127);
    intake(127);
}

void indexer(int speed)
{
    indexerMotor.move(speed);
}

void intakeFullStop()
{
    intake(0);
    indexer(0);
    frontRollers(0);
}

void intakeFullReverse()
{
    intake(-127);
    indexer(-127);
    frontRollers(-127);
}

void loadOneBallAndReverse(void *parameter)
{
    int count = 0;
    while (count < 1)
    {
        frontRollers(127);
        intake(127);
        if (intakeLimit.get_value() == 1)
        {
            count++;
            wait(100);
        }
    }
    frontRollers(-127);
    intake(0);
}

void loadOneBall(void *parameter)
{
    int count = 0;
    while (count < 1)
    {
        frontRollers(127);
        intake(127);
        if (intakeLimit.get_value() == 1)
        {
            count++;
            wait(300);
        }
    }
    frontRollers(0);
    intake(0);
}

void scoreOneBall()
{
    while (indexerLimit.get_value() != 1)
    {
        indexer(127);
        intake(127);
    }
    intake(0);
    wait(200);
    indexer(0);
}

void scoreOneBallWithFrontRollers()
{
    int intakeCount = 0;
    int indexerCount = 0;
    frontRollers(127);
    while (intakeCount < 2)
    {
        left(50);
        right(50);
        indexer(127);
        intake(127);
        if (intakeLimit.get_value() == 1)
        {
            intakeCount++;
            wait(75);
        }
        if (indexerLimit.get_value() == 1)
        {
            indexerCount++;
            wait(75);
        }
    }
    master.print(0, 0, "%d", indexerCount);
    frontRollers(-127);
    if (indexerCount <= 2)
    {
        scoreOneBall();
    }
}

void scoreBalls(int time)
{
    while (indexerLimit.get_value() == 0)
    {
        indexer(127);
        intake(127);
    }
    wait(200);
    intake(0);
    indexer(0);
    wait(50);
    intake(127);
    indexer(127);
    wait(time);
    intake(0);
    indexer(0);
}

void countThreeBlue()
{
    while (intakeLimit.get_value() == 0)
    {
        frontRollers(127);
        intake(127);
    }
    scoreOneBall();
}

// void visionTest()
// {
//     while (true)
//     {
//         vision_object_s_t returnRed = vision_sensor.get_by_sig(0, RED_BALL);
//         vision_object_s_t returnBlue = vision_sensor.get_by_sig(0, BLUE_BALL);
//     }
// }
void oneRedOneBlue()
{
    int count = 0;
    scoreOneBall();

    while (intakeLimit.get_value() == 0)
    {
        left(35);
        right(35);
        frontRollers(127);
        intake(75);
        // if (intakeLimit.get_value() == 1)
        // {
        //    // count++;
        //     wait(300);
        // }
    }
    intakeFullStop();
    scoreOneBall();
    wait(500);

    while (intakeLimit.get_value() == 0)
    {
        left(35);
        right(35);
        frontRollers(127);
        intake(75);
        // if (intakeLimit.get_value() == 1)
        // {
        //    // count++;
        //     wait(300);
        // }
    }
    intakeFullStop();
    scoreOneBall();
    wait(500);
    intakeFullReverse();
    wait(500);
    //intake(127);
    //indexer(127);
    //frontRollers(0);

    //for (int i = 0; i < 2; i++)
    // if (count > 2)
    // {
    //     intake(0);
    //     frontRollers(0);
    // }
}

void cycleBlueRed()
{
    for (int i = 0; i < 2; i++)
    {
        while (intakeLimit.get_value() == 0)
        {
            left(45);
            right(45);
            frontRollers(127);
            intake(127);
        }
        wait(300);
        intake(0);
        frontRollers(0);
    }
    scoreOneBall();
}

void topBallSwitch(int timeout)
{
    int time = 0;
    while (indexerLimit.get_value() == 0)
    {
        intake(127);
        indexer(127);
    }
    while (intakeLimit.get_value() == 0 && time < timeout)
    {
        left(40);
        right(40);
        intake(127);
        frontRollers(127);
        time++;
        wait(1);
    }
    intakeFullStop();
}

void topBallSwitch()
{
    while (indexerLimit.get_value() == 0)
    {
        intake(127);
        indexer(127);
    }
    while (intakeLimit.get_value() == 0)
    {
        intake(127);
        frontRollers(127);
    }
    intakeFullStop();
}

void intakeTests()
{
    int count = 0;

    while (count < 2)
    {
        intake(127);
        indexer(127);
        frontRollers(127);
        if (indexerLimit.get_value() == 1)
        {
            count++;
            wait(300);
        }
    }
    intakeFullStop();
}
void intakeOP()
{
    if (master.get_digital(DIGITAL_R1))
    {
        // if (intakeLimit.get_value())
        // {
        //     frontRollers(127);
        //     intake(0);
        //     wait(100);
        // }
        // else
        // {
        //     frontRollers(127);
        //     intake(127);
        // }
        frontRollers(127);
        intake(127);
    }
    else if (master.get_digital(DIGITAL_R2))
    {
        frontRollers(-127);
    }
    else if (master.get_digital(DIGITAL_Y))
    {
        frontRollers(-127);
        intake(-127);
    }
    else if (master.get_digital(DIGITAL_RIGHT))
    {
        topBallSwitch();
    }
    else if (master.get_digital(DIGITAL_L1))
    {
        frontRollers(0);
        if (indexerLimit.get_value() == 1)
        {
            indexer(127);
            intake(0);
            wait(100);
        }
        else
        {
            indexer(127);
            intake(127);
        }
    }
    else
    {
        frontRollers(0);
        intake(0);
        indexer(0);
    }
}