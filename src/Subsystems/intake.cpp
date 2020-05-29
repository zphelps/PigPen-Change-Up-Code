#include "main.h"

//Motors - 10
pros::Motor rightRollerMotor(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor leftRollerMotor(18, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor mainRollerMotor(19, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor indexerMotor(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

//Sensors
pros::ADIDigitalIn indexerLimit('G');
pros::ADIDigitalIn intakeLimit('H');

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

void loadIntake(void *parameter)
{
    frontRollers(127);
    while (intakeLimit.get_value() != 1)
    {
        frontRollers(127);
        intake(127);
    }
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

void intakeOP()
{
    if (master.get_digital(DIGITAL_R1))
    {
        if (intakeLimit.get_value())
        {
            frontRollers(127);
        }
        else
        {
            frontRollers(127);
            intake(127);
        }
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