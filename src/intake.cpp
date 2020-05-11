#include "main.h"

//Motors
pros::Motor rightRollerMotor(10, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor leftRollerMotor(18, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor mainRollerMotor(19, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor indexerMotor(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

//Sensors
pros::ADIDigitalIn indexerLimit('G');
pros::ADIDigitalIn intakeLimit('H');

bool indexIsPressed()
{
    if (indexerLimit.get_value() == 1)
    {
        return true;
    }
    return false;
}

bool intakeIsPressed()
{
    if (intakeLimit.get_value() == 1)
    {
        return true;
    }
    return false;
}

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
    else if (master.get_digital(DIGITAL_L1))
    {
        if (indexerLimit.get_value())
        {
            indexer(127);
            intake(0);
            wait(20);
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