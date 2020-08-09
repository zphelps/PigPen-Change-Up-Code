#include "main.h"

//Motors - 10
pros::Motor rightRollerMotor(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); //Orange - 17
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

void intakeOP()
{

    if (master.get_digital(DIGITAL_R1))
    {
        if (intakeLimit.get_value() == 1)
        {
            frontRollers(127);
            //intake(50);
            indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }
        else
        {
            frontRollers(127);
            intake(90);
            //indexer(75);
        }
    }
    else if (master.get_digital(DIGITAL_R2))
    {
        frontRollers(-127);
    }
    else if (master.get_digital(DIGITAL_L2))
    {
        indexer(-127);
        intake(127);
    }
    else if (master.get_digital(DIGITAL_L1))
    {
        indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        frontRollers(0);

        if (indexerLimit.get_value() == 1)
        {
            intake(0);
            wait(150);
            indexer(0);
            wait(100);
        }
        else if (intakeLimit.get_value() == 0)
        {
            indexer(127);
            intake(127);
        }
        else
        {
            indexer(127);
            intake(0);
        }
    }
    else
    {
        frontRollers(0);
        intake(0);
        indexer(0);
    }
}
//Counting rollers from the top
/*
void findColorWhenRed()
{
    frontRollers(127);
    when button pressed{
        frontRollers(0);
        if redBall{
            moveRoller4(-127);
            moveRoller3(127);
            moveRoller2(-127);
            moveRoller1(127);
        }
        else if blueBall{
            moveRoller4(-127);
            moveRoller3(127);
            moveRoller2(127);
        }
    }
}
void findColorWhenBlue()
{
    frontRollers(127);
    when button pressed{
        frontRollers(0);
        if blueBall{
            moveRoller4(-127);
            moveRoller3(127);
            moveRoller2(-127);
            moveRoller1(127);
        }
        else if redBall{
            moveRoller4(-127);
            moveRoller3(127);
            moveRoller2(127);
        }
    }
}
void oneBlueOneRedWithRedCycle()//or oneRedWithOneBlueCycle
{
    moveFoward
    findColor();//Do this 3 times with waits in between
}
void oneRedOneBlueWithBlueCycle()// or oneBlueWithOneRedCycle
{
    moveForward
    findColor();//Do this twice with waits in between
}
*/