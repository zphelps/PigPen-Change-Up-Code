#include "main.h"

//XDrive
pros::Motor leftFront(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor leftBack(2, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rightFront(8, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rightBack(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

//**************************Helper Functions************************************
void left(int speed)
{
    leftFront.move(speed);
    leftBack.move(speed);
}

void right(int speed)
{
    rightFront.move(speed);
    rightBack.move(speed);
}

void brake()
{
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void coast()
{
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void timedDrive(int time, int speed)
{
    right(speed);
    left(speed);
    wait(time);
    right(speed);
    left(speed);
}

//**************************Drive OPControl*************************************
void driveOP()
{
    leftFront.move(master.get_analog(ANALOG_LEFT_Y));
    leftBack.move(master.get_analog(ANALOG_LEFT_Y));
    rightFront.move(master.get_analog(ANALOG_RIGHT_Y));
    rightBack.move(master.get_analog(ANALOG_RIGHT_Y));
}

//******************************************************************************
//**************************Move Functions**************************************

//******************************************************************************
//**************************Turn Functions**************************************

//******************************************************************************
//*************************Sweep Functions**************************************

//******************************************************************************
//******************************************************************************