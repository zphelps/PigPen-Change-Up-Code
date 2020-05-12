#include "main.h"

//Motors
pros::Motor leftFront(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor leftBack(2, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rightFront(8, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rightBack(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

//Inertial
pros::Imu inertial(8);

//PID Gains
double MOVE_KP = 0.15;
double MOVE_KI = 0;
double MOVE_KD = 0;
double MOVE_ACCEL_STEP = 5;
int START_SPEED = 127;
int MOVE_MIN_SPEED = 20;
int MOVE_MIN_SPEED_FLUID = 75;

double TURN_KP = 1.25;
double TURN_KI = 0;
double TURN_KD = 0;
//25 turns the robot
int POINT_TURN_MIN_SPEED = 15;

const double WHEEL_DIAMETER = 2.75;
const int TICS_PER_REVOLUTION = 360;
const double LEFT_OFFSET = 5.95; //7
const double RIGHT_OFFSET = 6.1; //5.95
const double REAR_OFFSET = 5.75;

int rightSideSpeed = 0;
int leftSideSpeed = 0;

//**********************Drive PIDControllers************************************

PIDController movePID(MOVE_KP, MOVE_KI, MOVE_KD, MOVE_MIN_SPEED);

PIDController movePIDFluid(MOVE_KP, MOVE_KI, MOVE_KD, MOVE_MIN_SPEED_FLUID);

PIDController pointTurnPID(TURN_KP, TURN_KI, TURN_KD, POINT_TURN_MIN_SPEED);

PIDController sweepTurnPID = PIDController(1.1, 0, 0, 30);

PIDController sweepTurnPIDFluid(1.75, 0, 0, 80);

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

void slewRight(int speed, int accelStep)
{
    if (speed > rightSideSpeed)
    {
        rightSideSpeed++;
        right(rightSideSpeed);
    }
    else
    {
        right(speed);
    }
    wait(accelStep);
}

void slewLeft(int speed, int accelStep)
{
    if (speed > leftSideSpeed)
    {
        leftSideSpeed++;
        left(leftSideSpeed);
    }
    else
    {
        left(speed);
    }
    wait(accelStep);
}

void slewRightBack(int speed, int accelStep)
{
    if (speed < rightSideSpeed)
    {
        rightSideSpeed--;
        right(rightSideSpeed);
    }
    else
    {
        right(speed);
    }
    wait(accelStep);
}

void slewLeftBack(int speed, int accelStep)
{
    if (speed < leftSideSpeed)
    {
        leftSideSpeed--;
        left(leftSideSpeed);
    }
    else
    {
        left(speed);
    }
    wait(accelStep);
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

void initializeInertialSensor()
{
    inertial.reset();

    int time = pros::millis();
    int iter = 0;
    while (inertial.is_calibrating())
    {
        iter += 10;
        wait(10);
    }
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
/**
 * @brief Standard Move Function
 * 
 * @param distance 
 * @param heading 
 * @param fluid 
 */
void move(int distance, int heading, int accelStep, bool fluid)
{
    if (fluid)
    {
        moveFluid(distance, heading, accelStep);
    }

    double correctionMultiplier = 0.2;

    double target = (L.get_value()) + TICS_PER_REVOLUTION * (distance / (WHEEL_DIAMETER * PI));

    while (L.get_value() < target && !fluid)
    {

        double PIDSpeed = (movePID.getOutput(target, L.get_value()));

        if ((heading - getTheta() >= 5 || heading - getTheta() <= -5) && !fluid)
        {
            if (getTheta() < heading)
            {
                slewRight(correctionMultiplier * PIDSpeed, accelStep);
                slewLeft(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(correctionMultiplier * PIDSpeed, accelStep);
            }

            if (getTheta() == heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(PIDSpeed, accelStep);
            }
        }
        else
        {
            if (getTheta() < heading)
            {
                slewRight(PIDSpeed * 0.8, accelStep); //0.8
                slewLeft(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(PIDSpeed * 0.8, accelStep);
            }

            if (getTheta() == heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(PIDSpeed, accelStep);
            }
        }
    }

    if (!fluid)
    {
        right(0);
        left(0);
    }

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}

/**
 * @brief Override Move Function
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param fluid 
 */
void move(int distance, int heading, int accelStep, double kP, int minSpeed, double correction)
{

    PIDController tempMovePID(kP, 0, 0, minSpeed);

    double correctionMultiplier = correction;

    double target = (L.get_value()) + TICS_PER_REVOLUTION * (distance / (WHEEL_DIAMETER * PI));

    while (L.get_value() < target)
    {

        double PIDSpeed = (tempMovePID.getOutput(target, L.get_value()));

        if ((heading - getTheta() >= 5 || heading - getTheta() <= -5))
        {
            if (getTheta() < heading)
            {
                slewRight(correctionMultiplier * PIDSpeed, accelStep);
                slewLeft(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(correctionMultiplier * PIDSpeed, accelStep);
            }

            if (getTheta() == heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(PIDSpeed, accelStep);
            }
        }
        else
        {
            if (getTheta() < heading)
            {
                slewRight(PIDSpeed * 0.8, accelStep); //0.8
                slewLeft(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(PIDSpeed * 0.8, accelStep);
            }

            if (getTheta() == heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(PIDSpeed, accelStep);
            }
        }
    }

    right(0);
    left(0);

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}

/**
 * @brief move function for combining movements
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 */
void moveFluid(int distance, int heading, int accelStep)
{
    double correctionMultiplier = 0.4; //0.2

    double target = (L.get_value()) + TICS_PER_REVOLUTION * (distance / (WHEEL_DIAMETER * PI));

    while (L.get_value() < target)
    {

        double PIDSpeed = (movePIDFluid.getOutput(target, L.get_value()));

        if ((heading - getTheta() >= 5 || heading - getTheta() <= -5))
        {
            if (getTheta() < heading)
            {
                slewRight(correctionMultiplier * PIDSpeed, accelStep);
                slewLeft(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(correctionMultiplier * PIDSpeed, accelStep);
            }

            if (getTheta() == heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(PIDSpeed, accelStep);
            }
        }
        else
        {
            if (getTheta() < heading)
            {
                slewRight(PIDSpeed * 0.8, accelStep); //0.8
                slewLeft(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(PIDSpeed * 0.8, accelStep);
            }

            if (getTheta() == heading)
            {
                slewRight(PIDSpeed, accelStep);
                slewLeft(PIDSpeed, accelStep);
            }
        }
    }
}

/**
 * @brief Standard Move Backwards Function
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param fluid 
 */
void moveBack(int distance, int heading, int accelStep, bool fluid)
{

    if (fluid)
    {
        moveBackFluid(distance, heading, accelStep);
    }

    double correctionMultiplier = 0.2;

    double target = (L.get_value()) - TICS_PER_REVOLUTION * (distance / (WHEEL_DIAMETER * PI));

    while (L.get_value() > target && !fluid)
    {

        double PIDSpeed = (movePID.getOutput(target, L.get_value()));

        if ((heading - getTheta() >= 5 || heading - getTheta() <= -5) && !fluid)
        {
            if (getTheta() < heading)
            {
                slewLeftBack(correctionMultiplier * PIDSpeed, accelStep);
                slewRightBack(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(correctionMultiplier * PIDSpeed, accelStep);
            }

            if (getTheta() == heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(PIDSpeed, accelStep);
            }
        }
        else
        {
            if (getTheta() < heading)
            {
                slewLeftBack(PIDSpeed * 0.8, accelStep); //0.8
                slewRightBack(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(PIDSpeed * 0.8, accelStep);
            }

            if (getTheta() == heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(PIDSpeed, accelStep);
            }
        }
    }

    if (!fluid)
    {
        right(0);
        left(0);
    }

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}

/**
 * @brief Override Back Function
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param kP 
 * @param minSpeed 
 */
void moveBack(int distance, int heading, int accelStep, double kP, int minSpeed, double correction)
{

    PIDController tempMovePID(kP, 0, 0, minSpeed);

    double correctionMultiplier = correction;

    double target = (L.get_value()) - TICS_PER_REVOLUTION * (distance / (WHEEL_DIAMETER * PI));

    while (L.get_value() > target)
    {

        double PIDSpeed = (tempMovePID.getOutput(target, L.get_value()));

        if ((heading - getTheta() >= 5 || heading - getTheta() <= -5))
        {
            if (getTheta() < heading)
            {
                slewLeftBack(correctionMultiplier * PIDSpeed, accelStep);
                slewRightBack(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(correctionMultiplier * PIDSpeed, accelStep);
            }

            if (getTheta() == heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(PIDSpeed, accelStep);
            }
        }
        else
        {
            if (getTheta() < heading)
            {
                slewLeftBack(PIDSpeed * 0.8, accelStep); //0.8
                slewRightBack(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(PIDSpeed * 0.8, accelStep);
            }

            if (getTheta() == heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(PIDSpeed, accelStep);
            }
        }
    }

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}

/**
 * @brief Fluid Back Function
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 */
void moveBackFluid(int distance, int heading, int accelStep)
{
    double correctionMultiplier = 0.6; //0.2

    double target = (L.get_value()) - TICS_PER_REVOLUTION * (distance / (WHEEL_DIAMETER * PI));

    while (L.get_value() > target)
    {

        double PIDSpeed = (movePIDFluid.getOutput(target, L.get_value()));

        if ((heading - getTheta() >= 5 || heading - getTheta() <= -5))
        {
            if (getTheta() < heading)
            {
                slewLeftBack(correctionMultiplier * PIDSpeed, accelStep);
                slewRightBack(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(correctionMultiplier * PIDSpeed, accelStep);
            }

            if (getTheta() == heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(PIDSpeed, accelStep);
            }
        }
        else
        {
            if (getTheta() < heading)
            {
                slewLeftBack(PIDSpeed * 0.8, accelStep); //0.8
                slewRightBack(PIDSpeed, accelStep);
            }

            if (getTheta() > heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(PIDSpeed * 0.8, accelStep);
            }

            if (getTheta() == heading)
            {
                slewLeftBack(PIDSpeed, accelStep);
                slewRightBack(PIDSpeed, accelStep);
            }
        }
    }
}

//******************************************************************************
//**************************Turn Functions**************************************
/**
 * @brief Simple Turn
 * 
 * @param degrees 
 */
void turn(int degrees)
{

    int time = 0;

    while (time < 50)
    {

        right(-pointTurnPID.getOutput(degrees, getTheta()));
        left(pointTurnPID.getOutput(degrees, getTheta()));

        if (fabs(pointTurnPID.getError()) < 2.5)
        {
            time++;
            wait(5);
        }
    }

    left(0);
    right(0);
}

/**
 * @brief Custom turn for unique situations
 * 
 * @param degrees 
 * @param kP 
 * @param minSpeed 
 */
void turn(int degrees, int kP, double minSpeed)
{

    PIDController tempPointTurnPID = PIDController(kP, 0.0, 0.0, minSpeed);

    int time = 0;

    while (time < 50)
    {

        right(-tempPointTurnPID.getOutput(degrees, getTheta()));
        left(tempPointTurnPID.getOutput(degrees, getTheta()));

        if (fabs(tempPointTurnPID.getError()) < 2.5)
        {
            time++;
            wait(5);
        }
    }

    left(0);
    right(0);
}

/**
 * @brief Anlge Correction w/ minSpeed
 * 
 * @param degrees 
 * @param minSpeed 
 */
void correct(int degrees, int minSpeed)
{

    PIDController correctionPID(0.1, 0, 0, minSpeed);

    int time = 0;

    while (time < 100)
    {

        right(-correctionPID.getOutput(degrees, getTheta()));
        left(correctionPID.getOutput(degrees, getTheta()));

        if (fabs(correctionPID.getError()) < 2)
        {
            time++;
            wait(5);
        }
    }

    left(0);
    right(0);
}

/**
 * @brief Turn w/ Inertial Sensor
 * 
 * @param degrees 
 */
void turnInertial(int degrees)
{
    int time = 0;

    while (time < 50)
    {

        right(-pointTurnPID.getOutput(degrees, getTheta()));
        left(pointTurnPID.getOutput(degrees, getTheta()));

        if (abs(pointTurnPID.getError()) < 50)
        {
            time++;
            wait(2);
        }

        wait(5);
    }

    left(0);
    right(0);
}

//******************************************************************************
//*************************Sweep Functions**************************************
/**
 * @brief Standard Sweep Right
 * 
 * @param degrees 
 * @param rightSideSpeed 
 */
void sweepRight(int degrees, int rightSideSpeed)
{

    int time = 0;

    while (time < 25)
    {

        right(rightSideSpeed);
        left(sweepTurnPID.getOutput(degrees, getTheta()));

        if (abs(sweepTurnPID.getError()) < 2.5)
        {
            time++;
            wait(2);
        }

        wait(5);
    }

    left(0);
    right(0);
}

/**
 * @brief Sweep Right for Fluid Movements
 * 
 * @param degrees 
 * @param rightSideSpeed 
 * @param errorThreshhold 
 */
void sweepRight(int degrees, int rightSideSpeed, int errorThreshhold)
{

    while (getTheta() < degrees - errorThreshhold)
    {
        left(sweepTurnPIDFluid.getOutput(degrees, getTheta()));
        right(rightSideSpeed);
    }
}

/**
 * @brief Standard Sweep Left
 * 
 * @param degrees 
 * @param leftSideSpeed 
 */
void sweepLeft(int degrees, int leftSideSpeed)
{
    int time = 0;

    while (time < 25)
    {
        left(leftSideSpeed);

        right(-sweepTurnPID.getOutput(degrees, getTheta()));

        if (abs(sweepTurnPID.getError()) < 2.5)
        {
            time++;
            wait(2);
        }

        wait(5);
    }
    left(0);
    right(0);
}

/**
 * @brief Sweep Left for Fluid Movements
 * 
 * @param degrees 
 * @param leftSideSpeed 
 * @param errorThreshhold 
 */
void sweepLeft(int degrees, int leftSideSpeed, int errorThreshhold)
{

    while (getTheta() > degrees + errorThreshhold)
    {
        right(-sweepTurnPIDFluid.getOutput(degrees, getTheta()));
        left(leftSideSpeed);
    }
}

//******************************************************************************
//******************************************************************************

void sweepRightBack(int degrees, int leftSideSpeed)
{

    int time = 0;

    while (time < 50)
    {

        left(leftSideSpeed);
        right(-sweepTurnPID.getOutput(degrees, getTheta()));

        if (abs(sweepTurnPID.getError()) < 2.5)
        {
            time++;
            wait(2);
        }

        wait(5);
    }

    left(0);
    right(0);
}

void sweepRightBack(int degrees, int leftSideSpeed, int errorThreshhold)
{

    while (getTheta() < degrees - errorThreshhold)
    {
        right(sweepTurnPIDFluid.getOutput(degrees, getTheta()));
        left(leftSideSpeed);
    }
}

void sweepLeftBack(int degrees, int rightSideSpeed)
{
    int time = 0;

    while (time < 50)
    {
        right(rightSideSpeed);

        left(sweepTurnPID.getOutput(degrees, getTheta()));

        if (abs(sweepTurnPID.getError()) < 2.5)
        {
            time++;
            wait(2);
        }

        wait(5);
    }
    left(0);
    right(0);
}

void sweepLeftBack(int degrees, int rightSideSpeed, int errorThreshhold)
{

    while (getTheta() > degrees + errorThreshhold)
    {
        left(sweepTurnPIDFluid.getOutput(degrees, getTheta()));
        right(rightSideSpeed);
    }
}