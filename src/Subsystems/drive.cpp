#include "main.h"

//Motors - 8
pros::Motor leftFront(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor leftBack(2, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rightFront(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); //Orange -5
pros::Motor rightBack(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

//Inertial -> Not currently used
pros::Imu inertial(3);

//PID Gains
double MOVE_KP = 0.15;
double MOVE_KI = 0;
double MOVE_KD = 0;
double MOVE_ACCEL_STEP = 5;
int START_SPEED = 127;
int MOVE_MIN_SPEED = 20; //Re-caculated everytime we make large changes to the robot's weight or drive mechanics
int MOVE_MIN_SPEED_FLUID = 75;

double TURN_KP = 1.25;
double TURN_KI = 0;
double TURN_KD = 0;

int POINT_TURN_MIN_SPEED = 15; //Re-caculated everytime we make large changes to the robot's weight or drive mechanics

const double WHEEL_DIAMETER = 2.75;
const int TICS_PER_REVOLUTION = 360;
const double LEFT_OFFSET = 5.87;  //5.95
const double RIGHT_OFFSET = 5.87; //5.95
const double REAR_OFFSET = 5.75;

//Acceleration Step Variables
int rightSideSpeed = 0;
int leftSideSpeed = 0;

//**********************Drive PIDControllers************************************

PIDController movePID(MOVE_KP, MOVE_KI, MOVE_KD, MOVE_MIN_SPEED);

PIDController movePIDFluid(MOVE_KP, MOVE_KI, MOVE_KD, MOVE_MIN_SPEED_FLUID);

PIDController moveToCoordPID(6, MOVE_KI, MOVE_KD, 30);

PIDController moveToCoordFluidPID(6, MOVE_KI, MOVE_KD, 75);

PIDController pointTurnPID(TURN_KP, TURN_KI, TURN_KD, POINT_TURN_MIN_SPEED);

PIDController sweepTurnPID = PIDController(1.25, 0, 0, 35);

PIDController sweepTurnPIDFluid(1.75, 0, 0, 80);

//**************************Helper Functions************************************
/**
 * @brief This function makes it easier to refer to the left side of the drive base and cuts down on the amount of code needed
 * 
 * @param speed 
 */
void left(int speed)
{
    leftFront.move(speed);
    leftBack.move(speed);
}
/**
 * @brief This function makes it easier to refer to the right side of the drive base and cuts down on the amount of code needed
 * 
 * @param speed 
 */
void right(int speed)
{
    rightFront.move(speed);
    rightBack.move(speed);
}
/**
 * @brief This function reduces the need for telling both the right and left motor to move each time we want the robot to go forward
 * 
 * @param speed 
 */
void drive(int speed)
{
    left(speed);
    right(speed);
}
/**
 * @brief Used for gradual acceleration in both movements and turns. The accel jerk can be manipulated by passing in different values for accelStep
 * 
 * @param speed 
 * @param accelStep 
 */
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
/**
 * @brief Used for gradual acceleration in both movements and turns. The accel jerk can be manipulated by passing in different values for accelStep
 * 
 * @param speed 
 * @param accelStep 
 */
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
/**
 * @brief Used for gradual acceleration in both movements and turns. The accel jerk can be manipulated by passing in different values for accelStep
 * 
 * @param speed 
 * @param accelStep 
 */
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
/**
 * @brief Used for gradual acceleration in both movements and turns. The accel jerk can be manipulated by passing in different values for accelStep
 * 
 * @param speed 
 * @param accelStep 
 */
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
/**
 * @brief Set all drive motors' brakemode to E_MOTOR_BRAKE_BRAKE
 * 
 */
void brake()
{
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}
/**
 * @brief Set all drive motors' brakemode to E_MOTOR_BRAKE_COAST
 * 
 */
void coast()
{
    rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
/**
 * @brief This function moves the robot for an amount of time instead of a distance, to ensure that the robot does not get stuck in a while loop
 * 
 * @param time 
 * @param speed 
 */
void timedDrive(int time, int speed)
{
    right(speed);
    left(speed);
    wait(time);
    right(speed);
    left(speed);
}

/*
    The inertial sensor is used as a backup if the position tracking system's theta value is considerably different from that of the inertial's reading.
    If so, the postion tracking system is reset to the inertial sensor's reading.
*/
void checkInertial(int expectedAngle)
{
    if (abs(inertial.get_rotation() - expectedAngle) < abs(getTheta() - expectedAngle))
    {
        master.print(0, 0, "%s", "Reset Theta");
        setTheta(inertial.get_rotation());
    }
    master.print(0, 0, "%s", "Didn't Reset");
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
    leftFront.move(master.get_analog(ANALOG_LEFT_Y) * 1);
    leftBack.move(master.get_analog(ANALOG_LEFT_Y) * 1);
    rightFront.move(master.get_analog(ANALOG_RIGHT_Y) * 1);
    rightBack.move(master.get_analog(ANALOG_RIGHT_Y) * 1);
}

//Separate x drive model control allows for modular drive code
void xdriveOP()
{
    rightFront.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X));
    rightBack.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X));
    leftFront.move(master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X));
    leftBack.move(master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X));
}

//******************************************************************************
//**************************Move Functions**************************************
/**
 * @brief This function uses the parameters below to move the robot a certain distance at a chosen heading in a way that reduces the amount of code in the main skills program
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
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
 * @brief This function is an override code for the other move function, which allows us to change the kP, minSpeed, and correction factor 
 * in one of the move functions without having to worry about changing every single one
 * 
 * Changing the kP and minspeed allow us to make turns more accurate and efficient whenever we need them to be
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param kP 
 * @param minSpeed 
 * @param correction 
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
 * @brief This function does the same thing as the move function, except that it will cause the motors to not come to a complete stop before the next direction that we give the robot
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
 * @brief This function uses the position tracking system to move to a desired Y-coordinate, causing the forward movements to be more accurate
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param fluid 
 */
void moveToYCoord(int distance, int heading, int accelStep, bool fluid)
{

    if (fluid)
    {
        moveToYCoordFluid(distance, heading, accelStep);
    }

    double correctionMultiplier = 0.2;

    double target = distance;

    if (getY() < target)
    {
        while (getY() < target && !fluid)
        {

            double PIDSpeed = (moveToCoordPID.getOutput(target, getY()));

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
    else if (getY() > target)
    {
        while (getY() > target && !fluid)
        {

            double PIDSpeed = (moveToCoordPID.getOutput(target, getY()));

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
    if (!fluid)
    {
        right(0);
        left(0);
    }

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}
/**
 * @brief This function is the same as the moveToYCoord function, except that it incorporates the idea of the fluid movement mentioned in the moveFluid function comment
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 */
void moveToYCoordFluid(int distance, int heading, int accelStep)
{
    double correctionMultiplier = 0.4; //0.2

    double target = distance;

    if (getY() < target)
    {
        while (getY() < target)
        {

            double PIDSpeed = (moveToCoordFluidPID.getOutput(target, getY()));

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
    else if (getY() > target)
    {
        while (getY() > target)
        {

            double PIDSpeed = (moveToCoordFluidPID.getOutput(target, getY()));

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
}
/**
 * @brief This function is an override function for the other moveToYCoord function, allowing us to change the kP minSpeed and correction factor to allow us to make the movements
 * more efficient and accurate when needed
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param kP 
 * @param minSpeed 
 * @param correction 
 */
void moveToYCoord(int distance, int heading, int accelStep, double kP, int minSpeed, double correction)
{

    PIDController tempMovePID(kP, 0, 0, minSpeed);

    double correctionMultiplier = correction;

    double target = distance;
    if (getY() < target)
    {
        while (getY() < target)
        {

            double PIDSpeed = (tempMovePID.getOutput(target, getY()));

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
    else if (getY() > target)
    {
        while (getY() > target)
        {

            double PIDSpeed = (tempMovePID.getOutput(target, getY()));

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

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}
/**
 * @brief This function uses the position tracking system to move to a desired X-coordinate, causing the forward movements to be more accurate
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param fluid 
 */
void moveToXCoord(int distance, int heading, int accelStep, bool fluid)
{

    if (fluid)
    {
        moveToXCoordFluid(distance, heading, accelStep);
    }

    double correctionMultiplier = 0.2;

    double target = distance;

    if (getX() < target)
    {
        while (getX() < target && !fluid)
        {

            double PIDSpeed = (moveToCoordPID.getOutput(target, getX()));

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
    else if (getX() > target)
    {
        while (getX() > target && !fluid)
        {

            double PIDSpeed = (moveToCoordPID.getOutput(target, getX()));

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
    if (!fluid)
    {
        right(0);
        left(0);
    }

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}
/**
 * @brief This function is the same as the moveToXCoord function, except that it incorporates the idea of the fluid movement mentioned in the moveFluid function comment
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 */
void moveToXCoordFluid(int distance, int heading, int accelStep)
{
    double correctionMultiplier = 0.4; //0.2

    double target = distance;

    if (getX() < target)
    {
        while (getX() < target)
        {

            double PIDSpeed = (moveToCoordFluidPID.getOutput(target, getX()));

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
    else if (getX() > target)
    {
        while (getX() > target)
        {

            double PIDSpeed = (moveToCoordFluidPID.getOutput(target, getX()));

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
}
/**
 * @brief This function is an override function for the other moveToXCoord function, allowing us to change the kP minSpeed and correction factor to allow us to make the movements
 * more efficient and accurate when needed
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param kP 
 * @param minSpeed 
 * @param correction 
 */
void moveToXCoord(int distance, int heading, int accelStep, double kP, int minSpeed, double correction)
{

    PIDController tempMovePID(kP, 0, 0, minSpeed);

    double correctionMultiplier = correction;

    double target = distance;
    if (getX() < target)
    {
        while (getX() < target)
        {

            double PIDSpeed = (tempMovePID.getOutput(target, getX()));

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
    else if (getX() > target)
    {
        while (getX() > target)
        {

            double PIDSpeed = -(tempMovePID.getOutput(target, getX()));

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

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}

void moveAboslute(double x, double y, int accelStep)
{
    _Point targetPoint = _Point(x, y);

    double correctionMultiplier = 0.2;

    double distanceToTarget = sqrt(pow(targetPoint.x - getX(), 2) + pow(targetPoint.y - getY(), 2));

    double heading = atan2(targetPoint.y - getY(), targetPoint.x - getX()) * 180 / PI;

    master.print(0, 0, "%f", distanceToTarget);

    while (distanceToTarget > 2)
    {
        distanceToTarget = sqrt(pow(targetPoint.x - getX(), 2) + pow(targetPoint.y - getY(), 2));

        // if (targetPoint.x == getX())
        // {
        //     heading = 90;
        // }
        // else
        // {
        //     heading = (atan2(targetPoint.x - getX(), targetPoint.y - getY()) * 180 / PI) * -1;
        // }

        heading = (atan2(targetPoint.x - getX(), targetPoint.y - getY()) * 180 / PI) * -1;

        double PIDSpeed = 90; //movePID.getOutput(distanceToTarget);

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

    left(0);
    right(0);

    leftSideSpeed = 0;
    rightSideSpeed = 0;

    //turn(heading);
}
/**
 * @brief This function is used to move the robot backward at a specific heading for a specific amount of inches, it basically is the move function but reversed
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
 * @brief This is an override function for the moveBack function, which allows us to change the kP and minSpeed, giving a similar result to the move override function, but reversed
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param kP 
 * @param minSpeed 
 * @param correction 
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
 * @brief This function does the same thing as the moveBack function, except that it will cause the motors to not come to a complete stop before the next direction that we give the robot
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
/**
 * @brief This function is the same as the moveToYCoord function, but is used for when the robot is ahead of the Y-coordinate we want to be at, requiring it to move backward
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param fluid 
 */
void moveBackToYCoord(int distance, int heading, int accelStep, bool fluid)
{

    if (fluid)
    {
        moveFluid(distance, heading, accelStep);
    }

    double correctionMultiplier = 0.2;

    double target = distance;

    if (getY() < target)
    {
        while (getY() < target && !fluid)
        {

            double PIDSpeed = (-moveToCoordPID.getOutput(target, getY()));

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
    else if (getY() > target)
    {
        while (getY() > target && !fluid)
        {

            double PIDSpeed = (-moveToCoordPID.getOutput(target, getY()));

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
    if (!fluid)
    {
        right(0);
        left(0);
    }

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}
/**
 * @brief This function does the same thing as the moveBackToYCoord function, except that it will cause the motors to not come to a complete stop before the next direction that we give the robot
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 */
void moveBackToYCoordFluid(int distance, int heading, int accelStep)
{
    double correctionMultiplier = 0.4; //0.2

    double target = distance;

    if (getY() < target)
    {
        while (getY() < target)
        {

            double PIDSpeed = (-moveToCoordFluidPID.getOutput(target, getY()));

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
    else if (getY() > target)
    {
        while (getY() > target)
        {

            double PIDSpeed = (-moveToCoordFluidPID.getOutput(target, getY()));

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
}
/**
 * @brief This is an override function for the moveBackToYCoord function that allows us to change the kP and minSpeed to make the backward movement more accurate and efficient if needed
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param kP 
 * @param minSpeed 
 * @param correction 
 */
void moveBackToYCoord(int distance, int heading, int accelStep, double kP, int minSpeed, double correction)
{

    PIDController tempMovePID(kP, 0, 0, minSpeed);

    double correctionMultiplier = correction;

    double target = distance;
    if (getY() < target)
    {
        while (getY() < target)
        {

            double PIDSpeed = (-tempMovePID.getOutput(target, getY()));

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
    else if (getY() > target)
    {
        while (getY() > target)
        {

            double PIDSpeed = (tempMovePID.getOutput(target, getY()));

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

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}
/**
 * @brief This function is the same as the moveToXCoord function, but is used for when the robot is ahead of the Y-coordinate we want to be at, requiring it to move backward
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param fluid 
 */
void moveBackToXCoord(int distance, int heading, int accelStep, bool fluid)
{

    if (fluid)
    {
        moveBackToXCoordFluid(distance, heading, accelStep);
    }

    double correctionMultiplier = 0.2;

    double target = distance;

    if (getX() < target)
    {
        while (getX() < target && !fluid)
        {

            double PIDSpeed = (-moveToCoordPID.getOutput(target, getX()));

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
    else if (getX() > target)
    {
        while (getX() > target && !fluid)
        {

            double PIDSpeed = (-moveToCoordPID.getOutput(target, getX()));

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
    if (!fluid)
    {
        right(0);
        left(0);
    }

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}

/**
 * @brief This function does the same thing as the moveBackToXCoord function, except that it will cause the motors to not come to a complete stop before the next direction that we give the robot
 * 
 * @param distance 
 * @param heading 
 * @param accelStep
 * 
 *  
 */
void moveBackToXCoordFluid(int distance, int heading, int accelStep)
{
    double correctionMultiplier = 0.4; //0.2

    double target = distance;

    if (getX() < target)
    {
        while (getX() < target)
        {

            double PIDSpeed = (-moveToCoordFluidPID.getOutput(target, getX()));

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
    else if (getX() > target)
    {
        while (getX() > target)
        {

            double PIDSpeed = (-moveToCoordFluidPID.getOutput(target, getX()));

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
}
/**
 * @brief This is an override function for the moveBackToXCoord function that allows us to change the kP and minSpeed to make the backward movement more accurate and efficient if needed
 * 
 * @param distance 
 * @param heading 
 * @param accelStep 
 * @param kP 
 * @param minSpeed 
 * @param correction 
 */
void moveBackToXCoord(int distance, int heading, int accelStep, double kP, int minSpeed, double correction)
{

    PIDController tempMovePID(kP, 0, 0, minSpeed);

    double correctionMultiplier = correction;

    double target = distance;
    if (getX() < target)
    {
        while (getX() < target)
        {

            double PIDSpeed = (-tempMovePID.getOutput(target, getX()));

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
    else if (getX() > target)
    {
        while (getX() > target)
        {

            double PIDSpeed = (tempMovePID.getOutput(target, getX()));

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

    leftSideSpeed = 0;
    rightSideSpeed = 0;
}

//******************************************************************************
//**************************Turn Functions**************************************
/**
 * @brief This function uses the getTheta data that we collect at the beginning and sometimes in the middle of each run to turn to an accurate angle
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

        right(-pointTurnPID.getOutput(degrees, inertial.get_rotation()));
        left(pointTurnPID.getOutput(degrees, inertial.get_rotation()));

        if (fabs(pointTurnPID.getError()) < 2.5)
        {
            time++;
            wait(5);
        }

        wait(5);
    }

    left(0);
    right(0);
}

//******************************************************************************
//*************************Sweep Functions**************************************
/**
 * @brief This function is similar to the turn function, except that both sides of the drive base are moving at the same time, allowing for a softer and slighter movement that moves forward
 * as it is turning
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
 * @brief This is an override function that allows us to change the kP and minSpeed to allow for a more accurate and efficient turn when needed
 * 
 * @param degrees 
 * @param rightSideSpeed 
 * @param kP 
 * @param minspeed 
 */
void sweepRight(int degrees, int rightSideSpeed, double kP, int minspeed)
{
    PIDController tempSweepRightPID(kP, 0, 0, minspeed);
    int time = 0;

    while (time < 25)
    {

        right(rightSideSpeed);
        left(tempSweepRightPID.getOutput(degrees, getTheta()));

        if (abs(tempSweepRightPID.getError()) < 2.5)
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
 * @brief This is another override function for the sweepRight function, that includes error threshold to correct the angle of the turn if needed
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
 * @brief This is another override function for the sweepRight function that incorporates the other two override functions
 * 
 * @param degrees 
 * @param rightSideSpeed 
 * @param errorThreshhold 
 * @param kP 
 * @param minSpeed 
 */
void sweepRight(int degrees, int rightSideSpeed, int errorThreshhold, double kP, int minSpeed)
{
    PIDController tempSweepTurnPIDFluid = PIDController(kP, 0, 0, minSpeed);

    while (getTheta() < degrees - errorThreshhold)
    {
        left(tempSweepTurnPIDFluid.getOutput(degrees, getTheta()));
        right(rightSideSpeed);
    }
}
/**
 * @brief This function is similar to the turn function, except that both sides of the drive base are moving at the same time, allowing for a softer and slighter movement that moves forward
 * as it is turning
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
 * @brief This is another override function for the sweepLeft function, that includes error threshold to correct the angle of the turn if needed
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
/**
 * @brief This is another override function for the sweepLeft function that incorporates the other override function plus allowing us to change the kP and minSpeed for accuracy
 * 
 * @param degrees 
 * @param leftSideSpeed 
 * @param errorThreshhold 
 * @param kP 
 * @param minSpeed 
 */
void sweepLeft(int degrees, int leftSideSpeed, int errorThreshhold, double kP, int minSpeed)
{
    PIDController tempSweepTurnPIDFluid = PIDController(kP, 0, 0, minSpeed);

    while (getTheta() > degrees + errorThreshhold)
    {
        right(-tempSweepTurnPIDFluid.getOutput(degrees, getTheta()));
        left(leftSideSpeed);
    }
}

//******************************************************************************
//******************************************************************************
/**
 * @brief This function is the same as the sweepRight function except that it is used for turning while moving backward
 * 
 * @param degrees 
 * @param leftSideSpeed 
 */
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
/**
 * @brief This is an override function for the sweepRightBack function, that includes error threshold to correct the angle of the turn if needed
 * 
 * @param degrees 
 * @param leftSideSpeed 
 * @param errorThreshhold 
 */
void sweepRightBack(int degrees, int leftSideSpeed, int errorThreshhold)
{

    while (getTheta() < degrees - errorThreshhold)
    {
        right(sweepTurnPIDFluid.getOutput(degrees, getTheta()));
        left(leftSideSpeed);
    }
}
/**
 * @brief This is an override function that allows us to change the kP and minSpeed to allow for a more accurate and efficient turn when needed
 * 
 * @param degrees 
 * @param rightSideSpeed 
 * @param kP 
 * @param minSpeed 
 */
void sweepRightBack(int degrees, int rightSideSpeed, double kP, int minSpeed)
{

    PIDController tempSweepPID(kP, 0, 0, minSpeed);

    int time = 0;

    while (time < 50)
    {
        left(rightSideSpeed);

        right(-tempSweepPID.getOutput(degrees, getTheta()));

        if (abs(tempSweepPID.getError()) < 2.5)
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
 * @brief This is another override function for the sweepRightBack function that incorporates the other two override functions
 * 
 * @param degrees 
 * @param leftSideSpeed 
 * @param errorThreshhold 
 * @param kP 
 * @param minSpeed 
 */
void sweepRightBack(int degrees, int leftSideSpeed, int errorThreshhold, double kP, int minSpeed)
{
    PIDController tempSweepTurnPIDFluid = PIDController(kP, 0, 0, minSpeed);
    while (getTheta() < degrees - errorThreshhold)
    {
        right(-tempSweepTurnPIDFluid.getOutput(degrees, getTheta()));
        left(leftSideSpeed);
    }
}
/**
 * @brief This function is the same as the sweepLeftBack function except that it is used for turning while moving backward
 * 
 * @param degrees 
 * @param rightSideSpeed 
 */
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
/**
 * @brief This is an override function that allows us to change the kP and minSpeed to allow for a more accurate and efficient turn when needed
 * 
 * @param degrees 
 * @param rightSideSpeed 
 * @param kP 
 * @param minSpeed 
 */
void sweepLeftBack(int degrees, int rightSideSpeed, double kP, int minSpeed)
{

    PIDController tempSweepPID(kP, 0, 0, minSpeed);

    int time = 0;

    while (time < 50)
    {
        right(rightSideSpeed);

        left(tempSweepPID.getOutput(degrees, getTheta()));

        if (abs(tempSweepPID.getError()) < 2.5)
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
 * @brief This is an override function for the sweepLeftBack function, that includes error threshold to correct the angle of the turn if needed
 * 
 * @param degrees 
 * @param rightSideSpeed 
 * @param errorThreshhold 
 */
void sweepLeftBack(int degrees, int rightSideSpeed, int errorThreshhold)
{

    while (getTheta() > degrees + errorThreshhold)
    {
        left(sweepTurnPIDFluid.getOutput(degrees, getTheta()));
        right(rightSideSpeed);
    }
}
/**
 * @brief This is another override function for the sweepLeftBack function that incorporates the other two override functions
 * 
 * @param degrees 
 * @param rightSideSpeed 
 * @param errorThreshhold 
 * @param kP 
 * @param minSpeed 
 */
void sweepLeftBack(int degrees, int rightSideSpeed, int errorThreshhold, double kP, int minSpeed)
{
    PIDController tempSweepTurnPIDFluid = PIDController(kP, 0, 0, minSpeed);
    while (getTheta() > degrees + errorThreshhold)
    {
        left(tempSweepTurnPIDFluid.getOutput(degrees, getTheta()));
        right(rightSideSpeed);
    }
}