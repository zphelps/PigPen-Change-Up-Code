#include "main.h"

//Tracking Encoders Constructors
pros::ADIEncoder R('E', 'F', false);
pros::ADIEncoder L('C', 'D', false);
pros::ADIEncoder S('A', 'B', true);

const double WHEEL_DIAMETER = 2.75;
const int TICS_PER_REVOLUTION = 360;
const double LEFT_OFFSET = 5.95;  //5.9
const double RIGHT_OFFSET = 5.95; //5.9
const double REAR_OFFSET = 5.75;

double xglobal;
double yglobal;

double thetaInRadians = 0;
double thetaInDegrees = 0;
double thetaInDegreesUncorrected = 0;

//odom task definition
pros::Task *odometryTask = nullptr;

void odometryStartTask(bool reset)
{
    if (reset)
    {
        resetOdometry();
    }
    odometryTask = new pros::Task(calculate_position);
}

void odometryStopTask()
{
    if (odometryTask != nullptr)
    {
        //Remove old odom task
        odometryTask->remove();
        delete odometryTask;
        //set to null
        odometryTask = nullptr;
    }
}

void calculate_position(void *parameter)
{
    double prevL = 0;
    double prevR = 0;
    double prevS = 0;

    pros::lcd::initialize();

    while (1)
    {

        double leftEncoderInches = L.get_value() * PI * WHEEL_DIAMETER / TICS_PER_REVOLUTION;
        double rightEncoderInches = R.get_value() * PI * WHEEL_DIAMETER / TICS_PER_REVOLUTION;
        double backEncoderInches = S.get_value() * PI * WHEEL_DIAMETER / TICS_PER_REVOLUTION;

        double currentL = leftEncoderInches;
        double currentR = rightEncoderInches;
        double currentS = backEncoderInches;

        double deltaL = currentL - prevL;
        double deltaR = currentR - prevR;
        double deltaS = currentS - prevS;

        prevL = currentL;
        prevR = currentR;
        prevS = currentS;

        //Theta Calculation
        double deltaTheta = (deltaL - deltaR) / (LEFT_OFFSET + RIGHT_OFFSET);
        thetaInRadians += deltaTheta;
        thetaInDegrees = thetaInRadians * 180 / PI;
        thetaInDegreesUncorrected = thetaInDegrees;
        thetaInDegrees = thetaInDegrees - 360 * floor(thetaInDegrees / 360); //Angle Wrap for Display of theta
        if (thetaInDegrees < 0)
        {
            thetaInDegrees = 360 + thetaInDegrees;
        }

        //X & Y Calculation
        double chord;
        double chord2;

        if (deltaTheta == 0)
        {
            chord = deltaR;
            chord2 = deltaS;
        }
        else
        {
            double r = deltaR / deltaTheta;
            double sinI = sin(deltaTheta / 2);
            chord = ((r + RIGHT_OFFSET) * sinI) * 2.0;

            double r2 = deltaS / deltaTheta;
            chord2 = ((r2 + REAR_OFFSET) * sinI) * 2.0;
        }

        double p = (deltaTheta / 2) + thetaInRadians;
        double cosP = cos(p);
        double sinP = sin(p);

        xglobal = xglobal + (chord * cosP);
        yglobal = yglobal - (chord * sinP);

        xglobal = xglobal + (chord2 * -sinP);
        yglobal = yglobal - (chord2 * cosP);

        wait(1);

        //LCD Feedback
        pros::lcd::print(1, "Theta: %f", thetaInDegrees);

        pros::lcd::print(2, "X: %f", xglobal);
        pros::lcd::print(3, "Y: %f", yglobal);

        pros::lcd::print(4, "R: %d", R.get_value());
        pros::lcd::print(5, "L: %d", L.get_value());
        // pros::lcd::print(6, "S: %d", S.get_value());
    }
}

double getTheta()
{
    return thetaInDegreesUncorrected;
}

double getThetaRadians()
{
    return thetaInRadians;
}

double getX()
{
    return xglobal;
}

double getY()
{
    return yglobal;
}

void resetOdometry()
{
    thetaInRadians = 0;
    xglobal = 0;
    yglobal = 0;
}
//Resets thetaInDegrees to @param degrees
void setTheta(int degrees)
{
    thetaInRadians = degrees * PI / 180;
}

void setCoordinates(int x, int y, int theta)
{
    thetaInRadians = theta * PI / 180;
    xglobal = x;
    yglobal = y;
}
