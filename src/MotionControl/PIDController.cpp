#include "main.h"

/*
    The PIDController class is a very useful tool that we have implented into our program. It allows us to significantly reduce the use of redundant code throughtout our program.
*/

/*
    This is our complete constructor for this class which takes in kP, kI, kD, and minSpeed as parameters. 
*/
PIDController::PIDController(double inKP, double inKI, double inKD, int inMinSpeed)
{
    kP = inKP;
    kI = inKI;
    kD = inKD;
    minSpeed = inMinSpeed;
}

/*
    In practice, we rarely use the kI and kD arguments in our movements, so we have a constructor for this class that only takes in kP and minSpeed as parameters.
*/
PIDController::PIDController(double inKP, int inMinSpeed)
{
    kP = inKP;
    minSpeed = inMinSpeed;
}

/*
    This is the core of the controller. This method allows us to get the desired power ooutput to any motor based on only the target position and the current value of the sensor we are using to measure rotation.
*/
int PIDController::getOutput(int target, int current)
{

    double prevError = 0;

    error = (target - current);

    int integral = integral + error;

    if (error == 0)
    {
        integral = 0;
    }

    if (abs(error) > 40)
    {
        integral = 0;
    }

    int derivative = error - prevError;

    prevError = error;

    //power output calculation
    int power = (error * kP + derivative * kI + integral * kD);

    //Power defaults to our minimum speed if power generates a value < minSpeed
    if (power <= minSpeed && power <= 0 && fabs(error) < 0.5)
    {
        return power;
    }
    else if (power >= -minSpeed && power <= 0 && fabs(error) < 0.5)
    {
        return power;
    }
    else if (power <= minSpeed && power >= 0)
    {
        power = minSpeed;
    }
    else if (power >= -minSpeed && power <= 0)
    {
        power = -minSpeed;
    }
    return power;
}

int PIDController::getOutput(int error)
{

    double prevError = 0;

    int integral = integral + error;

    if (error == 0)
    {
        integral = 0;
    }

    if (abs(error) > 40)
    {
        integral = 0;
    }

    int derivative = error - prevError;

    prevError = error;

    //power output calculation
    int power = (error * kP + derivative * kI + integral * kD);

    //Power defaults to our minimum speed if power generates a value < minSpeed
    if (power <= minSpeed && power <= 0 && fabs(error) < 0.5)
    {
        return power;
    }
    else if (power >= -minSpeed && power <= 0 && fabs(error) < 0.5)
    {
        return power;
    }
    else if (power <= minSpeed && power >= 0)
    {
        power = minSpeed;
    }
    else if (power >= -minSpeed && power <= 0)
    {
        power = -minSpeed;
    }
    return power;
}

//Feedback
double PIDController::getError()
{
    return error;
}
