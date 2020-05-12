
#include "main.h"

PIDController::PIDController(double inKP, double inKI, double inKD, int inMinSpeed, double inAccelStep)
{
    kP = inKP;
    kI = inKI;
    kD = inKD;
    minSpeed = inMinSpeed;
    accelStep = inAccelStep;
}

PIDController::PIDController(double inKP, double inKI, double inKD, int inMinSpeed)
{
    kP = inKP;
    kI = inKI;
    kD = inKD;
    minSpeed = inMinSpeed;
}

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

    int power = (error * kP + derivative * kI + integral * kD);

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

double PIDController::getError()
{
    return error;
}
