
class PIDController
{
private:
    double kP;
    double kI;
    double kD;
    int minSpeed;
    double accelStep;
    int error;

public:
    PIDController(double kP, double kI, double kD, int inMinSpeed, double accelStep);
    PIDController(double kP, double kI, double kD, int inMinSpeed);
    int getOutput(int target, int current);
    double getError();
};
