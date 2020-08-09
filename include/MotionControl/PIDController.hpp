//PID Controller class header
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
    PIDController(double kP, double kI, double kD, int inMinSpeed);
    PIDController(double kP, int inMinSpeed);
    int getOutput(int target, int current);
    int getOutput(int error);
    double getError();
};
