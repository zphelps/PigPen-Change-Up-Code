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

    double DEFAULT_KP;
    double DEFAULT_KI;
    double DEFAULT_KD;
    double DEFAULT_MINSPEED;

public:
    PIDController(double kP, double kI, double kD, int inMinSpeed);
    PIDController(double kP, int inMinSpeed);
    int getOutput(int target, int current);
    int getOutput(int error);
    void setGains(double kP, double kI, double kD, int minSpeed);
    void resetGainsToDefaults();
    bool gainsAreAtDefaults();
    double getError();
};
