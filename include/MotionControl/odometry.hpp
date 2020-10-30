#include "main.h"

/**
 * @brief odom declarations
 * 
 */
extern pros::ADIEncoder R;
extern pros::ADIEncoder L;
extern pros::ADIEncoder S;

extern const double WHEEL_DIAMETER;
extern const int TICS_PER_REVOLUTION;
extern const double LEFT_OFFSET;
extern const double RIGHT_OFFSET;
extern const double REAR_OFFSET;

void odometryStartTask(bool reset = true);
void odometryStopTask();

void calculate_position(void *parameter);
double getX();
double getY();
double getTheta();
double getThetaRadians();
void resetOdometry();
void setTheta(int degrees);
void setCoordinates(int x, int y, int theta);
