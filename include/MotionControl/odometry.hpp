#include "main.h"

extern pros::ADIEncoder R;
extern pros::ADIEncoder L;
extern pros::ADIEncoder S;

void calculate_position(void *parameter);
double getX();
double getY();
double getTheta();
double getThetaRadians();
void setTheta(int degrees);
void setCoordinates(int x, int y, int theta);
