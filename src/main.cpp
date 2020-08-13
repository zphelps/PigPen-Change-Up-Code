#include "main.h"

void disabled() {}

void competition_initialize() {}

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Controller partner(pros::E_CONTROLLER_PARTNER);

void opcontrol()
{
	while (true)
	{
		if (master.get_digital(DIGITAL_A) && !pros::competition::is_connected())
		{
			autonomous();
		}
		if (master.get_digital(DIGITAL_B))
		{
			setCoordinates(0, 0, 0);
		}
		driveOP();
		//xdriveOP();
		intakeOP();
		pros::delay(20);
	}
}
