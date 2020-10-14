#include "main.h"

void disabled() {}

void competition_initialize() {}

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Controller partner(pros::E_CONTROLLER_PARTNER);

void opcontrol()
{
	int driveMode = 0;
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
		if (master.get_digital(DIGITAL_DOWN))
		{
			driveMode++;
			wait(200);
		}

		if (driveMode % 2 == 0)
		{
			driveOP();
		}
		else
		{
			driveOPSlow();
		}
		//driveOP();
		//xdriveOP();
		intakeOP();
		pros::delay(20);
	}
}
