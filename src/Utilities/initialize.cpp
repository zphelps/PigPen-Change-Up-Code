#include "main.h"

int autonIndex = 0;
pros::ADIDigitalIn forwardSelectorLimit({21, 'C'});
pros::ADIDigitalIn backwardSelectorLimit({21, 'D'});

const int autoCount = 7;
const char *autoNames[autoCount] = {
    "Home Row - Right Side - No Cycle",
    "Home Row - Right Side - Cycle",
    "Home Row - Left Side - No Cycle",
    "Home Row - Left Side - Cycle",
    "Two Goal - Right",
    "Two Goal - Left",
    "Programming Skills"};

void autonSelector(void *parameter)
{
    wait(500); //Fix Bug that starts index at 2
    pros::lcd::print(6, "%s", autoNames[autonIndex]);

    while (true)
    {

        if (forwardSelectorLimit.get_value())
        {
            autonIndex = autonIndex + 1;
            if (autonIndex == autoCount)
                autonIndex = 0;

            pros::lcd::print(6, "%s", autoNames[autonIndex]);
            wait(300);
        }
        else if (backwardSelectorLimit.get_value())
        {
            autonIndex = autonIndex - 1;
            if (autonIndex == autoCount)
                autonIndex = 0;
            pros::lcd::print(6, "%s", autoNames[autonIndex]);
            wait(300);
        }
    }
}

void initialize()
{
    pros::lcd::initialize();

    //Odom Init
    odometryStartTask();

    //Initialize Vision Sensor
    initVision();

    //Auton Selector Init
    pros::Task lcd_task(autonSelector);
    pros::lcd::set_text(6, "<Select an Autonomous>");
    lcd_task.set_priority(LV_TASK_PRIO_LOW); //Task infinite loop bug fix
    if (pros::competition::is_autonomous())
    {
        lcd_task.remove();
    }
}