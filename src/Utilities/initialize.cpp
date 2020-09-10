#include "main.h"

int autonIndex = 0;

const int autoCount = 5;
const char *autoNames[autoCount] = {
    "Home Row - Right Side - No Cycle",
    "Home Row - Right Side - Cycle",
    "Home Row - Left Side - No Cycle",
    "Two Goal Right",
    "Programming Skills"};

// void on_right_button()
// {
//     static bool pressed = false;
//     pressed = !pressed;
//     if (pressed)
//     {
//         autonIndex++;
//         if (autonIndex == autoCount)
//             autonIndex = 0;
//         pros::lcd::print(5, "%s", autoNames[autonIndex]);
//     }
// }

// void on_left_button()
// {
//     static bool pressed = false;
//     pressed = !pressed;
//     if (pressed)
//     {
//         autonIndex++;
//         if (autonIndex == autoCount)
//             autonIndex = 0;
//         pros::lcd::print(5, "%s", autoNames[autonIndex]);
//     }
// }

void autonSelector(void *parameter)
{
    wait(500); //Fix Bug that starts index at 2
    pros::lcd::print(6, "%s", autoNames[autonIndex]);

    while (true)
    {

        if (indexerLimit.get_value())
        {
            autonIndex = autonIndex + 1;
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

    pros::Task calcPos(calculate_position); //Begin position tracking
    pros::Task lcd_task(autonSelector);

    pros::lcd::set_text(6, "<Select an Autonomous>");

    lcd_task.set_priority(LV_TASK_PRIO_LOW); //Task infinite loop bug fix

    if (pros::competition::is_autonomous())
    {
        lcd_task.remove();
    }

    //initializeInertialSensor();
}