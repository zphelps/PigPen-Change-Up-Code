#include "main.h"

void on_center_button()
{
    static bool pressed = false;
    pressed = !pressed;
    if (pressed)
    {
        pros::lcd::set_text(2, "I was pressed!");
    }
    else
    {
        pros::lcd::clear_line(2);
    }
}

void initialize()
{
    pros::lcd::initialize();

    pros::Task calcPos(calculate_position); //Begin position tracking

    //initializeInertialSensor();

    pros::lcd::register_btn1_cb(on_center_button);
}