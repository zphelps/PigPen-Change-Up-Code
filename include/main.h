/**
 * @file main.h
 * @author Zach Phelps
 * @brief 
 * @version 6.8.42
 * @date 2020.10.30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES

#define PROS_USE_LITERALS

#include "okapi/api.hpp"
#include "api.h"

using namespace okapi;

#include "Subsystems/drive.hpp"
#include "Subsystems/intake.hpp"

#include "Utilities/misc.hpp"
#include "Utilities/Point.hpp"

#include "MotionControl/odometry.hpp"
#include "MotionControl/PIDController.hpp"

extern pros::Controller master;
extern pros::Controller partner;
extern int autonIndex;

#ifdef __cplusplus
extern "C"
{
#endif
    void autonomous(void);
    void initialize(void);
    void disabled(void);
    void competition_initialize(void);
    void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif // _PROS_MAIN_H_
