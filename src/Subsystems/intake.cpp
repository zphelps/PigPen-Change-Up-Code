#include "main.h"

//Motors - 10
pros::Motor rightRollerMotor(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); //Orange - 17
pros::Motor leftRollerMotor(18, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor mainRollerMotor(19, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor indexerMotor(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

//Sensors
pros::ADIDigitalIn indexerLimit('G');
pros::ADIDigitalIn intakeLimit('H');
pros::ADILineSensor frontRollerLine({21, 'E'});
pros::ADILineSensor topRollerLine({21, 'B'});
pros::Vision vision(10);
pros::Vision vision2(8);

//Vision Sensor
#define BLUE_ID 1
#define RED_ID 2

pros::vision_signature_s_t BLUE_BALL;
pros::vision_signature_s_t RED_BALL;

int BALL_DETECTED_SIGNATURE = 2500;

static int INTAKE_MODE = 0;

void initVision()
{
    BLUE_BALL = pros::Vision::signature_from_utility(BLUE_ID, -3461, -2325, -2893, 8235, 11659, 9947, 3.200, 0);

    RED_BALL = pros::Vision::signature_from_utility(RED_ID, 6495, 8301, 7398, -311, 403, 46, 5.000, 0);

    vision.set_signature(BLUE_ID, &BLUE_BALL);
    vision.set_signature(RED_ID, &RED_BALL);

    vision2.set_signature(BLUE_ID, &BLUE_BALL);
    vision2.set_signature(RED_ID, &RED_BALL);
}

void frontRollers(int speed)
{
    rightRollerMotor.move(speed);
    leftRollerMotor.move(speed);
}

void intake(int speed)
{
    mainRollerMotor.move(speed);
}

void fullIntake(int speed)
{
    frontRollers(127);
    intake(127);
}

void indexer(int speed)
{
    indexerMotor.move(speed);
}

void indexerVelocity(int speed)
{
    indexerMotor.move_velocity(speed);
}

void intakeFullStop()
{
    intake(0);
    indexer(0);
    frontRollers(0);
}

void intakeFullReverse()
{
    intake(-127);
    indexer(-127);
    frontRollers(-127);
}

void intakeManager(void *parameter)
{
    while (1)
    {
        switch (INTAKE_MODE)
        {
        case 0:
            intake(0);
            indexer(0);
            frontRollers(0);
            break;
        case 1: //load one ball while intake is empty
            if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
            {
                intake(0);
                indexer(-127);
                frontRollers(0);
            }
            else
            {
                intake(127);
                frontRollers(127);
                indexer(50);
            }
            break;
        case 2: //load one ball while intake has a ball in it
            if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
            {
                intake(0);
                indexer(0);
                frontRollers(127);
            }
            else
            {
                intake(127);
                frontRollers(127);
                indexer(50);
            }
            break;
        }
    }
}

void setIntakeMode(int mode)
{
    INTAKE_MODE = mode;
}

void scoreOneBall()
{
    indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    while (indexerLimit.get_value() != 1)
    {
        indexerVelocity(600);
        intake(127);
    }
    intake(-20);
    wait(200); //200
    indexer(0);
    intake(0);
}

void scoreOneBallWithVision()
{
    while (indexerLimit.get_value() == 0)
    {
        indexer(127);
        intake(127);
    }
    intake(0);
    wait(200);
    indexer(0);
    while (true)
    {
        pros::vision_object_s_t obj = vision2.get_by_size(0);

        if (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID) && obj.width > 100)
        {
            break;
        }
    }
}

void scoreOneBall(int timeout)
{
    indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    int time = 0;
    while (indexerLimit.get_value() != 1 && time < timeout)
    {
        indexerVelocity(600);
        intake(127);
        wait(1);
        time++;
    }
    intake(-20);
    wait(200); //200
    indexer(0);
    intake(0);
}

void scoreOneBallInCenterGoal()
{
    while (indexerLimit.get_value() != 1)
    {
        indexer(110);
        intake(127);
    }
    intake(0);
    wait(200);
    indexer(0);
}

void scoreOneBallWithWait()
{
    while (indexerLimit.get_value() != 1)
    {
        indexer(127);
        intake(127);
    }
    wait(500);
    intake(0);
    wait(200);
    indexer(0);
    while (true)
    {
        pros::vision_object_s_t obj = vision2.get_by_size(0);

        if (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID) && obj.width > 100)
        {
            break;
        }
    }
}

void loadBall()
{
    while (frontRollerLine.get_value() > BALL_DETECTED_SIGNATURE)
    {
        frontRollers(127);
        intake(127);
    }
    frontRollers(0);
    intake(0);
}

void visionTest()
{
    // while (1)
    // {

    //     pros::vision_object_s_t obj = vision2.get_by_size(0);
    //     driveOP();
    //     if (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID) && obj.width > 100)
    //     {
    //         frontRollers(127);
    //         intake(127);
    //     }
    //     else
    //     {
    //         break;
    //     }
    // }
    // intake(0);
    // frontRollers(0);
    scoreOneBallWithVision();
    timedDrive(500, -127);
}

//Driver Skills Intake Macros

void intakeReverseFor(int timeout)
{
    int time = 0;
    while (intakeLimit.get_value() == 0)
    {
        driveOP();
        indexer(75);
        intake(75);
        frontRollers(127);
    }
    while (time < 350)
    {
        frontRollers(0);
        indexer(-127);
        intake(-127);
        wait(1);
        time++;
        driveOP();
    }
    time = 0;
    while (time < timeout)
    {
        driveOP();
        wait(1);
        time++;
        driveOP();
        indexer(-127);
        intake(127);
        frontRollers(127);
    }
    intakeFullStop();
}

void ejectBalls(void *parameter)
{
    int time = 0;
    while (intakeLimit.get_value() == 0)
    {
        indexer(75);
        intake(75);
        frontRollers(127);
    }
    while (time < 350)
    {
        frontRollers(0);
        indexer(-127);
        intake(-127);
        wait(1);
        time++;
    }
    time = 0;
    while (time < 1000)
    {
        wait(1);
        time++;
        indexer(-127);
        intake(127);
        frontRollers(127);
    }
    intakeFullStop();
    frontRollers(127);
}

void ejectOneBall(void *parameter)
{
    int time = 0;
    while (intakeLimit.get_value() == 0)
    {
        indexer(75);
        intake(75);
        frontRollers(127);
    }
    while (time < 350)
    {
        frontRollers(0);
        indexer(-127);
        intake(-127);
        wait(1);
        time++;
    }
    time = 0;
    while (time < 250)
    {
        wait(1);
        time++;
        indexer(-127);
        intake(127);
        frontRollers(127);
    }
    intakeFullStop();
    frontRollers(127);
}

void twoBlueCycleThreeRed()
{
    scoreOneBall();

    for (int i = 0; i < 2; i++)
    {
        while (intakeLimit.get_value() == 0)
        {
            driveOP();
            frontRollers(127);
            intake(127);
        }
        wait(100);
        scoreOneBall();
    }
    frontRollers(-127);
    intake(127);
    //timedDrive(200, -127);
    //intakeReverseFor(2000);
}

void twoBlueCycleTwoRed()
{
    scoreOneBall();

    while (frontRollerLine.get_value() > BALL_DETECTED_SIGNATURE)
    {
        driveOP();
        frontRollers(127);
        intake(127);
    }
    wait(250);
    left(40);
    right(40);
    scoreOneBall();
    wait(250);
    while (true)
    {
        driveOP();
        if (frontRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            frontRollers(127);
            //wait(250);
        }
        else
        {
            intake(127);
            frontRollers(127);
            if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
            {
                break;
            }
        }
    }
    left(40);
    right(40);
    wait(250);
    //scoreOneBall();
    // frontRollers(127);
    // wait(200);
    frontRollers(0);
    if (frontRollerLine.get_value() > BALL_DETECTED_SIGNATURE)
    {
        while (frontRollerLine.get_value() > BALL_DETECTED_SIGNATURE)
        {
            driveOP();
            //frontRollers(127);
            intake(127);
        }
    }
    intake(-127);
    frontRollers(-127);
    wait(250);
    timedDrive(200, -127);
}

void twoBlueCycleTwoRedAuton()
{
    for (int i = 0; i < 2; i++)
    {
        scoreOneBall();
        while (intakeLimit.get_value() == 0)
        {
            driveOP();
            frontRollers(127);
            intake(127);
        }
        frontRollers(0);
        intake(0);
        wait(100);
    }
    timedDrive(200, 40);
    frontRollers(-127);
    intake(127);
    timedDrive(200, -127);
    //intakeReverseFor(2000);
}

void oneBlueCycleTwoRed()
{

    //scoreTwoBalls();
    while (intakeLimit.get_value() == 0)
    {
        driveOP();
        frontRollers(127);
        intake(127);
    }

    frontRollers(-127);
    intake(127);
    wait(500);
}

void oneBlueCycleOneRed()
{

    scoreOneBall();
    while (intakeLimit.get_value() == 0)
    {
        driveOP();
        frontRollers(127);
        intake(127);
    }
    frontRollers(-127);
    intake(127);
    timedDrive(200, -127);
}

void intakeOP()
{

    if (master.get_digital(DIGITAL_R1) || partner.get_digital(DIGITAL_L2))
    {
        indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        pros::vision_object_s_t obj = vision.get_by_size(0);

        if ((intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE) && (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID) && obj.width > 100) && frontRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            frontRollers(0);
            intake(0);
            indexer(0);
        }
        else if (intakeLimit.get_value() == 1 && (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID)) && !topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            indexer(30);
            intake(75);
            frontRollers(127);
        }
        else if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            indexer(0);
            frontRollers(127);
            intake(0);
        }
        else
        {
            frontRollers(127);
            intake(127);
            indexer(50);
        }
    }
    else if (master.get_digital(DIGITAL_R2) || partner.get_digital(DIGITAL_R2))
    {
        frontRollers(-127);
    }
    else if (master.get_digital(DIGITAL_L2))
    {
        intakeReverseFor(750);
    }
    else if (master.get_digital(DIGITAL_UP))
    {
        //visionTest();
        // pros::Task i(intakeManager);
        // setIntakeMode(1);
        // wait(1000);
        // setIntakeMode(0);
        // wait(1000);
        // setIntakeMode(1);
        // wait(1000);
        // i.remove();
    }
    else if (master.get_digital(DIGITAL_LEFT))
    {
        twoBlueCycleTwoRed();
    }
    else if (master.get_digital(DIGITAL_L1) || partner.get_digital(DIGITAL_L1))
    {
        indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        frontRollers(0);
        if (indexerLimit.get_value() == 1)
        {
            while (indexerLimit.get_value() == 1)
            {
                driveOP();
                indexer(127);
                intake(0);
            }
            wait(100);
        }
        else
        {
            indexer(127);
            intake(127);
        }
    }
    else
    {
        frontRollers(0);
        intake(0);
        indexer(0);
    }
}
//Counting rollers from the top
/*
void findColorWhenRed()
{
    frontRollers(127);
    when button pressed{
        frontRollers(0);
        if redBall{
            moveRoller4(-127);
            moveRoller3(127);
            moveRoller2(-127);
            moveRoller1(127);
        }
        else if blueBall{
            moveRoller4(-127);
            moveRoller3(127);
            moveRoller2(127);
        }
    }
}
void findColorWhenBlue()
{
    frontRollers(127);
    when button pressed{
        frontRollers(0);
        if blueBall{
            moveRoller4(-127);
            moveRoller3(127);
            moveRoller2(-127);
            moveRoller1(127);
        }
        else if redBall{
            moveRoller4(-127);
            moveRoller3(127);
            moveRoller2(127);
        }
    }
}
void oneBlueOneRedWithRedCycle()//or oneRedWithOneBlueCycle
{
    moveFoward
    findColor();//Do this 3 times with waits in between
}
void oneRedOneBlueWithBlueCycle()// or oneBlueWithOneRedCycle
{
    moveForward
    findColor();//Do this twice with waits in between
}
*/