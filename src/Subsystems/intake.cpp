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
pros::Vision vision2(7);

int BALL_DETECTED_SIGNATURE = 2500;

//Vision Sensor
pros::Vision vision(10); //Construct vision object

#define BLUE_ID 1
#define RED_ID 2

pros::vision_signature_s_t BLUE_BALL;
pros::vision_signature_s_t RED_BALL;

void initVision()
{
    //Set blue and red ball vision siginatures
    BLUE_BALL = pros::Vision::signature_from_utility(BLUE_ID, -3461, -2325, -2893, 8235, 11659, 9947, 3.200, 0);
    RED_BALL = pros::Vision::signature_from_utility(RED_ID, 6495, 8301, 7398, -311, 403, 46, 5.000, 0);
    //Set vision signature to BLUE_ID and RED_ID
    vision.set_signature(BLUE_ID, &BLUE_BALL);
    vision.set_signature(RED_ID, &RED_BALL);

    vision2.set_signature(BLUE_ID, &BLUE_BALL);
    vision2.set_signature(RED_ID, &RED_BALL);
}

void descoreBottomBall(int timeout)
{
    int time = 0;
    while (time < timeout)
    {
        pros::vision_object_s_t obj = vision2.get_by_size(0);

        if (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID) && obj.width > 100)
        {
            frontRollers(127);
            // if (time < 200 || time > 400 && time < 600 || time > 800)
            // {
            //     drive(-30);
            // }
            // else
            // {
            //     drive(30);
            // }
        }
        else
        {
            wait(100);
            frontRollers(0);
            break;
        }
        wait(1);
        time++;
    }
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
}

void scoreOneBallWithVision(int timeout)
{
    int time = 0;
    while (indexerLimit.get_value() == 0 && time < timeout)
    {
        indexer(127);
        intake(127);
        wait(1);
        time++;
    }
    intake(0);
    wait(200);
    indexer(-30);
    while (time < timeout)
    {
        pros::vision_object_s_t obj = vision2.get_by_size(0);

        if (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID) && obj.width > 100)
        {
            break;
        }

        wait(1);
        time++;
    }
    indexer(0);
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
    while (1)
    {
        //Take snapshot and find return the biggest object of sig -> BLUE_ID
        pros::vision_object_s_t obj = vision2.get_by_sig(0, BLUE_ID);
        //If the object returned is bigger thatn 100 pixels -> stop
        if (obj.signature != VISION_OBJECT_ERR_SIG && obj.width > 100)
        {
            frontRollers(127);
            intake(127);
        }
        //Run the front rollers and intake until the ball is no longer detected
        else
        {
            intakeFullStop();
            break;
        }
    }
}

void autoSort()
{
    //Sort for red balls
    while (1)
    {
        //Take snapshot and find return the biggest object
        pros::vision_object_s_t obj = vision.get_by_size(0);
        if (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID) && obj.width > 100)
        {
            indexer(-127);
            wait(300); //wait for the ball to exit the robot
            break;
        }
        else
        {
            frontRollers(127);
            intake(127);
            indexer(127);
        }
    }
}

//Driver Skills Intake Macros

void intakeReverseFor(int timeout)
{
    int time = 0;
    while (true)
    {
        if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            break;
        }
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
            indexer(100);
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
        visionTest();
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
