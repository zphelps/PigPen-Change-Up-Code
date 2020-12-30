#include "main.h"

//Motors
pros::Motor rightRollerMotor(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
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
pros::Vision vision3(4);

pros::vision_signature_s_t BLUE_BALL;
pros::vision_signature_s_t RED_BALL;

int BALL_DETECTED_SIGNATURE = 2500;

Intake intake;

//Vision Sensor Methods
void initVision()
{
    BLUE_BALL = pros::Vision::signature_from_utility(BLUE_ID, -3461, -2325, -2893, 8235, 11659, 9947, 3.200, 0);

    RED_BALL = pros::Vision::signature_from_utility(RED_ID, 6495, 8301, 7398, -311, 403, 46, 5.000, 0);

    vision.set_signature(BLUE_ID, &BLUE_BALL);
    vision.set_signature(RED_ID, &RED_BALL);

    vision2.set_signature(BLUE_ID, &BLUE_BALL);
    vision2.set_signature(RED_ID, &RED_BALL);
}

bool topBallDetected(int color)
{
    if (color == BLUE_ID)
    {
        pros::vision_object_s_t obj = vision2.get_by_sig(0, BLUE_ID);

        if (obj.signature != VISION_OBJECT_ERR_SIG && obj.width > 100)
        {
            return true;
        }
    }
    if (color == RED_ID)
    {
        pros::vision_object_s_t obj = vision2.get_by_sig(0, RED_ID);

        if (obj.signature != VISION_OBJECT_ERR_SIG && obj.width > 100)
        {
            return true;
        }
    }
    if (color == ANY_COLOR)
    {
        pros::vision_object_s_t obj = vision2.get_by_size(0);

        if (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID) && obj.width > 100)
        {
            return true;
        }
    }
    return false;
}

bool BallD(int color)
{
    if (color == BLUE_ID)
    {
        pros::vision_object_s_t obj = vision2.get_by_sig(0, BLUE_ID);

        if (obj.signature != VISION_OBJECT_ERR_SIG && obj.width > 100)
        {
            return true;
        }
    }
    if (color == RED_ID)
    {
        pros::vision_object_s_t obj = vision2.get_by_sig(0, RED_ID);

        if (obj.signature != VISION_OBJECT_ERR_SIG && obj.width > 100)
        {
            return true;
        }
    }
    if (color == ANY_COLOR)
    {
        pros::vision_object_s_t obj = vision2.get_by_size(0);

        if (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID) && obj.width > 100)
        {
            return true;
        }
    }
    return false;
}

bool intakeBallDetected(int color)
{
    if (color == BLUE_ID)
    {
        pros::vision_object_s_t obj = vision.get_by_sig(0, BLUE_ID);

        if (obj.signature != VISION_OBJECT_ERR_SIG && obj.width > 100)
        {
            return true;
        }
    }
    if (color == RED_ID)
    {
        pros::vision_object_s_t obj = vision.get_by_sig(0, RED_ID);

        if (obj.signature != VISION_OBJECT_ERR_SIG && obj.width > 100)
        {
            return true;
        }
    }
    if (color == ANY_COLOR)
    {
        pros::vision_object_s_t obj = vision.get_by_size(0);

        if (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID) && obj.width > 100)
        {
            return true;
        }
    }
    return false;
}

bool ballRightOfCenter(int x, int W)
{
    pros::vision_object_s_t obj = vision2.get_by_sig(0, BLUE_ID);
    if (obj.left_coord > (312 - x - W) && obj.width > 50)
    {
        return true;
    }
    return false;
}

bool ballLeftOfCenter(int x, int W)
{
    pros::vision_object_s_t obj = vision2.get_by_sig(0, BLUE_ID);
    if (obj.left_coord < (308 - x - W) && obj.width > 50)
    {
        return true;
    }
    return false;
}
//Intake Methods
void Intake::frontRollers(int speed, int time)
{
    if (time > 0)
    {
        rightRollerMotor.move(speed);
        leftRollerMotor.move(speed);
        wait(time);
        rightRollerMotor.move(0);
        leftRollerMotor.move(0);
    }
    else
    {
        rightRollerMotor.move(speed);
        leftRollerMotor.move(speed);
    }
}

void Intake::intakeRollers(int speed, int time)
{
    if (time > 0)
    {
        mainRollerMotor.move(speed);
        wait(time);
        mainRollerMotor.move(0);
    }
    else
    {
        mainRollerMotor.move(speed);
    }
}

void Intake::indexer(int speed, int time)
{
    if (time > 0)
    {
        indexerMotor.move(speed);
        wait(time);
        indexerMotor.move(0);
    }
    else
    {
        indexerMotor.move(speed);
    }
}

void Intake::intake(int FRSpeed, int IRSpeed, int ISpeed)
{
    frontRollers(FRSpeed);
    intakeRollers(IRSpeed);
    indexer(ISpeed);
}

void Intake::stop()
{
    intake(0, 0, 0);
}

/**
 * @brief brake specific subsystems
 * 
 * @param subsystem 
 */
void Intake::brake(int subsystem)
{
    switch (subsystem)
    {
    case FRONT_ROLLERS:
    {
        rightRollerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftRollerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
    case INTAKE:
    {
        mainRollerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
    case INDEXER:
    {
        indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
    }
}

/**
 * @brief coast specific subsystems
 * 
 * @param subsystem 
 */
void Intake::coast(int subsystem)
{
    switch (subsystem)
    {
    case FRONT_ROLLERS:
    {
        rightRollerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        leftRollerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    case INTAKE:
    {
        mainRollerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    case INDEXER:
    {
        indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    }
}

/**
 * @brief intake code to prevent balls from getting pushed out the ball ejector rollers
 * 
 */
void Intake::logic()
{
    pros::vision_object_s_t obj = vision.get_by_size(0);

    if ((intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE) && (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID) && obj.width > 100) && frontRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
    {
        intake(0, 0, 0);
    }
    else if (intakeLimit.get_value() == 1 && (obj.signature != VISION_OBJECT_ERR_SIG && (obj.signature == BLUE_ID || obj.signature == RED_ID)) && !topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
    {
        intake(127, 75, 30);
    }
    else if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
    {
        intake(127, 0, 0);
    }
    else
    {
        intake(127, 127, 75);
    }
}

void Intake::manager(void *parameter)
{
    while (1)
    {
        logic();
    }
}

/**
 * @brief eject ball with timeout
 * 
 * @param timeout 
 * @param auton 
 */
void Intake::reverseFor(int timeout, bool auton)
{
    if (auton)
    {
        int time = 0;
        while (time < 1000)
        {
            if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
            {
                break;
            }
            intake(127, 75, 75);
            wait(1);
            time++;
        }
        time = 0;
        while (time < 350)
        {
            intake(0, -127, -127);
            wait(1);
            time++;
        }
        time = 0;
        while (time < timeout)
        {
            drive.driveOP(NORMAL_DRIVE);
            wait(1);
            time++;
            intake(127, 127, -127);
        }
        intake(0, 0, 0);
    }
    else
    {
        int time = 0;
        while (true)
        {
            if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
            {
                break;
            }
            drive.driveOP(NORMAL_DRIVE);
            intake(127, 75, 75);
        }
        while (time < 350)
        {
            intake(0, -127, -127);
            wait(1);
            time++;
            drive.driveOP(NORMAL_DRIVE);
        }
        time = 0;
        while (time < timeout)
        {
            drive.driveOP(NORMAL_DRIVE);
            wait(1);
            time++;
            drive.driveOP(NORMAL_DRIVE);
            intake(127, 127, -127);
        }
        intake(0, 0, 0);
    }
}

void Intake::score(int numBalls, int timeout)
{
    coast(INDEXER);
    for (int i = 0; i <= numBalls; i++)
    {
        int time = 0;
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE || intakeLimit.get_value() == 1)
        {
            while (indexerLimit.get_value() != 1 && time < timeout)
            {
                indexer(127);
                wait(1);
                time++;
            }
        }
        else
        {
            while (indexerLimit.get_value() != 1 && time < timeout)
            {
                indexer(127);
                intakeRollers(127);
                wait(1);
                time++;
            }
        }
        intakeRollers(0);
        wait(200); //200
        indexer(0);
        wait(200);
    }
}

/**
 * @brief score ball by running indexer until ball is detected in goal by vision
 * 
 * @param timeout 
 */
void Intake::scoreWithVision(int timeout)
{
    score(ONE_BALL);
    int time = 0;
    while (!topBallDetected(ANY_COLOR) && time < timeout)
    {
        wait(1);
        time++;
    }
}

void Intake::intakeOP()
{
    if (master.get_digital(DIGITAL_R1) || partner.get_digital(DIGITAL_L2))
    {
        indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        logic();
    }
    else if (master.get_digital(DIGITAL_R2) || partner.get_digital(DIGITAL_R2))
    {
        frontRollers(-127);
    }
    else if (master.get_digital(DIGITAL_L2))
    {
        reverseFor(750);
    }
    else if (master.get_digital(DIGITAL_UP))
    {
        //Test Code:
        //twoBlueCycleOneRed();
        drive.move(-36, 0, 0, true, false);
        reverseFor(2000);
    }
    else if (master.get_digital(DIGITAL_L1) || partner.get_digital(DIGITAL_L1))
    {
        coast(INDEXER);
        frontRollers(0);
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            indexer(127);
        }
        else if (indexerLimit.get_value() == 1)
        {
            intakeRollers(0);
            wait(200);
            indexer(0);
            wait(50);
        }
        else
        {
            intakeRollers(127);
            indexer(127);
        }
    }
    else
    {
        intake(0, 0, 0);
    }
}

//Intake Macros (IN PROGRESS):

void Intake::twoBlueCycleTwoRed()
{
    scoreWithVision();
    while (topBallDetected(ANY_COLOR))
    {
        drive.driveOP();
        intake(127, 127, 0);
    }
    while (!intakeBallDetected(BLUE_ID))
    {
        drive.driveOP();
        intake(127, 127, 0);
    }
    scoreWithVision();
    while (topBallDetected(ANY_COLOR))
    {
        drive.driveOP();
        frontRollers(127);
    }
    intake(-127, 127, 0);
    wait(300);
    intake(0, 0, 0);
}
