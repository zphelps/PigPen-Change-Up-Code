#include "main.h"

//Motors - 10
pros::Motor rightRollerMotor(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); //Orange - 17
pros::Motor leftRollerMotor(18, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor mainRollerMotor(19, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor indexerMotor(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

//Sensors
pros::ADIDigitalIn indexerLimit('G');
pros::ADIDigitalIn intakeLimit('H');
pros::Vision vision_sensor(16);

int stopLoading = 0;

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
        if (intakeLimit.get_value() == 1)
        {
            indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            intake(0);
            indexer(-20);
            wait(100);
            indexer(0);
        }
        else
        {
            indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }
    }
}

void loadOneBallAndReverse(void *parameter)
{
    int count = 0;
    while (count < 1)
    {
        frontRollers(127);
        intake(85);
        if (intakeLimit.get_value() == 1)
        {
            count++;
            wait(100);
        }
    }
    frontRollers(-127);
    intake(0);
}

void loadOneBall(void *parameter)
{
    int count = 0;
    while (count < 1)
    {
        frontRollers(127);
        intake(127);
        if (intakeLimit.get_value() == 1)
        {
            count++;
            wait(300);
        }
    }
    frontRollers(0);
    intake(0);
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
}

void scoreOneBallWithFrontRollers()
{
    int intakeCount = 0;
    int indexerCount = 0;
    frontRollers(127);
    while (intakeCount < 2)
    {
        left(50);
        right(50);
        indexer(127);
        intake(127);
        if (intakeLimit.get_value() == 1)
        {
            intakeCount++;
            wait(75);
        }
        if (indexerLimit.get_value() == 1)
        {
            indexerCount++;
            wait(75);
        }
    }
    master.print(0, 0, "%d", indexerCount);
    frontRollers(-127);
    if (indexerCount <= 2)
    {
        scoreOneBall();
    }
}

void scoreBalls(int time)
{
    while (indexerLimit.get_value() == 0)
    {
        indexer(127);
        intake(127);
    }
    wait(200);
    intake(-10);
    indexer(-127);
    wait(200);
    intake(127);
    indexer(127);
    wait(time);
    intake(0);
    indexer(0);
}

void countThreeBlue()
{
    while (intakeLimit.get_value() == 0)
    {
        frontRollers(127);
        intake(127);
    }
    scoreOneBall();
}

// void visionTest()
// {
//     while (true)
//     {
//         vision_object_s_t returnRed = vision_sensor.get_by_sig(0, RED_BALL);
//         vision_object_s_t returnBlue = vision_sensor.get_by_sig(0, BLUE_BALL);
//     }
// }
void oneRedOneBlue()
{
    int count = 0;
    scoreOneBall();

    while (intakeLimit.get_value() == 0)
    {
        left(44);
        right(44);
        frontRollers(127);
        intake(75);
        // if (intakeLimit.get_value() == 1)
        // {
        //    // count++;
        //     wait(300);
        // }
    }
    intakeFullStop();
    frontRollers(-127);
    intake(127);
    //scoreOneBallWithWait();
    wait(500);

    while (intakeLimit.get_value() == 0)
    {
        left(35);
        right(35);
        frontRollers(127);
        intake(75);
        // if (intakeLimit.get_value() == 1)
        // {
        //    // count++;
        //     wait(300);
        // }
    }
    intakeFullStop();
    scoreOneBall();
    wait(500);
    intakeFullReverse();
    wait(500);
    //intake(127);
    //indexer(127);
    //frontRollers(0);

    //for (int i = 0; i < 2; i++)
    // if (count > 2)
    // {
    //     intake(0);
    //     frontRollers(0);
    // }
}

void cycleBlueRed()
{
    for (int i = 0; i < 2; i++)
    {
        while (intakeLimit.get_value() == 0)
        {
            left(45);
            right(45);
            frontRollers(127);
            intake(127);
        }
        wait(300);
        intake(0);
        frontRollers(0);
    }
    scoreOneBall();
}

void topBallSwitch(int timeout)
{
    int time = 0;
    while (indexerLimit.get_value() == 0)
    {
        intake(127);
        indexer(127);
    }
    while (intakeLimit.get_value() == 0 && time < timeout)
    {
        left(40);
        right(40);
        intake(127);
        frontRollers(127);
        time++;
        wait(1);
    }
    intakeFullStop();
}

void topBallSwitch()
{
    while (indexerLimit.get_value() == 0)
    {
        intake(127);
        indexer(127);
    }
    while (intakeLimit.get_value() == 0)
    {
        intake(127);
        frontRollers(127);
    }
    intakeFullStop();
}

void intakeTests()
{
    int count = 0;

    while (count < 2)
    {
        intake(127);
        indexer(127);
        frontRollers(127);
        if (indexerLimit.get_value() == 1)
        {
            count++;
            wait(300);
        }
    }
    intakeFullStop();
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
    frontRollers(127);
    scoreOneBall();
    wait(250);
    scoreOneBall();
    //intakeReverseFor(2000, 1000);
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
        if (intakeLimit.get_value() == 1)
        {
            indexer(0);
            frontRollers(127);
            indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }
        else
        {
            frontRollers(127);
            intake(75);
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
    else if (master.get_digital(DIGITAL_L1) || partner.get_digital(DIGITAL_L1))
    {
        indexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        frontRollers(0);

        if (indexerLimit.get_value() == 1)
        {
            intake(0);
            wait(200); //250
            indexer(-50);
            wait(150); //150
        }
        else if (intakeLimit.get_value() == 0)
        {
            indexerVelocity(600);
            intake(127);
        }
        else
        {
            indexerVelocity(600);
            intake(0);
        }
    }
    else if (master.get_digital(DIGITAL_LEFT) || partner.get_digital(DIGITAL_LEFT))
    {
        indexer(-127);
        intake(127);
    }
    else if (master.get_digital(DIGITAL_DOWN) || partner.get_digital(DIGITAL_DOWN))
    {
        indexer(-127);
        intake(-127);
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