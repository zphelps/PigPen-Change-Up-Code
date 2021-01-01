#include "main.h"

std::shared_ptr<ChassisController> okapiChassis = ChassisControllerBuilder()
                                                      .withMotors({9, 2}, {5, 1})
                                                      .withGains(
                                                          {0.01, 0, 0},
                                                          {0.1, 0, 0})
                                                      .withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 12.5_in}, 500})
                                                      .build();

std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
        .withLimits({
            2.0, //Maximum linear velocity of the chassis
            3.0, //Maximum linear acceleration
            8.0  //Maximum linear jerk of the chassis
        })
        .withOutput(okapiChassis)
        .buildMotionProfileController();

void test_sTurn()
{
    profileController->generatePath({{0_ft, 0_ft, 0_deg},
                                     {5_ft, 5_ft, 90_deg}},
                                    "A");
    profileController->setTarget("A");
    profileController->waitUntilSettled();
}

void testMinSpeed(int speed)
{
    drive.drivePower(speed, speed);
    wait(10000);
}

void two_goal_and_middle_left()
{
    long startTime = pros::millis();
    int time = 0;
    setCoordinates(0, 128, -90);
    intake.indexer(60, 150);
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.5, 0, 0, 35).sweepLeft(-135, 15, 15);
    intake.stop();
    drive.drivePower(10, 40);
    intake.indexer(-127, 250);
    intake.score(ONE_BALL);

    drive.move(-6, -135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepRightBack(-90, 0, 15);
    intake.intake(-127, 0, 0);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(86, -90, 0); //0.8, 88
    intake.stop();

    drive.withTurnGains(1.4, 0, 0, 20).turn(-180);   //1.25
    drive.withGains(0.3, 0, 0, 40).move(6, -180, 0); //8
    drive.drivePower(45, 45);
    intake.score(ONE_BALL);
    while (!intakeBallDetected(BLUE_ID) && time < 2500)
    {
        intake.intake(127, 127, 0);
        wait(1);
        time++;
    }
    intake.stop();
    //intake.score(ONE_BALL);
    intake.frontRollers(-127);

    drive.move(-28, -180, 0);
    drive.withTurnGains(1, 0, 0, 20).turn(0);
    intake.intake(127, 0, 0);
    drive.move(4, 0, 0);
    drive.withTurnGains(2, 0, 0, 35).sweepLeft(-25, 5, 2);
    intake.stop();
    drive.timedDrive(500, 100, 10);
    intake.score(ONE_BALL, 1000);
    while (pros::millis() - startTime < 14500)
    {
        wait(1);
    }
    drive.timedDrive(500, -127, -127);
}

void two_goal_right()
{
    int time = 0;
    setCoordinates(0, 24, 0);
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.5, 0, 0, 25).sweepRight(135, 0);
    drive.withGains(0.2, 0, 0, 35).move(8, 135, 0);
    drive.timedDrive(300, 30, 30); //300
    intake.stop();

    intake.score(ONE_BALL);
    intake.intake(0, 127, 0);

    drive.move(-6, 135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepLeftBack(90, 0, 15);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(56, 90, 0); //0.8
    intake.stop();

    drive.withTurnGains(1.3, 0, 0, 20).turn(180);
    drive.withGains(0.3, 0, 0, 40).move(10, 180, 0); //8
    drive.timedDrive(100, 30, 30);
    drive.drivePower(30, 30);
    intake.score(ONE_BALL);
    while (!intakeBallDetected(BLUE_ID))
    {
        intake.intake(127, 127, 0);
    }
    intake.stop();
    intake.score(ONE_BALL);
    intake.frontRollers(-127);

    drive.withGains(0.15, 0, 0, 30).move(-14, 180, 2);
}

void two_goal_left()
{
    int time = 0;
    setCoordinates(0, 125, 0);
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.5, 0, 0, 25).sweepLeft(-135, 0);
    drive.withGains(0.2, 0, 0, 35).move(8, -135, 0);
    drive.timedDrive(300, 30, 30); //300
    intake.stop();
    intake.score(ONE_BALL);
    intake.intake(-127, 127, 0);

    drive.move(-6, -135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepRightBack(-90, 0, 15);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(90, -90, 0); //0.8
    intake.stop();

    drive.turn(-180);
    drive.withGains(0.3, 0, 0, 40).move(10, -180, 0); //8
    drive.timedDrive(100, 30, 30);
    drive.drivePower(30, 30);
    intake.score(ONE_BALL);
    while (!intakeBallDetected(BLUE_ID))
    {
        intake.intake(127, 127, 0);
    }
    intake.stop();
    intake.score(ONE_BALL);

    drive.withGains(0.15, 0, 0, 30).move(-14, -180, 2);
}

void home_row_left_no_cycle()
{
    int time = 0;
    setCoordinates(0, 125, 0);
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.5, 0, 0, 25).sweepLeft(-135, 0);
    drive.withGains(0.2, 0, 0, 35).move(8, -135, 0);
    drive.timedDrive(300, 30, 30); //300
    intake.stop();
    intake.score(ONE_BALL);
    intake.intake(-127, 0, 0);

    drive.move(-6, -135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepRightBack(-90, 0, 15);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(90, -90, 0); //0.8
    intake.stop();

    drive.turn(-180);
    drive.withGains(0.3, 0, 0, 40).move(10, -180, 0); //8
    drive.timedDrive(100, 30, 30);
    intake.score(ONE_BALL);

    drive.withGains(0.15, 0, 0, 30).move(-14, -180, 2);
    drive.turn(-270);
    drive.move(21, -270, 1, false, true);                         //23
    drive.withTurnGains(1.45, 0, 0, 80).sweepRight(-225, 30, 20); //1.75
    intake.intake(127, 127, 0);
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).move(20, -225, 0);
    drive.timedDrive(1000, 40, 40);
    drive.drivePower(35, 35);
    intake.stop();
    intake.score(ONE_BALL);
    intake.intake(-127, -127, 0);
    drive.move(-24, -225, 1);
}

void home_row_left_cycle()
{
    int time = 0;
    setCoordinates(0, 125, 0);
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.5, 0, 0, 25).sweepLeft(-135, 0);
    drive.withGains(0.2, 0, 0, 35).move(8, -135, 0);
    drive.timedDrive(300, 30, 30); //300
    intake.stop();
    intake.score(ONE_BALL);
    intake.intake(-127, 127, 0);

    drive.move(-6, -135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepRightBack(-90, 0, 15);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(90, -90, 0); //0.8
    intake.stop();

    drive.turn(-180);
    drive.withGains(0.3, 0, 0, 40).move(10, -180, 0); //8
    drive.timedDrive(100, 30, 30);
    drive.drivePower(30, 30);
    intake.score(ONE_BALL);
    while (!intakeBallDetected(BLUE_ID))
    {
        intake.intake(127, 127, 0);
    }
    intake.stop();
    intake.score(ONE_BALL);

    drive.withGains(0.15, 0, 0, 30).move(-14, -180, 2);
    drive.turn(-270);
    intake.intake(127, 127, -127);
    drive.move(21, -270, 1, false, true);                         //23
    drive.withTurnGains(1.45, 0, 0, 80).sweepRight(-225, 30, 20); //1.75
    intake.intake(127, 127, 0);                                   //
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).move(22, -225, 0);
    drive.timedDrive(750, 40, 40);
    drive.drivePower(35, 35);
    intake.stop();
    intake.score(ONE_BALL);
    intake.frontRollers(-127);
    drive.move(-24, -225, 1);
}

void home_row_right_no_cycle()
{
    int time = 0;
    setCoordinates(0, 24, 0);
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.5, 0, 0, 25).sweepRight(135, 0);
    drive.withGains(0.2, 0, 0, 35).move(8, 135, 0);
    drive.timedDrive(350, 30, 30); //300
    intake.stop();
    drive.drivePower(40, 40);
    intake.score(ONE_BALL);
    intake.intake(-127, 0, 0);

    drive.move(-6, 135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepLeftBack(90, 0, 15);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(56, 90, 0); //0.8
    intake.stop();

    drive.withTurnGains(1.3, 0, 0, 20).turn(180);
    drive.withGains(0.3, 0, 0, 40).move(10, 180, 0); //8
    drive.timedDrive(100, 30, 30);
    intake.score(ONE_BALL);

    drive.withGains(0.15, 0, 0, 30).move(-14, 180, 2);
    drive.withTurnGains(1.3, 0, 0, 20).turn(270);
    drive.move(26, 270, 1, false, true);                       //25
    drive.withTurnGains(1.6, 0, 0, 80).sweepLeft(225, 30, 20); //1.75
    intake.intake(127, 127, 0);
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).move(10, 225, 0);
    drive.timedDrive(750, 40, 40); //1000
    intake.stop();
    drive.drivePower(30, 30);
    intake.score(ONE_BALL);
    intake.intake(-127, -127, 0);
    drive.move(-24, 225, 1);
}

void home_row_right_cycle()
{
    int time = 0;
    setCoordinates(0, 24, 0);
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.5, 0, 0, 25).sweepRight(135, 0);
    drive.withGains(0.2, 0, 0, 35).move(8, 135, 0);
    drive.timedDrive(300, 30, 30); //300
    intake.stop();

    intake.score(ONE_BALL);
    intake.intake(0, 127, 0);

    drive.move(-6, 135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepLeftBack(90, 0, 15);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(56, 90, 0); //0.8
    intake.stop();

    drive.withTurnGains(1.3, 0, 0, 20).turn(180);
    drive.withGains(0.3, 0, 0, 40).move(10, 180, 0); //8
    drive.timedDrive(100, 30, 30);
    drive.drivePower(30, 30);
    intake.score(ONE_BALL);
    while (!intakeBallDetected(BLUE_ID))
    {
        intake.intake(127, 127, 0);
    }
    intake.stop();
    intake.score(ONE_BALL);

    drive.withGains(0.15, 0, 0, 30).move(-14, 180, 2);
    drive.withTurnGains(1.25, 0, 0, 20).turn(270);
    intake.intake(127, 127, -127);
    drive.move(23, 270, 1, false, true);
    drive.withTurnGains(1.6, 0, 0, 80).sweepLeft(227, 30, 20); //1.75
    intake.intake(127, 127, 0);
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).move(8, 225, 0);
    drive.timedDrive(750, 40, 40);
    drive.drivePower(35, 35);
    intake.stop();
    intake.score(ONE_BALL);
    intake.frontRollers(-127);
    drive.move(-24, 225, 1);
}

void programming_skills()
{
    int time = 0;
    setCoordinates(1, 24, 0);
    intake.intake(127, 127, 0);
    drive.withGains(0.15, 0, 0, 25).move(12, 0, 2);
    intake.intake(127, 127, 0);
    wait(100);
    intake.intake(127, 0, 0);
    drive.withTurnGains(1.8, 0, 0, 30).sweepRight(90, 0, 2);       //1.75, 30, threshhold = 5
    drive.withGains(0.3, 0, 0, 30).move(15, 90, 1);                //0.3
    drive.timedDrive(300, 30, 30);                                 //350
    drive.withGains(0.15, 0, 0, 65).move(-16, 90, 1, false, true); //15

    drive.withTurnGains(1.75, 0, 0, 30).sweepRightBack(135, 5); //1.5
    intake.stop();
    drive.withGains(0.275, 0, 0, 30).move(22, 135, 1);
    intake.intake(0, -75, -127);
    wait(100);
    intake.stop();
    intake.frontRollers(127);
    drive.timedDrive(300, 30, 30);

    intake.score(ONE_BALL, 500);

    drive.withGains(10, 0, 0, 30).moveBackToYCoord(10, 135, 0, true); //9
    time = 0;
    while (time < 1000)
    {
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            intake.intake(127, 0, 0);
            break;
        }
        intake.intake(127, 127, 60);
        wait(1);
        time++;
    }
    drive.waitForComplete();
    intake.intake(-127, 75, 0);                  //50
    wait(300);                                   //200
    drive.withTurnGains(1.15, 0, 0, 20).turn(0); //1, 20
    intake.intake(127, 0, 0);
    drive.withGains(4, 0, 0, 30).moveToXCoord(65, 0, 0); //4.55

    wait(250);
    drive.untilLineDetected(-35, 5000); //30
    wait(150);
    intake.intake(127, 0, 0);
    wait(50);
    drive.withTurnGains(1.45, 0, 0, 22).turn(90); //1.35 , 20
    drive.withGains(0.2, 0, 0, 35).move(6, 90, 2);
    intake.stop();
    drive.timedDrive(250, 30, 30);
    intake.brake(INTAKE);
    //intake.stop();

    //get indexer unstuck
    intake.indexer(-127, 50);
    intake.score(TWO_BALLS);
    setCoordinates(getX(), 2, getTheta());
    intake.frontRollers(127);
    drive.timedDrive(250, 60, 60); //40, 40
    intake.intake(127, -5, 0);
    wait(250);

    drive.move(-18, 90, 0); //-18
    intake.coast(INTAKE);
    intake.intakeRollers(75);
    intake.frontRollers(-127, 750);
    intake.stop();

    drive.withTurnGains(1.3, 0, 0, 20).turn(0);
    drive.withGains(4, 0, 0, 25).moveToXCoord(108, 0, 0, true);
    time = 0;
    while (time < 1000)
    {
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            break;
        }
        intake.intake(127, 127, 75);
        wait(1);
        time++;
    }
    intake.intake(127, 0, 0);
    drive.waitForComplete();

    drive.withTurnGains(1.4, 0, 0, 20).turn(90); //no modifier
    intake.stop();
    drive.withGains(0.175, 0, 0, 35).withCorrection(0.3).move(18, 45, 1); //0.25
    drive.timedDrive(500, 45, 45);

    intake.scoreWithVision(1000);

    drive.withGains(8, 0, 0, 30).withCorrection(0.5).moveBackToXCoord(88, 0, 0, true); //0.4, 90
    time = 0;
    while (time < 2000)
    {
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            intake.intake(127, 0, 0);
            break;
        }
        intake.intake(127, 127, 75);
        wait(1);
        time++;
    }
    drive.waitForComplete();
    drive.withTurnGains(1.35, 0, 0, 20).turn(-90); //1.25

    intake.intake(127, 75, 0);
    drive.moveToYCoord(53, -90, 2); //54

    drive.withTurnGains(1.35, 0, 0, 20).turn(0); //1.25
    intake.stop();
    drive.move(28, 0, 2);
    intake.stop();
    drive.timedDrive(300, 35, 35);

    intake.score(TWO_BALLS, 1000); //500
    intake.intake(127, 0, 0);
    drive.timedDrive(250, 45, 45); //40, 40
    setCoordinates(132, getY(), getTheta());
    intake.intake(127, 127, -127);
    drive.withGains(0.25, 0, 0, 30).move(-6, 0, 1); //8

    drive.withTurnGains(1.35, 0, 0, 20).turn(-90); //1.25
    intake.intake(127, 127, -127);
    drive.withGains(0.25, 0, 0, 75).move(20, -90, 0, false, true); //18
    intake.intake(127, 127, 0);
    drive.withTurnGains(2, 0, 0, 75).sweepRight(-45, 0, 15);
    drive.withGains(0.2, 0, 0, 35).move(8, -45, 0);
    intake.stop();
    drive.timedDrive(400, 35, 35);

    drive.drivePower(30, 30);
    intake.score(ONE_BALL, 500);
    intake.frontRollers(127);
    wait(250);

    drive.withCorrection(0.5).withGains(7, 0, 0, 30).moveBackToYCoord(82, -90, 0); //80
    intake.intake(127, 127, -127);
    drive.withTurnGains(1.35, 0, 0, 25).turn(-180); //1.25
    intake.intake(127, 0, 0);
    drive.move(40, -180, 0); //23
    wait(100);
    wait(250);
    drive.untilLineDetected(-35, 5000); //30
    //wait(200);
    setCoordinates(60, getY(), -180);
    wait(100);
    intake.intake(127, 0, 0);
    //drive.withGains(0.25, 0, 0, 25).move(0.5, -180, 0);

    // drive.move(-28, -45, 2);
    // intake.intake(127, 127, -127);
    // drive.withTurnGains(1.75, 0, 0, 25).turn(-90);
    // intake.intake(127, 127, 0);
    // drive.move(18, -90, 0);
    // drive.timedDrive(350, 30, 30);
    // intake.stop();
    // drive.timedDrive(650, 75, 75);
    // setCoordinates(100, 144, -90);
    // drive.move(-36, -90, 1, true);
    // time = 0;
    // while (time < 1000)
    // {
    //     if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
    //     {
    //         break;
    //     }
    //     intake.intake(127, 127, 60);
    //     wait(1);
    //     time++;
    // }
    // intake.intake(127, 0, 0);
    // drive.waitForComplete();
    // drive.withTurnGains(1.25, 0, 0, 20).turn(-180);
    // drive.move(32, -180, 0);

    drive.move(-2, -180, 0);
    drive.withTurnGains(1.25, 0, 0, 20).turn(-90);
    drive.move(-6, -90, 0, false, true);
    drive.timedDrive(500, -40, -40);
    drive.timedDrive(500, 40, 40);
    drive.timedDrive(1000, -40, -40);

    //intake.frontRollers(127);
    drive.withGains(0.2, 0, 0, 35).move(38, -93, 1, true); //38, -92
    time = 0;
    while (time < 1000)
    {
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE || intakeLimit.get_value() == 1)
        {
            intake.intake(127, 0, 0);
            break;
        }
        else
        {
            intake.intake(127, 127, 75);
        }
        wait(1);
        time++;
    }
    drive.waitForComplete();
    intake.intake(127, 0, 0);
    drive.timedDrive(350, 30, 30);
    intake.stop();

    intake.score(ONE_BALL);
    intake.frontRollers(127);
    drive.timedDrive(250, 75, 75);
    intake.intake(127, 0, 0);

    drive.move(-5, -90, 0, false, true);
    drive.withCorrection(0.15).move(-33, -50, 0); //-32, 0.4
    intake.stop();
    drive.withTurnGains(1.3, 0, 0, 20).turn(50); //1.25, 20
    drive.move(6, 50, 0, false, true);           //8
    drive.timedDrive(400, 45, 45);               //500

    intake.score(ONE_BALL);

    //intake.intake(127, 127, -127);
    drive.withGains(0.2, 0, 0, 25).move(-6, 60, 0);
    drive.withTurnGains(1.35, 0, 0, 25).turn(125); //1.25
    intake.intake(127, 127, -127);

    drive.move(6, 125, 0, false, true); //6
    intake.intake(127, 127, 0);
    drive.sweepRight(180, -20, 10);             //-15
    drive.withCorrection(0.1).move(32, 181, 2); //32
    intake.stop();
    drive.timedDrive(400, 60, 60); //750, 30, 30

    intake.score(ONE_BALL);
    intake.intake(127, 127, -127);
    drive.timedDrive(100, 55, 5);
    //setTheta(180);

    drive.move(-8, 180, 0); //-10
    drive.withTurnGains(1.35, 0, 0, 20).turn(270);
    //intake.intake(127, 127, 0);
    drive.withGains(0.25, 0, 0, 75).move(25, 270, 0, false, true); //23
    intake.intake(127, 127, 0);
    drive.withTurnGains(2, 0, 0, 75).sweepLeft(225, 0, 15);
    drive.withGains(0.2, 0, 0, 35).move(6, 225, 0);
    intake.intake(127, 0, 0);
    drive.timedDrive(250, 35, 35);

    drive.drivePower(45, 45);
    intake.score(ONE_BALL, 500);
    drive.drivePower(40, 40);
    time = 0;
    while (time < 500)
    {
        if (intakeBallDetected(BLUE_ID))
        {
            break;
        }
        intake.intake(127, 127, 0);
        wait(10);
        time++;
    }

    drive.move(-24, 225, 0);
}

void programming_skills_126()
{
    int time = 0;
    setCoordinates(1, 24, 0);
    intake.intake(127, 127, 0);
    drive.withGains(0.15, 0, 0, 25).move(12, 0, 2);
    intake.intake(127, 127, 0);
    wait(100);
    intake.intake(127, 0, 0);
    drive.withTurnGains(1.8, 0, 0, 30).sweepRight(90, 0, 2);       //1.75, 30, threshhold = 5
    drive.withGains(0.3, 0, 0, 30).move(15, 90, 1);                //0.3
    drive.timedDrive(300, 30, 30);                                 //350
    drive.withGains(0.15, 0, 0, 65).move(-16, 90, 1, false, true); //15

    drive.withTurnGains(1.75, 0, 0, 30).sweepRightBack(135, 5); //1.5
    intake.stop();
    drive.withGains(0.275, 0, 0, 30).move(22, 135, 1);
    intake.intake(0, -75, -127);
    wait(100);
    intake.stop();
    intake.frontRollers(127);
    drive.timedDrive(300, 30, 30);

    intake.score(ONE_BALL, 500);

    drive.withGains(10, 0, 0, 30).moveBackToYCoord(10, 135, 0, true); //9
    time = 0;
    while (time < 1000)
    {
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            intake.intake(127, 0, 0);
            break;
        }
        intake.intake(127, 127, 60);
        wait(1);
        time++;
    }
    drive.waitForComplete();
    intake.intake(-127, 75, 0);                  //50
    wait(300);                                   //200
    drive.withTurnGains(1.15, 0, 0, 20).turn(0); //1, 20
    intake.intake(127, 0, 0);
    drive.withGains(4, 0, 0, 30).moveToXCoord(65, 0, 0); //4.55

    wait(250);
    drive.untilLineDetected(-35, 5000); //30
    wait(150);
    intake.intake(127, 0, 0);
    wait(50);
    drive.withTurnGains(1.45, 0, 0, 22).turn(90); //1.35 , 20
    drive.withGains(0.2, 0, 0, 35).move(6, 90, 2);
    intake.stop();
    drive.timedDrive(250, 30, 30);
    intake.brake(INTAKE);
    //intake.stop();

    //get indexer unstuck
    intake.indexer(-127, 50);
    intake.score(TWO_BALLS);
    setCoordinates(getX(), 2, getTheta());
    intake.frontRollers(127);
    drive.timedDrive(250, 60, 60); //40, 40
    intake.intake(127, -5, 0);
    wait(250);

    drive.move(-18, 90, 0); //-18
    intake.coast(INTAKE);
    intake.intakeRollers(75);
    intake.frontRollers(-127, 750);
    intake.stop();

    drive.withTurnGains(1.3, 0, 0, 20).turn(0);
    drive.withGains(4, 0, 0, 25).moveToXCoord(108, 0, 0, true);
    time = 0;
    while (time < 1000)
    {
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            break;
        }
        intake.intake(127, 127, 75);
        wait(1);
        time++;
    }
    intake.intake(127, 0, 0);
    drive.waitForComplete();

    drive.withTurnGains(1.4, 0, 0, 20).turn(90); //no modifier
    intake.stop();
    drive.withGains(0.175, 0, 0, 35).withCorrection(0.3).move(18, 45, 1); //0.25
    drive.timedDrive(500, 45, 45);

    intake.scoreWithVision(1000);

    drive.withGains(8, 0, 0, 30).withCorrection(0.5).moveBackToXCoord(88, 0, 0, true); //0.4, 90
    time = 0;
    while (time < 2000)
    {
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            intake.intake(127, 0, 0);
            break;
        }
        intake.intake(127, 127, 75);
        wait(1);
        time++;
    }
    drive.waitForComplete();
    drive.withTurnGains(1.35, 0, 0, 20).turn(-90); //1.25

    intake.intake(127, 75, 0);
    drive.moveToYCoord(53, -90, 2); //54

    drive.withTurnGains(1.35, 0, 0, 20).turn(0); //1.25
    intake.stop();
    drive.move(28, 0, 2);
    intake.stop();
    drive.timedDrive(300, 35, 35);

    intake.score(TWO_BALLS, 1000); //500
    intake.intake(127, 0, 0);
    drive.timedDrive(250, 45, 45); //40, 40
    setCoordinates(132, getY(), getTheta());
    intake.intake(127, 127, -127);
    drive.withGains(0.25, 0, 0, 30).move(-6, 0, 1); //8

    drive.withTurnGains(1.35, 0, 0, 20).turn(-90); //1.25
    intake.intake(127, 127, -127);
    drive.withGains(0.25, 0, 0, 75).move(20, -90, 0, false, true); //18
    intake.intake(127, 127, 0);
    drive.withTurnGains(2, 0, 0, 75).sweepRight(-45, 0, 15);
    drive.withGains(0.2, 0, 0, 35).move(8, -45, 0);
    intake.stop();
    drive.timedDrive(400, 35, 35);

    drive.drivePower(30, 30);
    intake.score(ONE_BALL, 500);
    intake.frontRollers(127);
    wait(250);

    drive.withCorrection(0.5).withGains(7, 0, 0, 30).moveBackToYCoord(82, -90, 0); //80
    intake.intake(127, 127, -127);
    drive.withTurnGains(1.35, 0, 0, 25).turn(-180); //1.25
    intake.intake(127, 0, 0);
    drive.move(40, -180, 0); //23
    wait(100);
    wait(250);
    drive.untilLineDetected(-35, 5000); //30
    //wait(200);
    setCoordinates(60, getY(), -180);
    wait(100);
    intake.intake(127, 0, 0);
    //drive.withGains(0.25, 0, 0, 25).move(0.5, -180, 0);

    // drive.move(-28, -45, 2);
    // intake.intake(127, 127, -127);
    // drive.withTurnGains(1.75, 0, 0, 25).turn(-90);
    // intake.intake(127, 127, 0);
    // drive.move(18, -90, 0);
    // drive.timedDrive(350, 30, 30);
    // intake.stop();
    // drive.timedDrive(650, 75, 75);
    // setCoordinates(100, 144, -90);
    // drive.move(-36, -90, 1, true);
    // time = 0;
    // while (time < 1000)
    // {
    //     if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
    //     {
    //         break;
    //     }
    //     intake.intake(127, 127, 60);
    //     wait(1);
    //     time++;
    // }
    // intake.intake(127, 0, 0);
    // drive.waitForComplete();
    // drive.withTurnGains(1.25, 0, 0, 20).turn(-180);
    // drive.move(32, -180, 0);

    drive.move(-2, -180, 0);
    drive.withTurnGains(1.25, 0, 0, 20).turn(-90);
    drive.move(-6, -90, 0, false, true);
    drive.timedDrive(500, -40, -40);
    drive.timedDrive(500, 40, 40);
    drive.timedDrive(1000, -40, -40);

    //intake.frontRollers(127);
    drive.withGains(0.2, 0, 0, 35).move(38, -93, 1, true); //38, -92
    time = 0;
    while (time < 1000)
    {
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE || intakeLimit.get_value() == 1)
        {
            intake.intake(127, 0, 0);
            break;
        }
        else
        {
            intake.intake(127, 127, 75);
        }
        wait(1);
        time++;
    }
    drive.waitForComplete();
    intake.intake(127, 0, 0);
    drive.timedDrive(350, 30, 30);
    intake.stop();

    intake.score(ONE_BALL);
    intake.frontRollers(127);
    drive.timedDrive(250, 75, 75);
    intake.intake(127, 0, 0);

    drive.move(-5, -90, 0, false, true);
    drive.withCorrection(0.15).move(-33, -50, 0); //-32, 0.4
    intake.stop();
    drive.withTurnGains(1.3, 0, 0, 20).turn(50); //1.25, 20
    drive.move(6, 50, 0, false, true);           //8
    drive.timedDrive(400, 45, 45);               //500

    intake.score(ONE_BALL);

    //intake.intake(127, 127, -127);
    drive.withGains(0.2, 0, 0, 25).move(-6, 60, 0);
    drive.withTurnGains(1.35, 0, 0, 25).turn(125); //1.25
    intake.intake(127, 127, -127);

    drive.move(6, 125, 0, false, true); //6
    intake.intake(127, 127, 0);
    drive.sweepRight(180, -20, 10);             //-15
    drive.withCorrection(0.1).move(32, 181, 2); //32
    intake.stop();
    drive.timedDrive(400, 60, 60); //750, 30, 30

    intake.score(ONE_BALL);
    intake.intake(127, 127, -127);
    drive.timedDrive(100, 55, 5);
    //setTheta(180);

    drive.move(-8, 180, 0); //-10
    drive.withTurnGains(1.35, 0, 0, 20).turn(270);
    //intake.intake(127, 127, 0);
    drive.withGains(0.25, 0, 0, 75).move(25, 270, 0, false, true); //23
    intake.intake(127, 127, 0);
    drive.withTurnGains(2, 0, 0, 75).sweepLeft(225, 0, 15);
    drive.withGains(0.2, 0, 0, 35).move(6, 225, 0);
    intake.intake(127, 0, 0);
    drive.timedDrive(250, 35, 35);

    drive.drivePower(45, 45);
    intake.score(ONE_BALL, 500);
    drive.drivePower(40, 40);
    time = 0;
    while (time < 500)
    {
        if (intakeBallDetected(BLUE_ID))
        {
            break;
        }
        intake.intake(127, 127, 0);
        wait(10);
        time++;
    }

    drive.move(-24, 225, 0);
}

void autonomous()
{
    //two_goal_and_middle_left();
    programming_skills_126();
    // intake.intake(127, 0, 0);
    //drive.move(48, 0, 0);
    //home_row_right_no_cycle();
    // home_row_right_cycle();
    // home_row_left_no_cycle();
    //home_row_left_cycle();
    //two_goal_right();
    // two_goal_left();

    // switch (autonIndex)
    // {
    // case 0:
    //     home_row_right_no_cycle();
    //     break;
    // case 1:
    //     home_row_right_cycle();
    //     break;
    // case 2:
    //     home_row_left_no_cycle();
    //     break;
    // case 3:
    //     home_row_left_cycle();
    //     break;
    // case 4:
    //     two_goal_right();
    //     break;
    // case 5:
    //     two_goal_left();
    //     break;
    // case 6:
    //     programming_skills();
    //     //two_goal_and_middle_left();
    //     break;
    //}
}
