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

void corner_side_center_right()
{
    long startTime = pros::millis();
    int time = 0;
    setCoordinates(0, 24, 90);
    intake.indexer(60, 150);
    intake.intake(127, 127, 0);
    drive.timedDrive(300, 40, 40);
    drive.timedDrive(600, 80, 20);
    // drive.withTurnGains(1.5, 0, 0, 35).sweepRight(130, 0, 15);
    intake.stop();
    drive.drivePower(25, 75);
    //intake.indexer(-127, 250);
    intake.scoreWithVision(RED_ID);
    drive.drivePower(20, 20);
    intake.intake(127, 0, 0);
    wait(500);
    //intake.stop();
    while (time < 2500)
    {
        if (!topBallDetected(RED_ID))
        {
            break;
        }
        if (time < 250)
        {
            drive.drivePower(40, 30);
        }
        else if (time < 750)
        {
            drive.drivePower(-20, -20);
        }
        else
        {
            drive.drivePower(20, 20);
        }
        intake.intake(127, 127, 0);
        wait(1);
        time++;
    }

    intake.intake(127, 0, 0);

    // drive.withGains(0.2, 0, 0, 25).move(-28, 135, 2); //10
    drive.withGains(8, 0, 0, 25).moveBackToYCoord(31, 135, 2); //29
    intake.intake(127, 0, 0);

    // drive.withTurnGains(5, 0, 0, 100).sweepRightBack(-90, 0, 15);
    // intake.intake(-127, 0, 0);
    // drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(86, -90, 0); //0.8, 88
    // intake.stop();

    //intake.intake(-127, 0, 0);
    drive.withTurnGains(1.4, 0, 0, 20).turn(45); //1.25
    intake.intake(127, 0, 0);
    drive.withGains(0.2, 0, 0, 30).move(32, 45, 0);                                  // 35-> with vision = 24
    drive.withGains(0.12, 0, 0, 30).moveWithVision(10, 45, 0, BLUE_ID, false, true); //8
    drive.timedDrive(300, 30, 30);
    intake.intake(127, 0, 0);
    time = 0;
    while (time < 3000)
    {
        if (intakeBallDetected(BLUE_ID))
        {
            break;
        }
        if (time < 250)
        {
            if (crossTapeLine.get_value() < LINE_DETECTED)
            {
                drive.brakeLeftSide();
                intake.frontRollersSeparate(127, 0);
                drive.drivePower(0, 60);
            }
            else
            {
                drive.coast();
                intake.frontRollersSeparate(127, 0);
                drive.drivePower(100, 90);
            }
        }
        else if (time < 750)
        {
            if (crossTapeLine.get_value() < LINE_DETECTED)
            {
                drive.brakeLeftSide();
                drive.drivePower(0, -25);
            }
            else
            {
                drive.coast();
                drive.drivePower(-25, -25);
            }
        }
        else if (time < 1250)
        {
            if (crossTapeLine.get_value() < LINE_DETECTED)
            {
                drive.brakeLeftSide();
                intake.intake(127, 50, 0);
                drive.drivePower(0, 60);
            }
            else
            {
                drive.coast();
                intake.intake(127, 50, 0);
                drive.drivePower(30, 30);
            }
        }
        else if (time < 2000)
        {
            if (crossTapeLine.get_value() < LINE_DETECTED)
            {
                drive.brakeLeftSide();
                drive.drivePower(0, 60);
            }
            else
            {
                drive.coast();
                drive.drivePower(127, 100);
            }
        }
        else if (time < 2500)
        {
            if (crossTapeLine.get_value() < LINE_DETECTED)
            {
                drive.brakeLeftSide();
                drive.drivePower(0, 60);
            }
            else
            {
                drive.coast();
                drive.drivePower(30, 30);
            }
        }
        else if (time < 3000)
        {
            if (crossTapeLine.get_value() < LINE_DETECTED)
            {
                drive.brakeLeftSide();
                drive.drivePower(0, 60);
            }
            else
            {
                drive.coast();
                drive.drivePower(90, 80);
            }
        }
        else
        {
            if (crossTapeLine.get_value() < LINE_DETECTED)
            {
                drive.brakeLeftSide();
                intake.intake(127, 0, 0);
                drive.drivePower(0, 15);
            }
            else
            {
                drive.coast();
                intake.intake(127, 0, 0);
                drive.drivePower(15, 15);
            }
        }
        // if (intakeBallDetected(BLUE_ID))
        // {
        //     intake.intake(127, 0, 0);
        // }
        // else
        // {
        //     intake.intake(127, 127, 0);
        // }
        wait(1);
        time++;
    }
    intake.intake(0, -127, -127);
    wait(100);
    intake.stop();
    intake.score(ONE_BALL);
    intake.stop();
    //intake.score(ONE_BALL);

    // intake.frontRollers(-127);

    // drive.withGains(0.2, 0, 0, 30).move(-36, 60, 0);
    // intake.intake(127, 127, -127);
    // drive.withTurnGains(1.35, 0, 0, 25).turn(0);
    // intake.intake(127, 127, 0);
    // drive.withGains(0.15, 0, 0, 30).moveWithVision(10, 40, 0, BLUE_ID, false, true); //8
    // drive.untilLineDetected(25);
    // drive.withGains(0.2, 0, 0, 30).move(-36, 60, 0);

    // drive.withGains(0.2, 0, 0, 30).move(-38, 40, 0);
    drive.withGains(8, 0, 0, 30).moveBackToYCoord(32, 40, 0);
    drive.withTurnGains(1.35, 0, 0, 25).turn(-35);
    intake.intake(127, 0, 0);
    drive.withGains(0.2, 0, 0, 35).move(30, -40, 0);
    drive.timedDrive(500, 30, 30);
    drive.drivePower(30, 30);
    intake.score(ONE_BALL);
    drive.drivePower(0, 0);
    intake.intake(0, 0, 0);
    // drive.withTurnGains(2, 0, 0, 35).sweepLeft(-25, 5, 2);
    // intake.stop();
    // drive.timedDrive(500, 100, 10);
    // intake.score(ONE_BALL, 1000);
    while (pros::millis() - startTime < 14500)
    {
        wait(1);
    }
    drive.timedDrive(500, -127, -127);
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
    drive.drivePower(15, 40);
    intake.indexer(-127, 250);
    wait(100);
    intake.stop();
    intake.score(ONE_BALL, 2000);
    wait(200);

    drive.move(-6, -135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepRightBack(-90, 0, 15);
    intake.intake(127, 0, 0);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(83, -90, 0); //0.8, 88
    intake.stop();

    drive.withTurnGains(1.4, 0, 0, 20).turn(-180);   //1.25
    drive.withGains(0.3, 0, 0, 40).move(6, -180, 0); //8
    drive.drivePower(45, 45);
    intake.score(ONE_BALL);
    while (!intakeBallDetected(RED_ID) && time < 2500) //BLUE_ID
    {
        intake.intake(127, 127, 0);
        wait(1);
        time++;
    }
    intake.stop();
    //intake.score(ONE_BALL);
    intake.frontRollers(-127);

    drive.withGains(6, 0, 0, 20).moveBackToXCoord(26, -180, 0);
    drive.withTurnGains(1.25, 0, 0, 25).turn(0);
    intake.intake(127, 0, 0);
    drive.withGains(0.2, 0, 0, 30).move(5, 0, 0);
    drive.withTurnGains(2, 0, 0, 40).sweepLeft(-25, 5, 8);
    intake.stop();
    drive.timedDrive(600, 127, 10);
    intake.score(ONE_BALL, 1000);
    while (pros::millis() - startTime < 14500)
    {
        wait(1);
    }
    drive.timedDrive(500, -127, -127);

    // drive.withGains(8, 0, 0, 30).moveBackToXCoord(34, -225, 0);
    // drive.withTurnGains(1.35, 0, 0, 30).turn(-315);
    // //intake.intake(127, 0, 0);
    // drive.withGains(0.2, 0, 0, 40).move(9, -315, 0, false, true);
    // //drive.withTurnGains(2, 0, 0, 35).sweepLeft(-25, 5, 2);
    // drive.timedDrive(1000, 40, 55);
    // intake.intake(-127, -50, -50);
    // wait(100);
    // intake.stop();
    // drive.drivePower(35, 35);
    // intake.score(ONE_BALL, 2000);
    // while (pros::millis() - startTime < 14500)
    // {
    //     wait(1);
    // }
    // drive.timedDrive(500, -127, -127);
}

void two_goal_right()
{
    long startTime = pros::millis();
    int time = 0;
    setCoordinates(0, 128, -90);
    intake.indexer(60, 150);
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.5, 0, 0, 35).sweepLeft(-135, 15, 15);
    intake.stop();
    drive.drivePower(15, 40);
    intake.indexer(-127, 250);
    wait(100);
    intake.stop();
    intake.score(ONE_BALL, 2000);
    wait(200);

    drive.move(-6, -135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepRightBack(-90, 0, 15);
    intake.intake(127, 0, 0);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(83, -90, 0); //0.8, 88
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
    intake.score(ONE_BALL);
    //intake.score(ONE_BALL);
    intake.frontRollers(-127);

    drive.withGains(6, 0, 0, 20).moveBackToXCoord(26, -180, 0);
    // drive.withTurnGains(1.25, 0, 0, 25).turn(0);
    // intake.intake(127, 0, 0);
    // drive.withGains(0.2, 0, 0, 30).move(6, 0, 0);
    // drive.withTurnGains(2, 0, 0, 35).sweepLeft(-25, 5, 2);
    // intake.stop();
    // drive.timedDrive(600, 127, 10);
    // intake.score(ONE_BALL, 1000);
    // while (pros::millis() - startTime < 14500)
    // {
    //     wait(1);
    // }
    // drive.timedDrive(500, -127, -127);
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
    drive.withTurnGains(1.5, 0, 0, 30).sweepLeft(-135, 0, 2);
    drive.withGains(0.25, 0, 0, 35).move(8, -135, 0);
    intake.intake(127, 127, -50);
    drive.timedDrive(300, 35, 35); //300
    intake.stop();
    intake.score(ONE_BALL);
    intake.intake(-127, 0, 0);

    drive.move(-6, -135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepRightBack(-90, 0, 15);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(87, -90, 0); //0.8
    intake.stop();

    drive.withTurnGains(1.35, 0, 0, 25).turn(-180);
    drive.withGains(0.3, 0, 0, 40).move(10, -180, 0); //8
    drive.timedDrive(300, 30, 30);
    intake.score(ONE_BALL);

    drive.withGains(0.15, 0, 0, 30).move(-14, -180, 2);
    drive.withTurnGains(1.35, 0, 0, 25).turn(-270);
    drive.move(21, -270, 0, false, true);                         //23
    drive.withTurnGains(1.45, 0, 0, 80).sweepRight(-225, 30, 20); //1.75
    intake.intake(127, 127, 0);
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).moveWithVision(20, -225, 0);
    drive.timedDrive(1000, 40, 40);
    drive.drivePower(35, 35);
    intake.stop();
    intake.score(ONE_BALL);
    intake.intake(-127, -127, 0);
    drive.move(-24, -225, 1);
}

void home_row_left_no_cycle_with_center()
{
    int time = 0;
    setCoordinates(0, 125, 0);
    intake.intake(127, 50, 0);
    drive.withTurnGains(1.5, 0, 0, 35).sweepLeft(-135, 0, 5);
    drive.withGains(0.25, 0, 0, 35).move(8, -135, 0);
    intake.intake(127, 75, -30);
    drive.timedDrive(300, 40, 40); //300
    intake.stop();
    intake.scoreWithVision();
    intake.intake(-127, 127, 0);

    drive.move(-6, -135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepRightBack(-90, 0, 18);
    drive.withCorrection(0.6).withGains(8, 0, 0, 30).moveBackToYCoord(88, -90, 0); //0.8
    intake.stop();

    drive.withTurnGains(1.25, 0, 0, 30).turn(-180);
    drive.withGains(0.3, 0, 0, 60).move(10, -180, 0, false, true); //8
    drive.timedDrive(300, 40, 40);
    intake.score(ONE_BALL, 750);

    drive.withGains(0.25, 0, 0, 30).move(-14, -180, 0);
    drive.withTurnGains(1.35, 0, 0, 25).turn(-270);
    drive.withGains(0.25, 0, 0, 75).move(15, -270, 0, false, true); //23
    drive.withTurnGains(1.45, 0, 0, 80).sweepRight(-225, 30, 20);   //1.75
    intake.intake(127, 127, 0);
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).moveWithVision(24, -225, 0);
    drive.timedDrive(500, 50, 50);
    drive.drivePower(0, 0);
    intake.stop();
    intake.scoreWithVision(750);
    intake.intake(127, 127, 0);
    drive.timedDrive(250, 50, 50);
    drive.withGains(0.25, 0, 0, 30).move(-34, -225, 0);
    drive.withTurnGains(1.25, 0, 0, 30).turn(-50);
    drive.withGains(0.35, 0, 0, 75).move(26, -46, 0, false, true);
    drive.timedDrive(300, 30, 30);
    intake.score(ONE_BALL);
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
    drive.withGains(0.2, 0, 0, 35).moveWithVision(8, 135, 0);
    drive.timedDrive(350, 30, 30); //300
    intake.stop();
    drive.drivePower(40, 40);
    intake.score(ONE_BALL);
    intake.intake(-127, 0, 0);

    drive.move(-6, 135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepLeftBack(90, 0, 15);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(57, 90, 0); //0.8 56
    intake.stop();

    drive.withTurnGains(1.35, 0, 0, 30).turn(180);
    drive.withGains(0.3, 0, 0, 40).moveWithVision(10, 180, 0); //8
    drive.timedDrive(100, 30, 30);
    intake.score(ONE_BALL);

    drive.withGains(0.15, 0, 0, 30).move(-14, 180, 0);
    drive.withTurnGains(1.35, 0, 0, 30).turn(270);
    drive.move(26, 270, 1, false, true);                       //25
    drive.withTurnGains(1.6, 0, 0, 80).sweepLeft(225, 30, 20); //1.75
    intake.intake(127, 127, 0);
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).moveWithVision(10, 225, 0);
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
    drive.withTurnGains(1.45, 0, 0, 30).sweepRight(135, 0, 2);
    drive.withGains(0.35, 0, 0, 40).move(9, 135, 0);
    drive.timedDrive(200, 30, 30); //300
    intake.intake(127, -127, -100);
    wait(100);
    intake.stop();
    intake.scoreWithVision(750);
    while (topBallDetected(RED_ID) && time < 1000)
    {
        if (time < 300)
        {
            drive.drivePower(40, 40);
        }
        else if (time < 600)
        {
            drive.drivePower(-25, -25);
        }
        else
        {
            drive.drivePower(10, 10);
        }
        intake.intake(127, 127, 0);
        wait(1);
        time++;
    }
    intake.intake(0, 127, 0);
    drive.drivePower(-10, -10);
    intake.scoreWithVision();
    time = 0;
    while (topBallDetected(RED_ID) && time < 1000)
    {
        if (time < 300)
        {
            drive.drivePower(40, 40);
        }
        else if (time < 600)
        {
            drive.drivePower(-25, -25);
        }
        else
        {
            drive.drivePower(10, 10);
        }
        intake.intake(127, 127, 0);
        wait(1);
        time++;
    }
    intake.intake(127, 0, 0);

    drive.move(-6, 135, 0, false, true); //10
    intake.intake(-127, 0, 0);
    drive.withTurnGains(5, 0, 0, 100).sweepLeftBack(90, 0, 15);
    drive.withCorrection(0.6).withGains(8, 0, 0, 35).moveBackToYCoord(55, 90, 0); //0.8
    intake.stop();

    drive.withTurnGains(1.45, 0, 0, 25).turn(180);
    drive.withGains(0.3, 0, 0, 40).moveWithVision(10, 180, 0); //8
    drive.timedDrive(100, 30, 30);
    intake.stop();
    intake.scoreWithVision();
    time = 0;
    while (topBallDetected(RED_ID) && time < 1000)
    {
        intake.intake(127, 0, 0);
        if (time < 300)
        {
            drive.drivePower(40, 40);
        }
        else
        {
            drive.drivePower(0, 0);
        }
        wait(1);
        time++;
    }
    drive.drivePower(-10, -10);
    intake.stop();
    intake.scoreWithVision();
    time = 0;
    while (topBallDetected(RED_ID) && time < 1000)
    {
        intake.intake(127, 0, 0);
        if (time < 200)
        {
            drive.drivePower(30, 30);
        }
        else
        {
            drive.drivePower(0, 0);
        }
        wait(1);
        time++;
    }
    intake.intake(127, 0, 0);

    drive.withGains(0.25, 0, 0, 30).move(-14, 180, 0);
    drive.withTurnGains(1.35, 0, 0, 25).turn(270);
    intake.intake(127, 127, -127);
    drive.withGains(0.35, 0, 0, 75).move(21, 270, 0, false, true); //25
    drive.withTurnGains(1.6, 0, 0, 80).sweepLeft(225, 30, 20);     //1.75
    intake.intake(127, 127, 0);
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).moveWithVision(10, 225, 0);
    drive.timedDrive(750, 40, 40); //1000
    intake.stop();
    drive.drivePower(30, 30);
    intake.score(ONE_BALL);
    intake.intake(-127, -127, 0);
    drive.move(-24, 225, 1);
}

/*
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
*/

void programming_skills()
{
    int time = 0;
    setCoordinates(16, 50, 90);
    intake.intake(127, 127, -127);
    wait(100);
    intake.intake(127, 127, 0);
    drive.withTurnGains(2, 0, 0, 40).turn(65); //70
    intake.intake(127, 127, 0);
    drive.withGains(0.2, 0, 0, 30).move(39, 68, 0); //38
    drive.timedDrive(200, 30, 30);
    intake.intake(127, 0, 0);
    // drive.withGains(0.2, 0, 0, 30).move(-12, 65, 0); //-12
    drive.withGains(9, 0, 0, 30).moveBackToYCoord(22, 65, 0); //-12
    drive.withTurnGains(1.6, 0, 0, 30).turn(135);

    drive.withGains(0.2, 0, 0, 40).moveWithVision(13, 135, 0, BLUE_ID, false, true); //13, 0.15
    drive.timedDrive(300, 30, 30);

    intake.intake(0, -100, -127); //-75
    wait(200);
    intake.stop();
    intake.frontRollers(127);

    drive.drivePower(-15, -15);
    intake.score(ONE_BALL);
    intake.stop();
    // drive.drivePower(-15, -15);
    // intake.scoreWithVision();
    // time = 0;
    // while (topBallDetected(RED_ID) && time < 500)
    // {
    //     if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
    //     {
    //         intake.intake(127, 0, 0);
    //     }
    //     else
    //     {
    //         intake.intake(127, 0, 0);
    //     }
    //     drive.drivePower(20, 20); //20
    //     wait(1);
    //     time++;
    // }
    // drive.drivePower(10, 10);
    // intake.stop();
    // intake.score(ONE_BALL, 1500);
    // wait(250);

    intake.intake(127, -30, 0);
    drive.withGains(10, 0, 0, 30).moveBackToYCoord(20, 135, 0, true); //10
    if (intakeBallDetected(BLUE_ID))
    {
        intake.intake(-127, -50, 0);
    }
    else
    {
        intake.intake(-127, 50, 0);
    }
    drive.waitForComplete();
    wait(100);
    drive.withTurnGains(1.15, 0, 0, 20).turn(0); //1, 20
    intake.intake(127, 127, 0);
    drive.withGains(8, 0, 0, 100).move(15, 0, 0, false, true); //24
    intake.intake(127, 0, 0);
    drive.withGains(3, 0, 0, 30).moveWithVisionToXCoord(66, 0, 0); //66, 30

    // wait(250);
    // drive.untilLineDetected(-35, 5000); //30
    // wait(150);
    // intake.intake(127, 0, 0);
    // wait(50);
    drive.withTurnGains(1.45, 0, 0, 22).turn(90); //1.35 , 20
    drive.withGains(0.3, 0, 0, 50).move(6, 90, 0, false, true);
    intake.stop();
    drive.timedDrive(250, 35, 35); //250
    intake.brake(INTAKE);
    //intake.stop();

    //get indexer unstuck
    intake.indexer(-127, 50);
    intake.score(TWO_BALLS);
    setCoordinates(getX(), 2, getTheta());
    intake.frontRollers(127);
    drive.timedDrive(200, 50, 50); //100, 60, 60
    intake.intake(127, -5, 0);
    wait(100);

    drive.withGains(0.2, 0, 0, 30).move(-17, 90, 0); //-18

    drive.withTurnGains(1.25, 0, 0, 30).turn(0);
    intake.intake(127, 127, -127);

    drive.withGains(0.1, 0, 0, 40).moveWithVision(40, 0, 0, RED_ID, true, true); //38
    if (intakeBallDetected(RED_ID))
    {
        intake.intake(127, 0, 0);
    }
    drive.waitForComplete();
    intake.intake(127, 0, 0);
    drive.untilLineDetected(30); //30
    setCoordinates(120, getY(), 0);
    intake.intake(127, 0, 0);
    drive.withTurnGains(1.25, 0, 0, 25).turn(90);                          //1.3, 30
    drive.withGains(0.175, 0, 0, 40).withCorrection(0.35).move(18, 55, 1); //0.35, 50
    drive.timedDrive(300, 50, 50);
    // drive.withTurnGains(1.25, 0, 0, 20).turn(110);
    // intake.intake(127, 0, 0);
    // drive.withGains(0.2, 0, 0, 25).move(24, 115, 1); //22
    // intake.intake(127, 0, 0);
    // drive.timedDrive(300, 30, 30);
    // // drive.withGains(0.2, 0, 0, 30).move(-15, 115, 0);
    // drive.withGains(0.2, 0, 0, 30).move(-12, 115, 0);
    // drive.withTurnGains(1.6, 0, 0, 30).turn(40);

    // drive.withGains(0.175, 0, 0, 30).moveWithVision(15, 45, 0, BLUE_ID); //17
    // drive.timedDrive(300, 30, 30);
    // intake.stop();

    intake.scoreWithVision(1000);
    //intake.intake(-127, -127, 0);

    intake.intake(127, -127, -60);
    drive.withGains(0.2, 0, 0, 35).move(-10, 45, 0); //30
    if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
    {
        intake.intake(50, -100, -50);
    }
    //wait(300);
    //intake.intake(127, 127, 0);
    drive.withTurnGains(1.25, 0, 0, 25).turn(-125); //1.25, 30
    //intake.intake(127, 127, 40);
    intake.intake(127, 127, -127);
    drive.withGains(0.25, 0, 0, 80).move(22, -120, 0, false, true); //20
    intake.intake(127, 127, 0);
    drive.withGains(6, 0, 0, 30).moveWithVisionToYCoord(51, -120, 0); //53

    drive.withTurnGains(1.2, 0, 0, 25).turn(0); //1.35, 25
    intake.intake(127, 0, 0);
    drive.withGains(0.25, 0, 0, 100).move(9, 0, 0, false, true);
    drive.withGains(0.2, 0, 0, 30).moveWithVision(16, 0, 0, BLUE_ID, false, true);
    //drive.move(28, 0, 2);
    intake.stop();
    drive.timedDrive(300, 30, 30);

    intake.score(ONE_BALL, 1000); //500
    intake.intake(127, 0, 0);
    drive.timedDrive(50, 35, 35); //100, 40, 40

    drive.untilLineDetected(-30);
    setCoordinates(120, getY(), 0);
    intake.intake(127, 127, -127);
    drive.withTurnGains(1.25, 0, 0, 30).turn(-90);
    intake.intake(127, 127, 0);

    drive.withGains(0.18, 0, 0, 35).moveWithVision(30, -93, 0); //29
    drive.withTurnGains(1.4, 0, 0, 25).turn(-110);
    intake.intake(127, 0, 0);
    drive.withGains(0.2, 0, 0, 35).move(25, -110, 1, false, true); //22
    drive.timedDrive(300, 30, 30);
    drive.withGains(0.2, 0, 0, 30).move(-12, -120, 0);
    drive.withTurnGains(1.6, 0, 0, 30).turn(-45);

    drive.withGains(0.2, 0, 0, 30).moveWithVision(10, -45, 0, BLUE_ID); //12
    drive.timedDrive(400, 35, 35);

    // setCoordinates(132, getY(), getTheta());
    // intake.intake(127, 127, -127);
    // drive.withGains(0.25, 0, 0, 30).move(-6, 0, 1); //8

    // drive.withTurnGains(1.35, 0, 0, 20).turn(-90); //1.25
    // intake.intake(127, 127, -127);
    // drive.withGains(0.25, 0, 0, 75).move(20, -90, 0, false, true); //18
    // intake.intake(127, 127, 0);
    // drive.withTurnGains(2, 0, 0, 75).sweepRight(-45, 0, 15);
    // drive.withGains(0.2, 0, 0, 35).move(8, -45, 0);
    // intake.stop();
    // drive.timedDrive(400, 35, 35);

    intake.score(ONE_BALL, 500);
    drive.drivePower(45, 45);
    intake.frontRollers(127);
    wait(250);
    intake.intake(-127, 0, 0);

    drive.withCorrection(0.3).withGains(6, 0, 0, 35).moveBackToYCoord(79, -90, 0); //79 , 0.3, 35
    intake.intake(-127, 30, 0);
    drive.withTurnGains(1.35, 0, 0, 30).turn(-180); //1.25
    intake.intake(127, 0, 0);
    drive.withGains(8, 0, 0, 45).moveWithVisionToXCoord(82, -180, 0, RED_ID, false, true); //25
    drive.untilLineDetected(30, 5000);                                                     //30
    setCoordinates(60, getY(), -180);
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

    drive.withTurnGains(1.35, 0, 0, 30).turn(-90);
    drive.timedDrive(800, -60, -60); //600

    // drive.withGains(0.3, 0, 0, 50).move(5, -90, 0);
    // drive.timedDrive(800, -60, -60);

    // drive.withGains(0.3, 0, 0, 50).move(5, -90, 0);
    // drive.timedDrive(600, -60, -60);

    //intake.frontRollers(127);
    drive.withGains(0.15, 0, 0, 30).move(38, -86, 0, true, true); //38, -92
    // drive.withGains(0.25, 0, 0, 100).move(12, -90, 0, false, true); //10
    // drive.withGains(0.25, 0, 0, 40).moveWithVision(12, -90, 0, BLUE_ID, false, true);
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
    drive.timedDrive(500, 35, 35);
    intake.stop();

    intake.score(TWO_BALLS);
    intake.frontRollers(127);
    drive.timedDrive(250, 50, 50);
    intake.intake(127, 0, 0);

    drive.move(-5, -90, 0, false, true);
    drive.withCorrection(0.15).withGains(0.2, 0, 0, 30).move(-32, -50, 0); //-32, 0.4
    intake.stop();
    drive.withTurnGains(1.345, 0, 0, 30).turn(50); //1.25, 20
    intake.intake(127, 50, 0);
    //drive.move(6, 50, 0, false, true);           //8
    drive.timedDrive(750, 60, 60); //500
    intake.stop();

    intake.score(ONE_BALL);
    drive.timedDrive(500, 60, 60);

    //intake.intake(127, 127, -127);
    drive.withGains(0.25, 0, 0, 35).move(-6, 60, 0);
    drive.withTurnGains(1.45, 0, 0, 30).turn(125); //1.25
    intake.intake(127, 127, -127);

    drive.withGains(0.2, 0, 0, 35).move(12, 125, 0, false, true); //6
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.35, 0, 0, 30).turn(180);
    // drive.sweepRight(180, -20, 10);                                                   //-15
    //drive.withCorrection(0.1).withGains(0.2, 0, 0, 40).move(32, 181, 0, false, true); //32
    drive.withCorrection(0.1).withGains(0.3, 0, 0, 90).move(20, 182, 0, false, true); //32
    drive.withGains(0.15, 0, 0, 40).moveWithVision(14, 181, 0, BLUE_ID, false, true); //32
    intake.stop();
    drive.timedDrive(300, 60, 60); //750, 30, 30

    intake.score(ONE_BALL);
    intake.intake(127, 127, -127);
    drive.timedDrive(50, 55, 55);
    //setTheta(180);

    drive.withGains(0.2, 0, 0, 30).move(-6, 180, 0); //-10
    drive.withTurnGains(1.35, 0, 0, 20).turn(270);
    //intake.intake(127, 127, 0);
    drive.withGains(0.2, 0, 0, 60).moveWithVision(23, 270, 0, RED_ID, false, true); //22
    intake.intake(127, 127, 0);
    drive.withTurnGains(2, 0, 0, 75).sweepLeft(225, 0, 15);
    drive.withGains(0.2, 0, 0, 40).moveWithVision(11, 225, 0, BLUE_ID);
    intake.intake(127, 0, 0);
    drive.timedDrive(300, 40, 40);
    drive.drivePower(30, 30);

    //drive.drivePower(45, 45);
    intake.score(ONE_BALL, 500);
    time = 0;
    while (time < 2000)
    {
        if (intakeBallDetected(BLUE_ID) || intakeLimit.get_value() == 1)
        {
            break;
        }
        if (time > 500 && time < 1200)
        {
            drive.drivePower(-20, -20);
        }
        else
        {
            drive.drivePower(30, 30);
        }
        intake.intake(127, 127, 0);
        wait(1);
        time++;
    }
    intake.stop();
    drive.move(-24, 225, 0);
}

void programming_skills_126()
{
    int time = 0;
    setCoordinates(16, 55, 90);
    intake.intake(127, 127, -127);
    wait(100);
    intake.intake(127, 127, 0);
    drive.withTurnGains(3, 0, 0, 35).turn(70); //70
    // intake.intake(127, 127, 0);
    drive.withGains(0.2, 0, 0, 30).moveWithVision(46, 70, 0); //38
    drive.timedDrive(100, 35, 35);
    intake.intake(127, 0, 0);
    // drive.withGains(0.2, 0, 0, 30).move(-12, 65, 0); //-12
    drive.withGains(10, 0, 0, 40).moveBackToYCoord(18, 65, 0); //-12
    drive.withTurnGains(1.6, 0, 0, 30).turn(135);

    drive.withGains(0.2, 0, 0, 40).moveWithVision(13, 135, 0, BLUE_ID, false, true); //13, 0.15
    drive.timedDrive(300, 30, 30);

    intake.intake(0, -100, -127); //-75
    wait(200);
    intake.stop();
    intake.frontRollers(127);

    drive.drivePower(-15, -15);
    // intake.score(ONE_BALL);
    // intake.stop();
    drive.drivePower(40, 40);
    intake.scoreWithVision();
    time = 0;
    while (time < 1500)
    {
        if (topBallDetected(RED_ID))
        {
            break;
        }
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            intake.intake(127, 0, 0);
        }
        else
        {
            intake.intake(127, 127, 0);
        }
        drive.drivePower(30, 30); //20
        wait(1);
        time++;
    }
    drive.drivePower(10, 10);
    intake.intake(127, 127, 0);
    intake.score(ONE_BALL, 1500);
    wait(250);

    intake.intake(127, -30, 0);
    drive.withGains(0.2, 0, 0, 40).move(-10, 135, 0);
    intake.intake(-127, -50, 0);
    drive.withGains(10, 0, 0, 35).moveBackToYCoord(22, 135, 0); //10

    drive.withTurnGains(1.15, 0, 0, 30).turn(-30); //1, 20
    intake.intake(127, 127, 0);
    drive.withGains(8, 0, 0, 75).move(12, 0, 0, false, true); //24
    intake.intake(127, 0, 0);
    drive.withGains(6, 0, 0, 25).moveWithVisionToXCoord(64, 0, 0); //66, 30

    // wait(250);
    // drive.untilLineDetected(-35, 5000); //30
    // wait(150);
    // intake.intake(127, 0, 0);
    // wait(50);
    drive.withTurnGains(1.2, 0, 0, 30).turn(90); //1.35 , 20
    drive.withGains(2, 0, 0, 75).moveWithVision(12, 90, 0, BLUE_ID, false, true);
    drive.withGains(1.5, 0, 0, 30).moveWithVision(12, 90, 0, BLUE_ID, false, true);
    intake.stop();
    drive.timedDrive(300, 35, 35); //250
    intake.brake(INTAKE);
    //intake.stop();

    //get indexer unstuck
    intake.indexer(-127, 50);
    intake.score(TWO_BALLS);
    setCoordinates(getX(), 2, getTheta());
    intake.frontRollers(127);
    drive.timedDrive(200, 75, 75); //100, 60, 60
    intake.intake(127, -5, 0);

    drive.withGains(0.2, 0, 0, 40).move(-8, 90, 0, false, true); //-18
    intake.intake(-127, 50, 0);
    drive.withGains(6, 0, 0, 35).moveBackToYCoord(19, 90, 0);

    drive.withTurnGains(1.25, 0, 0, 30).turn(0);
    intake.intake(127, 127, 0);

    drive.withGains(0.1, 0, 0, 40).moveWithVision(40, 0, 0, RED_ID, true, true); //38
    if (intakeBallDetected(RED_ID))
    {
        intake.intake(127, 0, 0);
    }
    drive.waitForComplete();
    intake.intake(127, 0, 0);
    drive.untilLineDetected(30); //30
    setCoordinates(120, getY(), 0);
    intake.intake(127, 0, 0);
    // drive.withTurnGains(1.25, 0, 0, 25).turn(90);                          //1.3, 30
    // drive.withGains(0.175, 0, 0, 40).withCorrection(0.35).move(18, 55, 1); //0.35, 50
    // drive.timedDrive(300, 50, 50);
    drive.withTurnGains(1.25, 0, 0, 30).turn(115);
    intake.intake(127, 0, 0);
    drive.withGains(0.2, 0, 0, 35).moveWithVision(25, 115, 1); //22
    intake.intake(127, 0, 0);
    drive.timedDrive(200, 30, 30);
    // drive.withGains(0.2, 0, 0, 30).move(-15, 115, 0);
    drive.withGains(0.2, 0, 0, 35).move(-12, 115, 0);
    drive.withTurnGains(1.75, 0, 0, 30).turn(40);

    drive.withGains(0.175, 0, 0, 30).moveWithVision(11, 45, 0, BLUE_ID); //17
    drive.timedDrive(300, 30, 30);

    // intake.scoreWithVision(1000);
    //intake.intake(-127, -127, 0);
    drive.drivePower(40, 40);
    intake.scoreWithVision();
    time = 0;
    while (time < 1500)
    {
        if (topBallDetected(RED_ID))
        {
            break;
        }
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            intake.intake(127, 0, 0);
        }
        else
        {
            intake.intake(127, 127, 0);
        }
        drive.drivePower(30, 30); //20
        wait(1);
        time++;
    }
    drive.drivePower(10, 10);
    intake.intake(127, 127, 0);
    intake.score(ONE_BALL, 1500);
    wait(250);

    intake.intake(-127, -127, 0);
    drive.withGains(0.2, 0, 0, 35).move(-10, 45, 0); //30
    wait(250);
    intake.intake(127, 127, 0);
    // if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
    // {
    //     intake.intake(50, -100, -50);
    // }
    //wait(300);
    //intake.intake(127, 127, 0);
    drive.withTurnGains(1.25, 0, 0, 25).turn(-125); //1.25, 30
    //intake.intake(127, 127, 40);
    intake.intake(127, 0, 0);
    drive.withGains(0.25, 0, 0, 100).move(22, -120, 0, false, true); //20
    intake.intake(127, 127, 0);
    drive.withGains(6, 0, 0, 30).moveWithVisionToYCoord(51, -120, 0); //53

    drive.withTurnGains(1.2, 0, 0, 25).turn(0); //1.35, 25
    intake.intake(127, 0, 0);
    drive.withGains(0.25, 0, 0, 100).move(9, 0, 0, false, true);
    drive.withGains(0.2, 0, 0, 30).moveWithVision(16, 0, 0, BLUE_ID, false, true);
    //drive.move(28, 0, 2);
    intake.stop();
    drive.timedDrive(300, 30, 30);

    intake.score(TWO_BALLS, 1000); //500
    intake.intake(127, 0, 0);
    drive.timedDrive(150, 75, 75); //100, 40, 40

    drive.untilLineDetected(-30);
    setCoordinates(120, getY(), 0);
    intake.intake(127, 127, -127);
    drive.withTurnGains(1.25, 0, 0, 30).turn(-90);
    intake.intake(127, 127, 0);

    drive.withGains(0.18, 0, 0, 35).moveWithVision(30, -93, 0); //29
    drive.withTurnGains(1.4, 0, 0, 30).turn(-110);
    intake.intake(127, 0, 0);
    drive.withGains(0.2, 0, 0, 35).move(25, -110, 1, false, true); //22
    drive.timedDrive(300, 30, 30);
    drive.withGains(0.2, 0, 0, 35).move(-12, -120, 0);
    drive.withTurnGains(1.6, 0, 0, 30).turn(-45);

    drive.withGains(0.2, 0, 0, 35).moveWithVision(10, -45, 0, BLUE_ID); //12
    drive.timedDrive(400, 35, 35);
    drive.drivePower(40, 40);
    intake.scoreWithVision();
    time = 0;
    while (time < 1500)
    {
        if (topBallDetected(RED_ID))
        {
            break;
        }
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
        {
            intake.intake(127, 0, 0);
        }
        else
        {
            intake.intake(127, 127, 0);
        }
        drive.drivePower(30, 30); //20
        wait(1);
        time++;
    }
    drive.drivePower(10, 10);
    intake.intake(127, 127, 0);
    intake.score(ONE_BALL, 1500);
    wait(250);

    intake.intake(127, -30, 0);
    drive.withGains(0.2, 0, 0, 40).move(-10, -45, 0);
    intake.intake(-127, -127, -127);
    drive.withGains(10, 0, 0, 35).moveBackToYCoord(100, -45, 0); //10

    drive.withTurnGains(1.15, 0, 0, 30).turn(-210); //1, 20
    intake.intake(127, 127, 0);
    drive.withGains(8, 0, 0, 75).move(12, -210, 0, false, true); //24
    intake.intake(127, 0, 0);
    drive.withGains(6, 0, 0, 30).moveWithVisionToXCoord(72, 0, 0); //66, 30

    // wait(250);
    // drive.untilLineDetected(-35, 5000); //30
    // wait(150);
    // intake.intake(127, 0, 0);
    // wait(50);
    drive.withTurnGains(1.35, 0, 0, 30).turn(-90); //1.35 , 20
    drive.withGains(2, 0, 0, 75).moveWithVision(12, -90, 0, BLUE_ID, false, true);
    drive.withGains(1.5, 0, 0, 30).moveWithVision(12, -90, 0, BLUE_ID, false, true);
    intake.stop();
    drive.timedDrive(300, 35, 35); //250
    intake.brake(INTAKE);
    //intake.stop();

    // setCoordinates(132, getY(), getTheta());
    // intake.intake(127, 127, -127);
    // drive.withGains(0.25, 0, 0, 30).move(-6, 0, 1); //8

    // drive.withTurnGains(1.35, 0, 0, 20).turn(-90); //1.25
    // intake.intake(127, 127, -127);
    // drive.withGains(0.25, 0, 0, 75).move(20, -90, 0, false, true); //18
    // intake.intake(127, 127, 0);
    // drive.withTurnGains(2, 0, 0, 75).sweepRight(-45, 0, 15);
    // drive.withGains(0.2, 0, 0, 35).move(8, -45, 0);
    // intake.stop();
    // drive.timedDrive(400, 35, 35);

    intake.score(TWO_BALLS, 500);
    drive.drivePower(45, 45);
    intake.frontRollers(127);
    wait(250);
    intake.intake(-127, 0, 0);

    intake.frontRollers(127);
    drive.timedDrive(200, 75, 75); //100, 60, 60
    intake.intake(127, -5, 0);

    //Start of alternate
    // drive.withGains(0.2, 0, 0, 40).move(-8, -90, 0, false, true); //-18
    // intake.intake(-127, 50, 0);
    // drive.withGains(6, 0, 0, 35).moveBackToYCoord(88, -90, 0);

    // drive.withTurnGains(1.25, 0, 0, 30).turn(-180);
    // intake.intake(127, 127, 0);

    // drive.withGains(0.1, 0, 0, 40).moveWithVision(40, -180, 0, RED_ID, true, true); //38
    // if (intakeBallDetected(RED_ID))
    // {
    //     intake.intake(127, 0, 0);
    // }
    // drive.waitForComplete();
    // intake.intake(127, 0, 0);
    // drive.untilLineDetected(30); //30
    // setCoordinates(24, getY(), 0);
    // intake.intake(127, 0, 0);
    // // drive.withTurnGains(1.25, 0, 0, 25).turn(90);                          //1.3, 30
    // // drive.withGains(0.175, 0, 0, 40).withCorrection(0.35).move(18, 55, 1); //0.35, 50
    // // drive.timedDrive(300, 50, 50);
    // drive.withTurnGains(1.25, 0, 0, 30).turn(-65);
    // intake.intake(127, 0, 0);
    // drive.withGains(0.2, 0, 0, 35).moveWithVision(25, -65, 1); //22
    // intake.intake(127, 0, 0);
    // drive.timedDrive(200, 30, 30);
    // // drive.withGains(0.2, 0, 0, 30).move(-15, 115, 0);
    // drive.withGains(0.2, 0, 0, 35).move(-12, -65, 0);
    // drive.withTurnGains(1.75, 0, 0, 30).turn(-140);

    // drive.withGains(0.175, 0, 0, 30).moveWithVision(11, -145, 0, BLUE_ID); //17
    // drive.timedDrive(300, 30, 30);

    // // intake.scoreWithVision(1000);
    // //intake.intake(-127, -127, 0);
    // drive.drivePower(40, 40);
    // intake.scoreWithVision();
    // time = 0;
    // while (time < 1500)
    // {
    //     if (topBallDetected(RED_ID))
    //     {
    //         break;
    //     }
    //     if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
    //     {
    //         intake.intake(127, 0, 0);
    //     }
    //     else
    //     {
    //         intake.intake(127, 127, 0);
    //     }
    //     drive.drivePower(30, 30); //20
    //     wait(1);
    //     time++;
    // }
    // drive.drivePower(10, 10);
    // intake.intake(127, 127, 0);
    // intake.score(ONE_BALL, 1500);
    // wait(250);

    // intake.intake(-127, -127, 0);
    // drive.withGains(0.2, 0, 0, 35).move(-10, -145, 0); //30
    // wait(250);
    // intake.intake(127, 127, 0);
    // // if (intakeLimit.get_value() == 1 || topRollerLine.get_value() < BALL_DETECTED_SIGNATURE)
    // // {
    // //     intake.intake(50, -100, -50);
    // // }
    // //wait(300);
    // //intake.intake(127, 127, 0);
    // drive.withTurnGains(1.25, 0, 0, 25).turn(-305); //1.25, 30
    // //intake.intake(127, 127, 40);
    // intake.intake(127, 0, 0);
    // drive.withGains(0.25, 0, 0, 100).move(22, -305, 0, false, true); //20
    // intake.intake(127, 127, 0);
    // drive.withGains(6, 0, 0, 30).moveWithVisionToYCoord(51, -120, 0); //53

    // drive.withTurnGains(1.2, 0, 0, 25).turn(-180); //1.35, 25
    // intake.intake(127, 0, 0);
    // drive.withGains(0.25, 0, 0, 100).move(9, -180, 0, false, true);
    // drive.withGains(0.2, 0, 0, 30).moveWithVision(16, -180, 0, BLUE_ID, false, true);
    // //drive.move(28, 0, 2);
    // intake.stop();
    // drive.timedDrive(300, 30, 30);

    // intake.score(TWO_BALLS, 1000); //500
    // intake.intake(127, 0, 0);
    // drive.timedDrive(150, 75, 75); //100, 40, 40
    //end of alternate

    drive.move(-5, -90, 0, false, true);
    drive.withCorrection(0.15).withGains(0.2, 0, 0, 30).move(-32, -50, 0); //-32, 0.4
    intake.stop();
    drive.withTurnGains(1.345, 0, 0, 30).turn(50); //1.25, 20
    intake.intake(127, 50, 0);
    //drive.move(6, 50, 0, false, true);           //8
    drive.timedDrive(750, 60, 60); //500
    intake.stop();

    intake.score(ONE_BALL);
    drive.timedDrive(500, 60, 60);

    //intake.intake(127, 127, -127);
    drive.withGains(0.25, 0, 0, 35).move(-6, 60, 0);
    drive.withTurnGains(1.45, 0, 0, 30).turn(125); //1.25
    intake.intake(127, 127, -127);

    drive.withGains(0.2, 0, 0, 35).move(12, 125, 0, false, true); //6
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.35, 0, 0, 30).turn(180);
    // drive.sweepRight(180, -20, 10);                                                   //-15
    //drive.withCorrection(0.1).withGains(0.2, 0, 0, 40).move(32, 181, 0, false, true); //32
    drive.withCorrection(0.1).withGains(0.3, 0, 0, 90).move(20, 182, 0, false, true); //32
    drive.withGains(0.15, 0, 0, 40).moveWithVision(14, 181, 0, BLUE_ID, false, true); //32
    intake.stop();
    drive.timedDrive(300, 60, 60); //750, 30, 30

    intake.score(ONE_BALL);
    intake.intake(127, 127, -127);
    drive.timedDrive(50, 55, 55);
    //setTheta(180);

    drive.withGains(0.2, 0, 0, 30).move(-6, 180, 0); //-10
    drive.withTurnGains(1.35, 0, 0, 20).turn(270);
    //intake.intake(127, 127, 0);
    drive.withGains(0.2, 0, 0, 60).moveWithVision(23, 270, 0, RED_ID, false, true); //22
    intake.intake(127, 127, 0);
    drive.withTurnGains(2, 0, 0, 75).sweepLeft(225, 0, 15);
    drive.withGains(0.2, 0, 0, 40).moveWithVision(11, 225, 0, BLUE_ID);
    intake.intake(127, 0, 0);
    drive.timedDrive(300, 40, 40);
    drive.drivePower(30, 30);

    //drive.drivePower(45, 45);
    intake.score(ONE_BALL, 500);
    time = 0;
    while (time < 2000)
    {
        if (intakeBallDetected(BLUE_ID) || intakeLimit.get_value() == 1)
        {
            break;
        }
        if (time > 500 && time < 1200)
        {
            drive.drivePower(-20, -20);
        }
        else
        {
            drive.drivePower(30, 30);
        }
        intake.intake(127, 127, 0);
        wait(1);
        time++;
    }
    intake.stop();
    drive.move(-24, 225, 0);
}

void autonomous()
{
    //two_goal_and_middle_left();
    programming_skills_126();
    //home_row_left_no_cycle_with_center();
    // programming_skills();
    // two_goal_and_middle_left();
    //corner_side_center_right();
    //home_row_left_no_cycle();
    // home_row_right_no_cycle();
    //intake.frontRollers(127);
    //drive.withGains(0.2, 0, 0, 20).moveWithVision(24, 0, 0);
    // while (1)
    // {
    //     double baseTurnBias = driverBaseAngle(RED_ID);

    //     int n = vision3.get_object_count();
    //     pros::lcd::print(0, "%d ", n);

    //     if (n > 0)
    //     {
    //         pros::vision_object_s_t obj = vision3.get_by_sig(0, RED_ID);
    //         pros::lcd::print(1, "x: %3d y: %3d", obj.x_middle_coord, obj.y_middle_coord);
    //         pros::lcd::print(2, "w: %3d h: %3d", obj.width, obj.height);
    //     }
    //     else
    //     {
    //         pros::lcd::print(1, "no objects");
    //         pros::lcd::print(2, "          ");
    //     }

    //     pros::delay(20);

    //     drive.right(80 - baseTurnBias);
    //     drive.left(80 + baseTurnBias);
    // }
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
    //     home_row_left_no_cycle_with_center();
    //     break;
    // case 4:
    //     two_goal_right();
    //     break;
    // case 5:
    //     two_goal_and_middle_left();
    //     break;
    // case 6:
    //     programming_skills();
    //     //two_goal_and_middle_left();
    //     break;
    // }
}
