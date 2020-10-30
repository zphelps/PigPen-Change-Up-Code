#include "main.h"
/*
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
    left(speed);
    right(speed);
    wait(10000);
}

void twoGoalAutonRight()
{
    setCoordinates(0, 24, 0);
    sweepRight(135, 0, 1.5, 25);
    frontRollers(127);
    move(8, 135, 0, 0.2, 35, 0.2);
    timedDrive(300, 30); //300
    frontRollers(0);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(8, 135, 0, true); //10
    fullIntake(127);
    sweepLeftBack(90, 0, 15, 5, 100);
    moveBackToYCoord(56, 90, 0, 6, 30, 0.6); //0.8
    //58

    indexer(-127);
    turn(180);
    indexer(0);
    intakeFullStop();
    move(10, 180, 0, 0.3, 40, 0.2); //8
    timedDrive(100, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, 180, 2, 0.15, 30, 0.2);
}

void twoGoalAutonLeft()
{
    setCoordinates(0, 125, 0);
    sweepLeft(-135, 0);
    frontRollers(127);
    move(8, -135, 0);
    timedDrive(300, 30);
    frontRollers(0);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(10, -135, 0, true);
    fullIntake(127);
    sweepRightBack(-90, 0, 20, 5, 100);
    moveBackToYCoord(90, -90, 0, 6, 30, 0.8);

    turn(-180);
    intakeFullStop();
    move(10, -180, 0, 0.3, 40, 0.2); //8
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, -180, 0, 0.25, 35, 0.8);
}
*/
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
    drive.move(20, -270, 1, false, true);                        //23
    drive.withTurnGains(1.6, 0, 0, 80).sweepRight(-225, 30, 20); //1.75
    intake.intake(127, 127, 0);
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).move(20, -225, 0);
    drive.timedDrive(500, 40, 40);
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
    drive.move(20, -270, 1, true, true); //23
    intake.reverseFor(500, true);
    drive.waitForComplete();
    drive.withTurnGains(1.6, 0, 0, 80).sweepRight(-225, 30, 20); //1.75
    intake.intake(127, 127, 0);                                  //
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).move(20, -225, 0);
    drive.timedDrive(500, 40, 40);
    drive.drivePower(35, 35);
    time = 0;
    while (time < 1500)
    {
        intake.intake(127, 127, 0);
        intake.logic();
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE && intakeBallDetected(ANY_COLOR))
        {
            break;
        }
        wait(1);
        time++;
    }
    intake.stop();
    intake.score(TWO_BALLS);
    drive.move(-24, -225, 1);
}
void home_row_right_no_cycle()
{
    int time = 0;
    setCoordinates(0, 24, 0);
    intake.intake(127, 127, 0);
    drive.withTurnGains(1.5, 0, 0, 25).sweepRight(135, 0);
    drive.withGains(0.2, 0, 0, 35).move(8, 135, 0);
    drive.timedDrive(300, 30, 30); //300
    intake.stop();
    intake.score(ONE_BALL);
    intake.intake(-127, 0, 0);

    drive.move(-6, 135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepLeftBack(90, 0, 15);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(56, 90, 0); //0.8
    intake.stop();

    drive.turn(180);
    drive.withGains(0.3, 0, 0, 40).move(10, 180, 0); //8
    drive.timedDrive(100, 30, 30);
    intake.score(ONE_BALL);

    drive.withGains(0.15, 0, 0, 30).move(-14, 180, 2);
    drive.turn(270);
    drive.move(24, 270, 1, false, true);                       //23
    drive.withTurnGains(1.6, 0, 0, 80).sweepLeft(225, 30, 20); //1.75
    intake.intake(127, 127, 0);
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).move(10, 225, 0);
    drive.timedDrive(1000, 40, 40);
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
    //intake.score(ONE_BALL);

    intake.score(ONE_BALL);
    intake.intake(0, 127, 0);

    drive.move(-6, 135, 0, false, true); //10
    drive.withTurnGains(5, 0, 0, 100).sweepLeftBack(90, 0, 15);
    drive.withCorrection(0.6).withGains(6, 0, 0, 30).moveBackToYCoord(56, 90, 0); //0.8
    intake.stop();

    drive.turn(180);
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
    drive.turn(270);
    intake.intake(127, 127, -127);
    drive.move(23, 270, 1, false, true);
    drive.withTurnGains(1.6, 0, 0, 80).sweepLeft(227, 30, 20); //1.75
    intake.intake(127, 127, 0);
    drive.withCorrection(0.5).withGains(0.2, 0, 0, 40).move(6, 225, 0);
    drive.timedDrive(500, 40, 40);
    drive.drivePower(30, 30);
    time = 0;
    while (time < 1000)
    {
        intake.intake(127, 127, 0);
        if (topRollerLine.get_value() < BALL_DETECTED_SIGNATURE && intakeBallDetected(ANY_COLOR))
        {
            break;
        }
        wait(1);
        time++;
    }
    intake.stop();
    if (time >= 1000)
    {
        intake.score(ONE_BALL);
    }
    else if (time < 1000)
    {
        intake.score(TWO_BALLS);
    }
    intake.frontRollers(127);
    drive.timedDrive(250, 40, 40);
    drive.move(-24, 225, 1);
}
/*
void home_row_left_no_cycle()
{
    setCoordinates(0, 125, 0);
    sweepLeft(-135, 0);
    frontRollers(127);
    move(8, -135, 0, 0.2, 35, 0.2);
    frontRollers(0);
    timedDrive(300, 30);
    scoreOneBall();
    intake(90);
    frontRollers(-127);

    moveBack(8, -135, 0, true);
    fullIntake(127);
    sweepRightBack(-90, 0, 20, 5, 100);
    moveBackToYCoord(90, -90, 0, 6, 30, 0.8);

    turn(-180);
    intakeFullStop();
    move(10, -180, 0, 0.3, 40, 0.2); //8
    timedDrive(300, 30);
    scoreOneBall();
    intakeFullReverse();

    moveBack(14, -180, 2, 0.25, 35, 0.8);
    turn(-270);
    move(16, -270, 2, true); //18
    sweepRight(-225, 0, 20, 1.6, 80);
    frontRollers(127);
    move(10, -225, 0, 0.2, 40, 0.5);
    timedDrive(500, 40);
    scoreOneBall();
    wait(200);
    scoreOneBall();
    intakeFullReverse();
    moveBack(24, -225, 1);
}
*/
void programming_skills()
{
    int time = 0;
    setCoordinates(1, 24, 0);
    intake.intake(127, 127, 0);
    drive.withGains(0.15, 0, 0, 25).move(12, 0, 2);
    intake.intake(127, 127, 0);
    wait(100);
    intake.intake(127, 0, 0);
    drive.withTurnGains(1.75, 0, 0, 30).sweepRight(90, 0, 5);      //1.35, 35
    drive.withGains(0.3, 0, 0, 30).move(15, 90, 0);                //0.3
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

    drive.withGains(10, 0, 0, 30).moveBackToYCoord(9, 135, 0); //15
    intake.intake(-127, 50, 0);
    wait(200);
    drive.withTurnGains(1, 0, 0, 20).turn(0); //1
    intake.intake(127, 0, 0);
    drive.withGains(4.55, 0, 0, 30).moveToXCoord(60, 0, 0); //4

    drive.withTurnGains(1.25, 0, 0, 20).turn(90);
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
    drive.timedDrive(250, 40, 40);
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
        intake.intake(127, 127, 60);
    }
    intake.intake(127, 0, 0);
    drive.waitForComplete();

    drive.turn(90);
    intake.stop();
    drive.withGains(0.175, 0, 0, 35).withCorrection(0.25).move(18, 45, 1); //20
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
    drive.withTurnGains(1.25, 0, 0, 20).turn(-90);

    intake.intake(127, 75, 0);
    drive.moveToYCoord(54, -90, 2); //55

    drive.withTurnGains(1.25, 0, 0, 20).turn(0);
    intake.stop();
    drive.move(28, 0, 2);
    intake.stop();
    drive.timedDrive(300, 35, 35);

    intake.score(TWO_BALLS, 500);
    intake.intake(127, 0, 0);
    drive.timedDrive(250, 40, 40);
    setCoordinates(132, getY(), getTheta());
    drive.withGains(0.2, 0, 0, 30).move(-6, 0, 2); //8

    drive.withTurnGains(1.25, 0, 0, 20).turn(-90);
    intake.intake(127, 127, -127);
    drive.withGains(0.25, 0, 0, 75).move(20, -90, 0, false, true); //24
    intake.intake(127, 127, 0);
    drive.withTurnGains(2, 0, 0, 75).sweepRight(-45, 0, 15);
    drive.withGains(0.2, 0, 0, 35).move(8, -45, 0);
    intake.stop();
    drive.timedDrive(400, 35, 35);

    drive.drivePower(30, 30);
    intake.score(ONE_BALL, 500);
    intake.frontRollers(127);
    wait(250);

    drive.withCorrection(0.5).withGains(7, 0, 0, 30).moveBackToYCoord(80, -90, 0, true, false);
    intake.reverseFor(500, true);
    drive.waitForComplete();
    drive.withTurnGains(1.25, 0, 0, 25).turn(-180);
    intake.intake(127, 127, 0);
    drive.move(25, -180, 1);
    drive.untilLineDetected(30, 5000);
    wait(200);
    setCoordinates(60, getY(), -180);
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

    drive.withTurnGains(1.25, 0, 0, 20).turn(-90);
    drive.move(-6, -90, 0, false, true);
    drive.timedDrive(500, -40, -40);
    drive.timedDrive(500, 40, 40);
    drive.timedDrive(1000, -40, -40);

    intake.frontRollers(127);
    drive.withGains(0.2, 0, 0, 35).move(38, -92, 1);
    drive.timedDrive(350, 30, 30);
    intake.stop();

    intake.score(ONE_BALL);
    intake.frontRollers(127);
    drive.timedDrive(250, 75, 75);
    intake.intake(127, 0, 0);

    drive.move(-5, -90, 0, false, true);
    drive.withCorrection(0.4).move(-32, -50, 0); //-18
    intake.stop();
    drive.withTurnGains(1.25, 0, 0, 20).turn(50);
    drive.move(8, 50, 0, false, true);
    drive.timedDrive(300, 30, 30);

    intake.score(ONE_BALL);

    intake.intake(127, 127, -127);
    drive.withGains(0.2, 0, 0, 25).move(-6, 60, 0);
    drive.withTurnGains(1.25, 0, 0, 20).turn(125);
    intake.intake(127, 127, 0);

    drive.move(6, 125, 0, false, true);
    drive.sweepRight(180, -10, 10);
    drive.move(34, 180, 1);
    intake.stop();
    drive.timedDrive(350, 30, 30);

    intake.score(ONE_BALL);
    intake.intake(127, 127, -127);
    drive.timedDrive(250, 40, 40);

    drive.move(-8, 180, 0);
    drive.turn(270);
    intake.intake(127, 127, 0);
    drive.withGains(0.25, 0, 0, 75).move(22, 270, 0, false, true); //24
    intake.intake(127, 127, 0);
    drive.withTurnGains(2, 0, 0, 75).sweepLeft(225, 0, 15);
    drive.withGains(0.2, 0, 0, 35).move(6, 225, 0);
    intake.stop();
    drive.timedDrive(500, 35, 35);

    drive.drivePower(30, 30);
    intake.score(ONE_BALL, 500);
    drive.drivePower(40, 40);
    time = 0;
    while (time < 2000)
    {
        if (intakeBallDetected(BLUE_ID))
        {
            break;
        }
        intake.intake(127, 127, 0);
    }

    drive.move(-24, 225, 0);
}

void autonomous()
{
    // drive.move(24, 0, 0, false, true);
    // drive.untilLineDetected(30, 5000);
    // wait(250);
    // setTheta(0);
    //drive.turn(90);
    //drive.move(48, 0, 2);
    //programming_skills();
    //home_row_right_no_cycle();

    //drive.withTurnGains(1.75, 0, 0, 25).sweepRight(90, 0); //1.35, 35
    switch (autonIndex)
    {
    case 0:
        home_row_right_no_cycle();
        break;
    case 1:
        home_row_right_cycle();
        break;
    case 2:
        home_row_left_no_cycle();
        break;
    case 3:
        //home_row_left_cycle();
        home_row_left_cycle();
        break;
    case 4:
        programming_skills();
        break;
    }

    //intake_manager.remove();
}
