#include "Subsystems/Drivetrain.hpp"
#include "Utility/Pose.hpp"
#include "pros/misc.h"
#include "pros/screen.h"
#include <iostream>

Drivetrain::Drivetrain()
{
    // Construct the Motor objects
    // PORT 17 IS BROKEN FOR SOME REASON!!!
    rightFront = new pros::Motor(6, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    rightMiddle = new pros::Motor(19, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    rightMiddle->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    rightBack = new pros::Motor(4, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    rightBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    leftFront = new pros::Motor(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    leftFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    leftMiddle = new pros::Motor(11, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    leftMiddle->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    leftBack = new pros::Motor(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    leftBack->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // // Construct Odometry Encoder objects
    // forwardEncoder = new pros::ADIEncoder('A', 'B', false);
    // sideEncoder = new pros::ADIEncoder('C', 'D', false);

    driveDistancePID = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);
    turnAnglePID = new PIDController(TURN_P, TURN_I, TURN_D);
    driveTurnPID = new PIDController(DRIVE_TURN_P, DRIVE_TURN_I, DRIVE_TURN_D);

    // turnTarget = 0.0;
    driveDistancePID->setEpsilon(DRIVE_EPSILON);
    turnAnglePID->setEpsilon(TURN_EPSILON);
    driveTurnPID->setEpsilon(DRIVE_TURN_EPSILON);

    justResetFlag = false;

    // Construct Gyro object
    gyro = new pros::Imu(17);

    this->distanceSetpoint = 0;
    this->angleSepoint = 0;
    this->headingSetpoint = 0;

    prevTime = pros::millis();
}

Drivetrain::~Drivetrain()
{
}

void Drivetrain::updateDrivetrain(pros::Controller &driver)
{

    // Finite State Machine (FSM)

    odometryStep(driver);

    switch (mDriveState)
    {
        // The Operator Control state that allows the driver to have open loop control over the drivetrain

    case MOVE_DISTANCE:
    {
        this->currTime = pros::millis();
        double deltaTimeMs = this->currTime - this->prevTime;

        driveDistancePID->setTarget(this->distanceSetpoint);
        double driveOutput = driveDistancePID->stepPID(getAvgEncInches(), deltaTimeMs);
       
        // driveDistancePID->setTarget(this->headingSetpoint);
        // double headingOutput = driveDistancePID->stepPID(gyro->get_yaw(), deltaTimeMs);
        double headingOutput = 0.0;
        rightFront->move_velocity(driveOutput + headingOutput);
        rightMiddle->move_velocity((driveOutput + headingOutput) * (60.0 / 84.0));
        rightBack->move_velocity(driveOutput + headingOutput);

        leftFront->move_velocity(driveOutput - headingOutput);
        leftMiddle->move_velocity((driveOutput - headingOutput) * (60.0 / 84.0));
        leftBack->move_velocity(driveOutput - headingOutput);

        this->prevTime = this->currTime;

        justResetFlag = true;
        break;
    }

    case TURN_ANGLE:
    {
        this->currTime = pros::millis();
        double deltaTimeMs = this->currTime - this->prevTime;

        turnAnglePID->setTarget(this->angleSepoint);
        double output = turnAnglePID->stepTurnPID(gyro->get_yaw(), deltaTimeMs);
        // driver.print(2, 2, "%d   ", justResetFlag);
        rightFront->move_velocity(output);
        rightMiddle->move_velocity((output) * (60.0 / 84.0));
        rightBack->move_velocity(output);

        leftFront->move_velocity(-output);
        leftMiddle->move_velocity(-(output) * (60.0 / 84.0));
        leftBack->move_velocity(-output);

        this->prevTime = this->currTime;
        justResetFlag = true;
        break;
    }

    case OPEN_LOOP:
    {

        int fwd_val = (abs(driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) >= 30) ? driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) : 0;
        int turn_val = (abs(driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) >= 15) ? driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) : 0;

        // driver.print(2, 2,"%d   %d  ", fwd_val,turn_val);
        double fwdGrnCart = (pow(fwd_val / 127.0, 3.0)) * 200;
        double turnGrnCart = (pow(turn_val / 127.0, 3.0)) * 200;
        // double fwdGrnCart = (fwd_val/127.0)*600;
        // double turnGrnCart = (turn_val/127.0)*600;

        rightFront->move_velocity(fwdGrnCart - turnGrnCart);
        rightMiddle->move_velocity((fwdGrnCart - turnGrnCart) * (60.0 / 84.0));
        rightBack->move_velocity(fwdGrnCart - turnGrnCart);

        leftFront->move_velocity(fwdGrnCart + turnGrnCart);
        leftMiddle->move_velocity((fwdGrnCart + turnGrnCart) * (60.0 / 84.0));
        leftBack->move_velocity(fwdGrnCart + turnGrnCart);
        break;
    }

    case DEAD:
        stop();

        break;
    case BLANK:
      

        break;
    }
}

Drivetrain::DrivetrainStates Drivetrain::getState()
{
    return mDriveState;
}

void Drivetrain::setState(DrivetrainStates state)
{

    mDriveState = state;
}
void Drivetrain::resetEnc(){
    rightFront->tare_position();
    rightBack->tare_position();

    leftFront->tare_position();
    leftBack->tare_position();

}
bool Drivetrain::isSettledTurned()
{
    bool isSettled;
    isSettled = false;
    if (justResetFlag)
    {
        pros::screen::print(pros::E_TEXT_MEDIUM, 8, "IN RESET");
    // pros::screen::print(pros::E_TEXT_MEDIUM, 6, "IN RESET");
        if (turnAnglePID->isSettledTime())
        {
            isSettled = true;
        }
    }
    

    return isSettled;

    
}

bool Drivetrain::isSettledMove()
{
    bool isSettled;
    isSettled = false;
    if (justResetFlag)
    {
        if (driveDistancePID->isSettledTime())
        {
            isSettled = true;
        }
    }
    

    return isSettled;
}

void Drivetrain::moveDistance(double driveSetpoint, double headingSetpoint)
{
    justResetFlag = false;
    this->distanceSetpoint = driveSetpoint;
    this->headingSetpoint = headingSetpoint;
}

void Drivetrain::turnAngle(double setpoint)
{
    justResetFlag = false;
    this->angleSepoint = setpoint;
}

double Drivetrain::getGyroYaw()
{
    return this->gyro->get_yaw();
}

double Drivetrain::getRightEncInches()
{
    // double rightAvg = (rightFront->get_position()+rightBack->get_position()) / 2;
    return rightFront->get_position()* TICKS_PER_INCH*4.0;
}

double Drivetrain::getLeftEncInches()
{
    // double leftAvg = (leftFront->get_position()+leftBack->get_position()) / 2;
    return leftFront->get_position()*TICKS_PER_INCH*4.0;
}

double Drivetrain::getAvgEncInches()
{

    return (getRightEncInches() + getLeftEncInches()) / 2.0;
}

void Drivetrain::odometryStep(pros::Controller driver)
{

    // ------------------------------- CALCULATIONS -------------------------------

    double rightDriveEncoderRaw = -1.0 * double(this->rightFront->get_position() + this->rightBack->get_position()) / 2.0;
    double leftDriveEncoderRaw = -1.0 * double(this->leftFront->get_position() + this->leftBack->get_position()) / 2.0;

    double deltaRightSideEncoderInches = (rightDriveEncoderRaw - this->righDriveEncoderPrev) * (2.0 * M_PI * WHEEL_RADIUS) / 900.0;
    double deltaLeftSideEncoderInches = (leftDriveEncoderRaw - this->leftDriveEncoderPrev) * (2.0 * M_PI * WHEEL_RADIUS) / 900.0;

    double heading = (this->gyro->get_yaw()) * M_PI / 180.0;

    double deltaHeading = (heading - prevHeading);

    double totalDistance = ((deltaRightSideEncoderInches + (deltaHeading * DRIVE_RADIUS) + deltaLeftSideEncoderInches - (deltaHeading * DRIVE_RADIUS)) / 2);

    double deltaYLocal = totalDistance * cos(deltaHeading);
    double deltaXLocal = totalDistance * sin(deltaHeading);

    double deltaYGlobal = (deltaXLocal * sin(heading)) + (deltaYLocal * cos(heading));
    double deltaXGlobal = (deltaXLocal * cos(heading)) - (deltaYLocal * sin(heading));

    xPoseGlobal += deltaXGlobal;
    yPoseGlobal += deltaYGlobal;

    // this->robotPose->setXComponent(xPoseGlobal);
    // this->robotPose->setYComponent(yPoseGlobal);
    // this->robotPose->setThetaComponent(heading);

    // driver.print(2, 2, "%.1f, %.1f, %.4f", this->deltaXGlobal, xPoseGlobal, headingRaw);

    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "X Global: %f", this->xPoseGlobal);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Y Global: %f", this->yPoseGlobal);
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Heading: %f", heading);

    this->prevHeading = heading;
    this->righDriveEncoderPrev = rightDriveEncoderRaw;
    this->leftDriveEncoderPrev = leftDriveEncoderRaw;
}

void Drivetrain::moveEncoder(double inches, int vel)
{
    resetEnc();
    double inchPerDeg = (M_PI * 4.0) / 360.0;
    double degToMove = inches / inchPerDeg;
    rightFront->move_relative(degToMove, vel);
    rightMiddle->move_relative(degToMove, (vel / 600) * 428);
    rightBack->move_relative(degToMove, vel);

    leftFront->move_relative(degToMove, vel);
    leftMiddle->move_relative(degToMove, (vel / 600) * 428);
    leftBack->move_relative(degToMove, vel);

    while (!((rightFront->get_position() < degToMove + 5) && (rightFront->get_position() > degToMove - 5)))
    {

        pros::delay(2);
    }
}

void Drivetrain::moveSeconds(int seconds, int vel)
{

    rightFront->move_velocity(vel);
    rightMiddle->move_velocity(vel* (60.0 / 84.0));
    rightBack->move_velocity(vel);

    leftFront->move_velocity(vel);
    leftMiddle->move_velocity(vel* (60.0 / 84.0));
    leftBack->move_velocity(vel);

    pros::delay(seconds);
    stop();
}
void Drivetrain::setVel(int vel)
{

    rightFront->move_velocity(vel);
    rightMiddle->move_velocity(vel* (60.0 / 84.0));
    rightBack->move_velocity(vel);

    leftFront->move_velocity(vel);
    leftMiddle->move_velocity(vel* (60.0 / 84.0));
    leftBack->move_velocity(vel);
}
void Drivetrain::turnEncoder(double deg, int vel)
{
    // double inchPerDeg = (M_PI*13.4)/360.0;
    // double turnInches = inchPerDeg*deg;
    // rightFront->move_relative(degToMove, vel);
    // rightMiddle->move_relative(degToMove, (vel/600)*428);
    // rightBack->move_relative(degToMove, vel);

    // leftFront->move_relative(degToMove, vel);
    // leftMiddle->move_relative(degToMove, (vel/600)*428);
    // leftBack->move_relative(degToMove, vel);
}

void Drivetrain::resetGyro()
{
    this->gyro->tare_rotation();
    pros::delay(50);
}

void Drivetrain::turnGyro(double deg, int vel)
{
    // TURN LEFT
    resetGyro();
    if (deg < 0)
    {
        while (gyro->get_yaw() > deg)
        {
            rightFront->move_velocity(vel);
            rightMiddle->move_velocity(vel);
            rightBack->move_velocity(vel);

            leftFront->move_velocity(-vel);
            leftMiddle->move_velocity(-vel);
            leftBack->move_velocity(-vel);
        }
        stop();
    }

    else
    {
        while (gyro->get_yaw() < deg)
        {
            rightFront->move_velocity(-vel);
            rightMiddle->move_velocity(-vel);
            rightBack->move_velocity(-vel);

            leftFront->move_velocity(vel);
            leftMiddle->move_velocity(vel);
            leftBack->move_velocity(vel);
        }
        stop();
    }
}

void Drivetrain::stop()
{
    this->rightFront->move_velocity(0);
    this->rightMiddle->move_velocity(0);
    this->rightBack->move_velocity(0);
    this->leftFront->move_velocity(0);
    this->leftMiddle->move_velocity(0);
    this->leftBack->move_velocity(0);
}