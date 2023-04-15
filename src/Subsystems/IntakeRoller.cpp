#include "Subsystems/IntakeRoller.hpp"
#include "pros/misc.h"

IntakeRoller::IntakeRoller()
{
    // PORT 17 IS BROKEN FOR SOME REASON!!!
    intakeMotor = new pros::Motor(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    colourSensor = new pros::Optical(18);

    rollerPID = new PIDController(0, 0, 0);

    colourFlag = 0;
}

void IntakeRoller::updateIntake(pros::Controller driver)
{

    switch (mIntakeState)
    {


    case OPERATOR_CONTROL:

        if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intakeMotor->move_velocity(-600);
        }

        else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intakeMotor->move_velocity(600);
        }
        else
        {

            intakeMotor->move_velocity(0);
            intakeMotor->move_velocity(0);
        }

        break;

    case COLOUR_AUTO:

        colourSensor->set_led_pwm(50);

        if (colourFlag == 0)
        {
            if (colourSensor->get_hue() < 10.0)
            {
                colourFlag = 1;
            }
            else
            {
                colourFlag = 2;
            }
        }

        if (colourFlag == 1)
        {
            intakeMotor->move_velocity(-400);
            // MOVE UNTIL SEE BLUE
            if (colourSensor->get_hue() > 200.0)
            {
                // CORRCTION
                intakeMotor->tare_position();
                intakeMotor->move_absolute(100, 400);
                if (intakeMotor->get_position() >= 100)
                {
                    intakeMotor->move_velocity(0);
                    colourFlag = 0;
                    setIntakeState(BLANK);
                }
            }
        }

        if (colourFlag == 2)
        {
            intakeMotor->move_velocity(-400);
            // MOVE UNTIL SEE RED
            if (colourSensor->get_hue() < 10.0)
            {
                // CORRCTION
                intakeMotor->tare_position();
                intakeMotor->move_absolute(100, 400);
                if (intakeMotor->get_position() >= 100)
                {
                    intakeMotor->move_velocity(0);
                    colourFlag = 0;
                    setIntakeState(BLANK);
                }
            }
        }

        break;

    case COLOUR_MANUAL:

        break;

    case BLANK:

        break;

    case ON:
        intakeMotor->move_velocity(600);
        break;
    case DEAD:
        intakeMotor->move_velocity(0);
        break;
    }
}

// void IntakeRoller::updateRoller()
// {

// }

// void IntakeRoller::setRollerState(enum RollerStates)
// {

// }

void IntakeRoller::setIntakeState(IntakeStates intakeState)
{
    mIntakeState = intakeState;
}
void IntakeRoller::rollToColourAUTO(int inches)
{
    double inchPerDeg = (M_PI * 4.0) / 360.0;
    double degToMove = inches / inchPerDeg;
    intakeMotor->move_relative(degToMove, -400);
    while (!((intakeMotor->get_position() < degToMove + 5) && (intakeMotor->get_position() > degToMove - 5)))
    {

        pros::delay(2);
    }
}
void IntakeRoller::setVel(int vel)
{
    intakeMotor->move_velocity(vel);
}

void IntakeRoller::rollToColourSEC(int ms)
{

    intakeMotor->move_velocity(-200);
    pros::delay(ms);
    intakeMotor->move_velocity(0);
}
void IntakeRoller::rollToColourDRIVE(bool alliance)
{
    switch (colourFlag)
    {
    case 0:
        intakeMotor->move_velocity(-400);
        // RED
        if (alliance)
        {
            if (colourSensor->get_hue() < 10.0)
            {
                colourFlag = 1;
            }
        }
        // BLUE
        else
        {
            if (colourSensor->get_hue() > 200.0)
            {
                colourFlag = 1;
            }
        }
        break;

    case 1:
        // RED
        if (alliance)
        {
            if (colourSensor->get_hue() > 200.0)
            {
                intakeMotor->tare_position();
                colourFlag = 2;
            }
        }
        // BLUE
        else
        {
            if (colourSensor->get_hue() < 10.0)
            {
                intakeMotor->tare_position();
                colourFlag = 2;
            }
        }

        break;

    case 2:
        intakeMotor->move_velocity(400);
        // RED
        if (alliance)
        {
            if (colourSensor->get_hue() < 10.0)
            {
                intakeMotor->move_velocity(0);
            }
        }
        // BLUE
        else
        {
            if (colourSensor->get_hue() > 200.0)
            {
                intakeMotor->move_velocity(0);
            }
        }

        // intakeMotor->move_absolute(800, 400);

        break;
    }
}

void IntakeRoller::spinSec(int ms, int vel){
    intakeMotor->move_velocity(vel);
    pros::delay(ms);
    intakeMotor->move_velocity(0);
}
void IntakeRoller::rollToColourMANUAL(bool colour)
{
    // TRUE IS RED
    if (colour)
    {
        intakeMotor->move_velocity(-400);
        while (100.0 < colourSensor->get_hue() < 300.0)
        {
        }

        // ROLL UNTIL SEE BLUE
        intakeMotor->move_velocity(-400);
        while (!(100.0 < colourSensor->get_hue() < 300.0))
        {
        }

        // CORRECTION FOR GOING OVER
        intakeMotor->move_velocity(400);
        pros::delay(500);

        intakeMotor->move_velocity(0);
        // intakeMotor->move_relative(20, -200);
    }

    else
    {
        // ROLL UNTIL SEE BLUE
        intakeMotor->move_velocity(-400);
        while (colourSensor->get_hue() < 10.0)
        {
        }

        // ROLL UNTIL SEE RED
        intakeMotor->move_velocity(-400);
        while (colourSensor->get_hue() > 200.0)
        {
        }

        // CORRECTION FOR GOING OVER
        intakeMotor->move_velocity(400);
        pros::delay(500);

        intakeMotor->move_velocity(0);
        // intakeMotor->move_relative(20, -200);
    }
}
double IntakeRoller::readColour()
{

    if (colourSensor->get_hue() < 10.0)
    {
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "RED %f      ", colourSensor->get_hue());
    }
    else if (colourSensor->get_hue() > 200.0)
    {
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "BLUE %f      ", colourSensor->get_hue());
    }

    // pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Hue value: %f      ", colourSensor->get_hue());
    //
    return colourSensor->get_hue();
}

enum IntakeRoller::IntakeStates IntakeRoller::getIntakeState()
{
    return mIntakeState;
}

// enum IntakeRoller::RollerStates IntakeRoller::getRollerState()
// {

// }

bool IntakeRoller::isSettled(double epsilon)
{
}
