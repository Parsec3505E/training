#include "Subsystems/Shooter.hpp"

Shooter::Shooter()
{
    // PORT 17 IS BROKEN FOR SOME REASON!!!
    shooterPwr = new pros::Motor(1, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

    shooterInd = new pros::ADIDigitalOut('B');

    motorVelLimit = 0;
    motorAccLimit = 0;

    flywheelEncoder = new pros::ADIEncoder('G', 'H');

    rpmPID = new PIDController(0, 0, 0);
    // targetRPM = 0;

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TBH VARS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    TBHPrevErr = 0.0;
    TBHGain = 1.0;
    TBHThresh = 0.0;
    TBHInputVolt = 0.0;
    TBHCross = false;

    targetVel = 0;
    beenSettled = false;
    timeSettled = pros::millis();

    minSettledTime = 200;
    epsilon = 0.8;
    indFlag = false;
    reloadDelay = 0;
    delayConst = 500;
}

void Shooter::updateShooter(pros::Controller driver)
{

    switch (mShooterState)
    {

    case CLOSED_LOOP:
        TBHController(this->targetVel);
        if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            this->TBHGain += 0.1;
        }
        if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            this->TBHGain -= 0.1;
        }
        if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            this->targetVel += 20.0;
        }
        if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            this->targetVel -= 20.0;
        }

        if (indFlag && pros::millis() - reloadDelay > delayConst)
        {
            reloadDelay = pros::millis();
            shooterInd->set_value(true);
            indFlag = false;
        }
        if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !indFlag && pros::millis() - reloadDelay > delayConst)
        {
            reloadDelay = pros::millis();
            shooterInd->set_value(false);
            indFlag = true;
        }
        driver.print(2, 2, "%.1f  %d  %f  ", shooterPwr->get_actual_velocity(), targetVel, TBHGain);
        break;

    case OPERATOR_CONTROL:
        if (indFlag && pros::millis() - reloadDelay > delayConst)
        {
            reloadDelay = pros::millis();
            shooterInd->set_value(true);
            indFlag = false;
        }

        if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        {
            this->targetVel = 340;
            this->epsilon = 0.85;
        }
        else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_A))
        {
            this->targetVel = 310;
            this->epsilon = 0.8;
        }
        else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            this->targetVel = 380;
            this->epsilon = 0.7;
        }
        else if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            // shooterPwr->move_velocity(0);
            // shooterPwr2->move_velocity(0);
            this->targetVel += 10;
            this->epsilon = 0.7;
        }
        if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            shooterPwr->move_velocity(targetVel);
        }
        else if (!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            shooterPwr->move_velocity(0);
        }
        if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !indFlag && pros::millis() - reloadDelay > delayConst)
        {
            reloadDelay = pros::millis();
            shooterInd->set_value(false);
            indFlag = true;
            // shooterInd->set_value(false);
        }

        driver.print(2, 2, "%.1f  %d    ", shooterPwr->get_actual_velocity(), targetVel);

        break;

    case CLOSED_LOOP_AUTO:
        TBHController(this->targetVel);
        driver.print(2, 2, "%.1f  %d  %f  ", shooterPwr->get_actual_velocity(), targetVel, TBHGain);


        break;

    case DISABLED:

        shooterPwr->move_velocity(0);

        break;
    }
}

void Shooter::setState(ShooterStates shooterState)
{
    mShooterState = shooterState;
}

enum Shooter::ShooterStates Shooter::getState()
{
    return mShooterState;
}

void Shooter::setTargetRPM(double RPM)
{
    this->targetVel = RPM;
}

void Shooter::TBHController(int targetVel)
{
    double TBHErr = targetVel - shooterPwr->get_actual_velocity();
    TBHInputVolt += TBHErr * TBHGain;
    if (TBHInputVolt > 12000)
    {
        TBHInputVolt = 12000;
    }
    if (TBHInputVolt < 0)
    {
        TBHInputVolt = 0;
    }

    if (std::signbit(TBHErr) != std::signbit(TBHPrevErr))
    {
        if (!TBHCross)
        {
            TBHThresh = TBHInputVolt * 0.10;
            TBHInputVolt = TBHThresh;
            TBHCross = true;
        }
        else
        {
            TBHInputVolt = 0.5* (TBHThresh + TBHInputVolt);
        }
        TBHThresh = TBHInputVolt;
    }

    TBHPrevErr = TBHErr;
    shooterPwr->move_voltage(TBHInputVolt);
}

void Shooter::setIndexerState(bool state)
{

    shooterInd->set_value(state);
}

void Shooter::getRPM()
{
}

void Shooter::setMotorSpeed(int vel)
{
    shooterPwr->move_velocity(vel);
    targetVel = vel;
}

void Shooter::indexAll()
{

    shooterInd->set_value(false);
    pros::delay(500);
    shooterInd->set_value(true);
    pros::delay(500);
    setMotorSpeed(445);
    pros::delay(3000);
    shooterInd->set_value(false);
    pros::delay(500);
    shooterInd->set_value(true);
    pros::delay(500);
}
void Shooter::indexAll2()
{

    shooterInd->set_value(false);
    pros::delay(500);
    shooterInd->set_value(true);
    //was 1000 with third commented out
    pros::delay(800);

    shooterInd->set_value(false);
    pros::delay(500);
    shooterInd->set_value(true);
    pros::delay(800);

    shooterInd->set_value(false);
    pros::delay(500);
    shooterInd->set_value(true);
    pros::delay(300);
}
double Shooter::slewRPM(double request)
{
}

bool Shooter::isSettled()
{
    // pros::screen::print(pros::E_TEXT_MEDIUM, 7, "SHOOTER VEL    %f", shooterPwr->get_actual_velocity());
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, "TIME SETTLED: %f", timeSettled);
    if (shooterPwr->get_actual_velocity() >= targetVel * epsilon)
    {
        if (!beenSettled)
        {
            beenSettled = true;
            timeSettled = pros::millis();
        }
        return (pros::millis() - timeSettled) > minSettledTime;
    }
    else
    {
        beenSettled = false;
        return false;
    }
}

double Shooter::calcShotRPM(double distance)
{
}