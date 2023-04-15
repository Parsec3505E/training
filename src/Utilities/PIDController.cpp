#include "utility/PIDController.hpp"

PIDController::PIDController(double kP, double kI, double kD)
{

    this->kP = kP;
    this->kI = kI;
    this->kD = kD;

    this->target = 0.0;
    this->error = 0.0;
    this->prevError = 0.0;
    this->errorSum = 0.0;
    this->lastError = 0.0;

    this->integralWindUp = 10.0;

    this->maxOutput = 0.0;
    this->minOutput = 0.0;

    beenSettled = false;
    timeSettled = pros::millis();

    minSettledTime = 100;
}

void PIDController::setConstants(double kP, double kI, double kD)
{

    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

void PIDController::setTarget(double target)
{
    this->target = target;
}

void PIDController::setEpsilon(double epsilon)
{
    this->epsilon = epsilon;
}

double PIDController::stepPID(double input, double deltaTime)
{

    this->error = this->target - input;

    if (fabs(this->error) > this->integralWindUp)
    {
        this->integral = 0.0;
    }
    else
    {
        this->integral = (this->integral + this->error); // *
    }

    this->deriative = ((this->error - this->prevError)/deltaTime); // / 

    //pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Deriv: %f", this->deriative);
    double output = (this->kP * this->error) + (this->kI * this->integral) + (this->kD * this->deriative);

    
    // pros::screen::print(pros::E_TEXT_MEDIUM, 11, "input: %.4f", input);
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, "error: %.4f", this->error);
    // pros::screen::print(pros::E_TEXT_MEDIUM, 7, "target: %.4f", this->target);

    this->prevError = this->error;

    return output;
}
double PIDController::stepTurnPID(double input, double deltaTime)
{

    this->error =  input - this->target;

    if (fabs(this->error) > this->integralWindUp)
    {
        this->integral = 0.0;
    }
    else
    {
        this->integral = (this->integral + this->error); // *
    }

    this->deriative = ((this->error - this->prevError)/deltaTime); // / 

    //pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Deriv: %f", this->deriative);
    double output = (this->kP * this->error) + (this->kI * this->integral) + (this->kD * this->deriative);

    
    // pros::screen::print(pros::E_TEXT_MEDIUM, 11, "input: %.4f", input);
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, "error: %.4f", this->error);
    // pros::screen::print(pros::E_TEXT_MEDIUM, 7, "target: %.4f", this->target);

    this->prevError = this->error;

    return output;
}

bool PIDController::isSettled()
{
    return (fabs(this->error) <= this->epsilon);
    // pros::screen::print(pros::E_TEXT_MEDIUM, 6, "settle");
}
bool PIDController::isSettledTime()
{
    if(isSettled()){
        if(!beenSettled){
            beenSettled = true;
            timeSettled = pros::millis();
        }
        return (pros::millis()-timeSettled)>minSettledTime;
    }
    else{
        beenSettled = false;
        return false;
 
    }
}