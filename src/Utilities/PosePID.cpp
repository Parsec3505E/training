#include "utility/PosePID.hpp"

PosePID::PosePID()
{
    this->xPID = new PIDController(-2.25, 0.0, 0.0);
    this->yPID = new PIDController(-2.25, 0.0, 0.0);
    this->thetaPID = new PIDController(-1.5, 0.0, 0.1);

    this->xPID->setEpsilon(0.75);
    this->yPID->setEpsilon(0.75);
    this->thetaPID->setEpsilon(0.04);

    this->targetPose = new Pose(Vector(0.0, 0.0), 0.0);
    this->outputPose = new Pose(Vector(0.0, 0.0), 0.0);

    beenSettled = false;
    timeSettled = pros::millis();

    minSettledTime = 100;

}

void PosePID::setXConstants(double kP, double kI, double kD)
{
    xPID->setConstants(kP, kI, kD);
}

void PosePID::setYConstants(double kP, double kI, double kD)
{
    yPID->setConstants(kP, kI, kD);
}

void PosePID::setThetaConstants(double kP, double kI, double kD)
{
    thetaPID->setConstants(kP, kI, kD);
}

void PosePID::setXEpsilon(double epsilon)
{
    this->xPID->setEpsilon(epsilon);
}

void PosePID::setYEpsilon(double epsilon)
{
    this->yPID->setEpsilon(epsilon);
}

void PosePID::setThetaEpsilon(double epsilon)
{
   this->thetaPID->setEpsilon(epsilon); 
}

void PosePID::setTarget(Pose* target)
{

    this->targetPose->setXComponent(target->getXComponent());
    this->targetPose->setYComponent(target->getYComponent());
    this->targetPose->setThetaComponent(target->getThetaComponent());

}

Pose* PosePID::getTarget()
{
    return this->targetPose;
}


Pose* PosePID::stepPID(Pose* input, double deltaTime)
{
    this->xPID->setTarget(this->targetPose->getXComponent());
    double xOutput = this->xPID->stepPID(input->getXComponent(), deltaTime);
    this->outputPose->setXComponent(xOutput);

    this->yPID->setTarget(this->targetPose->getYComponent());
    double yOutput = this->yPID->stepPID(input->getYComponent(), deltaTime);
    this->outputPose->setYComponent(yOutput);

    this->thetaPID->setTarget(this->targetPose->getThetaComponent());
    double thetaOutput = this->thetaPID->stepPID(fmod(input->getThetaComponent(), M_PI*2), deltaTime);
    this->outputPose->setThetaComponent(thetaOutput);

    //pros::screen::print(pros::E_TEXT_MEDIUM, 9, "x: %f    ", xOutput);
    // pros::screen::print(pros::E_TEXT_MEDIUM, 6, "y: %f", yOutput);
    //pros::screen::print(pros::E_TEXT_MEDIUM, 9, "theta: %f", thetaOutput);

    return outputPose;
}

bool PosePID::isSettled()
{
    if(this->thetaPID->isSettled() && this->xPID->isSettled() && this->yPID->isSettled()){
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
