#include "utility/Pose.hpp"
#include "Subsystems/Drivetrain.hpp"
#include "Utility/Pose.hpp"
#include "pros/misc.h"
#include "pros/screen.h"
#include <iostream>

Pose::Pose(Vector vector, double posTheta)
{
    this->x = vector.getXComponent();
    this->y = vector.getYcomponent();
    this->theta = posTheta;
}

void Pose::setXComponent(double x)
{
    this->x = x;
}

void Pose::setYComponent(double y)
{
    this->y = y;
}

void Pose::setThetaComponent(double theta)
{

    this->theta = theta;

}

double Pose::getXComponent()
{
    return this->x;
}

double Pose::getYComponent()
{
    return this->y;
}

double Pose::getThetaComponent()
{

    return this->theta;
}

bool Pose::comparePoses(Pose* otherPose)
{
    return this->x == otherPose->getXComponent() && this->y == otherPose->getYComponent() && this->theta == otherPose->getThetaComponent();
}

Pose* Pose::rotatePose(double angle)
{
    double rotatedX = (this->x * cos(angle)) - (this->y * sin(angle));
    double rotatedY = (this->x * sin(angle)) + (this->y * cos(angle));

    Pose* tempPose = new Pose(Vector(rotatedX, rotatedY), this->theta);

    return tempPose;
}

Pose* Pose::DividePose(double divisor)
{
    Pose* newPose = new Pose(Vector((this->x)/divisor, (this->y)/divisor), (this->theta)/divisor);
    return newPose;
}

Pose* Pose::subtractPose(Pose* subtractedPose)
{
    double deltaDistX = this->getXComponent() - subtractedPose->getXComponent();
    double deltaDistY = this->getYComponent() - subtractedPose->getYComponent();
    double deltaDistTheta = this->getThetaComponent() - subtractedPose->getThetaComponent();
    
    Pose* newPose = new Pose(Vector(deltaDistX, deltaDistY), deltaDistTheta);

    return newPose;
}



