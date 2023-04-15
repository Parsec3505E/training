#include "utility/Vector.hpp"

Vector::Vector(double posX, double posY)
{
    this->x = posX;
    this->y = posY;
}

void Vector::setXComponent(double posX)
{
    this->x = posX;
}

void Vector::setYComponent(double posY)
{
    this->y = posY;
}

double Vector::getXComponent()
{
    return this->x;
}

double Vector::getYcomponent()
{
    return this->y;
}