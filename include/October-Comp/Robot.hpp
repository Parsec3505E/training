#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "api.h"
#include "okapi/api.hpp"

using namespace okapi;


class Robot
{

    public:


    IMU* gyro;

    std::shared_ptr<okapi::XDriveModel> xDriveTrain; 

    std::shared_ptr<ChassisController> drive;

        Robot();

        void operatorControl(double x, double y, double turn);


};

#endif