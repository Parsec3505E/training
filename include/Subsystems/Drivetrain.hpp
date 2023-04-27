#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"

#include <map>
#include "iostream"

#include <ctime>

#include "utility/Pose.hpp"
#include "utility/PosePID.hpp"

#include <chrono>

    
class Drivetrain
{

    public:
        //Drivetrain States
        enum DrivetrainStates{HUMAN_CONTROL, MOVE_DISTANCE, DEAD, BLANK};

    

        DrivetrainStates mDriveState;

        

        // Drivetrain Motor Declarations
        pros::Motor* rightFront;
        

       

       

    public:

        // Drivetrain Constructor
        Drivetrain();

        // Update the state of the Drivetrain
        void updateDrivetrain(pros::Controller &driver);

        // Set the state of the Drivetrain
        void setState(enum DrivetrainStates);

        // Get the state of the Drivetrain
        enum DrivetrainStates getState();
        
        void moveSeconds(int seconds, int vel);
       
        void stop();


};

typedef struct{
Drivetrain drive;
} drive_arg;

#endif