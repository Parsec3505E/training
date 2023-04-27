#include "Subsystems/Drivetrain.hpp"
#include "Utility/Pose.hpp"
#include "pros/misc.h"
#include "pros/screen.h"
#include <iostream>

Drivetrain::Drivetrain()
{
    // Construct the Motor objects
    rightFront = new pros::Motor(6, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    rightFront->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    //CONSTRUCT MORE
  
}

Drivetrain::~Drivetrain()
{}

//METHODS
void Drivetrain::updateDrivetrain(pros::Controller &driver)
{

    // Finite State Machine (FSM)

    

    switch (mDriveState)
    {
        
        
        // move forward at a certain speed infinitly until button 'B' is pressed
        //we DO NOT care about what the joystick values are here
    case MOVE_DISTANCE:
    {
        
    }

   
//driver control
    case HUMAN_CONTROL:
    {

        int fwd_val = driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int turn_val = driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

      
      
        //ADD MORE MOTORS
        rightFront->move_velocity(fwd_val);
        
        //IF THE 'A' BUTTON IS PRESSED, SWITCH TO MOVE DISTANCE MODE
        break;
    }

// ======================== IGNORE THESE ====================================================================================
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

void Drivetrain::moveSeconds(int seconds, int vel)
{

    rightFront->move_velocity(vel);
    

    pros::delay(seconds);
    stop();
}

void Drivetrain::stop()
{
    this->rightFront->move_velocity(0);
   
}