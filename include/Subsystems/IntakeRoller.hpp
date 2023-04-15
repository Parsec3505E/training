#ifndef INTAKE_HPP
#define INTAKE_HPP


#include "api.h"
#include "Utility/PIDController.hpp"


class IntakeRoller
{

    public:

        // IntakeRoller States
        enum IntakeStates{OPERATOR_CONTROL, COLOUR_AUTO, COLOUR_MANUAL, BLANK, ON, DEAD};
        // enum RollerStates{};

        PIDController* rollerPID;

        // IntakeRoller Motor Declarations
        pros::Motor* intakeMotor;


        // Roller Sensors

        // Roller Colour Sensor Declarations
        pros::Optical* colourSensor;

        int colourFlag;

        IntakeStates mIntakeState;


        // IntakeRoller Constructor
        IntakeRoller();

        // Update the state of the Intake
        void updateIntake(pros::Controller driver);

        //Hardcoded Methods
        void spinSec(int ms, int vel);
        void setVel(int vel);


        //Optical Methods
        void rollToColourAUTO(int inches);
        void rollToColourDRIVE(bool alliance);
        void rollToColourMANUAL(bool colour);
        void rollToColourSEC(int ms);

        double readColour();

        // Update the state of the Roller
        // void updateRoller();

        enum IntakeStates getIntakeState();
        // enum RollerStates getRollerState();

        // Set the state of the intake
        void setIntakeState(enum IntakeStates intakeState);

        // Set the state of the roller
        // void setRollerState(enum RollerStates);

        bool isSettled(double epsilon);
};

typedef struct{
IntakeRoller intake;
} intake_arg;

#endif