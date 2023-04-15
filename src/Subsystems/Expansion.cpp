#include "Subsystems/Expansion.hpp"

Expansion::Expansion()
{

    expansionPistonR = new pros::ADIDigitalOut('A');
    blockerPiston = new pros::ADIDigitalOut('C');
}

void Expansion::updateExpansion(pros::Controller driver)
{
    switch (mExpansionState)
    {

    case OPERATOR_CONTROL:
        if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            expansionPistonR->set_value(true);
        }
        // if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        // {
        //     blockerPiston->set_value(true);
        // }

        break;
    }
}

Expansion::ExpansionStates Expansion::getState()
{

    return mExpansionState;
}

void Expansion::expand()
{
    blockerPiston->set_value(true);
    // expansionPistonR->set_value(true);
}

void Expansion::setState(ExpansionStates state)
{
    mExpansionState = state;
}
