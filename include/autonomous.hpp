#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include "main.h"


void odomAuton();
void odomAutonAWP();
void PIDRollerLow();
void PIDAutonFarRoller();
void PIDAutonTwoRoller();
void PIDAutonFarRollDisk();

void HCRoller();
void HCRollerDisc();
void HCRollerTwo();
void HCRollerTwoDisc();

void endAllTasks();



#endif