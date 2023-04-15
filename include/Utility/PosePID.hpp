#ifndef POSEPID_HPP
#define POSEPID_HPP

#include "PIDController.hpp"
#include "Pose.hpp"

class PosePID
{

    private:

        // Three PIDs for three seperate components
        PIDController* xPID;
        PIDController* yPID;
        PIDController* thetaPID;

        Pose* targetPose;
        Pose* outputPose;

        bool beenSettled;
        int timeSettled;
        int minSettledTime;

    public:

        // PosePID constructor
        PosePID();

        void setXConstants(double kP, double kI, double kD);
        void setYConstants(double kP, double kI, double kD);
        void setThetaConstants(double kP, double kI, double kD);

        void setXEpsilon(double epsilon);
        void setYEpsilon(double epsilon);
        void setThetaEpsilon(double epsilon);
        


        
        // Set the target value to the PID to reach
        void setTarget(Pose* target);

        Pose* getTarget();
        
        // Step the PID every iteration of the loop
        Pose* stepPID(Pose* input, double deltaTime);

        bool isSettled();
    

};

#endif