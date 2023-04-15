#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include "iostream"
#include "api.h"


class PIDController
{

    private:

        // The P, I, and D term of the PID
        double kP;
        double kI;
        double kD;

        double target;
        double error;
        double errorSum;
        double lastError;

        double maxOutput;
        double minOutput;
        
        double prevError;
        double integral;
        double integralWindUp;;
        double deriative;

        double epsilon;
        bool beenSettled;
        int timeSettled;
        int minSettledTime;

    public:

        // PIDController constructor
        PIDController(double kP, double kI, double kD);
        
        // Set the PID term constants
        void setConstants(double kP, double kI, double kD);
        
        // Set the target value to the PID to reach
        void setTarget(double target);

        // Sets the tolerance of the PID
        void setEpsilon(double epsilon);
        
        // Step the PID every iteration of the loop
        double stepPID(double input, double deltaTime);
        double stepTurnPID(double input, double deltaTime);

        // Determines if the PID is settled
        bool isSettled();
        bool isSettledTime();
    

};

#endif