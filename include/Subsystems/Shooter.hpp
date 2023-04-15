#ifndef SHOOTER_HPP
#define SHOOTER_HPP


#include "api.h"

#include "Utility/PIDController.hpp"


class Shooter
{

    public:

        // Shooter States
        enum ShooterStates{CLOSED_LOOP, CLOSED_LOOP_AUTO, OPERATOR_CONTROL, DISABLED};

    private:

        ShooterStates mShooterState;

        double RPM;

        PIDController* rpmPID;
        double TBHPrevErr;
        double TBHGain;
        double TBHThresh;
        double TBHInputVolt;
        bool TBHCross;

        double motorVelLimit;
        double motorAccLimit;

        int targetVel;
        bool beenSettled;
        int timeSettled;
        int minSettledTime;
        double epsilon;
        bool indFlag;
        int reloadDelay;
        int delayConst;

        // Shooter Motor Declarations
        pros::Motor* shooterPwr;
       
        pros::ADIDigitalOut* shooterInd;

        // Shooter Sensors

        // Shooter Encoder Declarations
        pros::ADIEncoder* flywheelEncoder;

        bool indexerTrigger = false;

        std::uint32_t currTime;
        std::uint32_t prevTime;

    public:

        // Shooter Constructor
        Shooter();

        // Update the state of the Shooter
        void updateShooter(pros::Controller driver);

        void getRPM();

        void setMotorSpeed(int vel);
        void indexAll();
        void indexAll2();
        

        enum ShooterStates getState();

    //private:

        // Set the state of the Shooter
        void setState(enum ShooterStates);
        void setIndexerState(bool state);

        void setTargetRPM(double RPM);
        void TBHController(int targetVel);

        double slewRPM(double request);

        bool isSettled();

        double calcShotRPM(double distance);

        double stepPID(double deltaTime);
};



typedef struct{
Shooter shooter;
} shooter_arg;


#endif