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
        enum DrivetrainStates{OPEN_LOOP, MOVE_DISTANCE, TURN_ANGLE, DEAD, BLANK};

    //private:

        const double DRIVE_RADIUS = 6.453;
        const double WHEEL_RADIUS = 2.0;

        DrivetrainStates mDriveState;

        PIDController* driveDistancePID;
        PIDController* turnAnglePID;
        PIDController* driveTurnPID;
        bool justResetFlag;

        const double DRIVE_P = 8.0;
        const double DRIVE_I = 0.0;
        const double DRIVE_D = 0.0;

        // oiginally 0.5 for kP
        const double DRIVE_TURN_P = 0.1;
        const double DRIVE_TURN_I = 0.0;
        const double DRIVE_TURN_D = 0.0;

        const double TURN_P = 2.0;
        const double TURN_I = 0.05;
        const double TURN_D = 2.0;  

        const double DRIVE_EPSILON = 0.5;
        const double TURN_EPSILON = 0.75;
        const double DRIVE_TURN_EPSILON = 5.0;

        const double VEL_TO_VOLT_RATIO = 12000/200;

        const double TICKS_PER_REV = 900.0;

        const double TICKS_PER_INCH = (2.0 * M_PI * 2.0)/TICKS_PER_REV;



        // Drivetrain Motor Declarations
        pros::Motor* rightFront;
        pros::Motor* leftFront;
        pros::Motor* rightMiddle;
        pros::Motor* leftMiddle;
        pros::Motor* rightBack;
        pros::Motor* leftBack;

        // Drivetrain Sensors

        // Odometry Encoder Declarations
        pros::ADIEncoder* forwardEncoder;
        pros::ADIEncoder* sideEncoder;

        // Drivetrain Gyro Declaration
        pros::IMU* gyro;


        std::uint32_t currTime;
        std::uint32_t prevTime;

        // ------------------------------- ODOMETRY VARS ------------------------------- 

        const double WHEEL_DIAMETER = 2.75;

        //Distances of tracking wheels from tracking center (INCHES)
        const double FORWARD_ENCODER_TRACK_RADIUS = 5.5225;
        //Might need to be negative
        const double SIDE_ENCODER_TRACK_RADIUS = 5.5225;

        double righDriveEncoderPrev = 0.0;
        double leftDriveEncoderPrev = 0.0;

        double prevHeading = 0.0;

        double xPoseGlobal = 0.0;
        double yPoseGlobal = 0.0;

        double distanceSetpoint;
        double angleSepoint;
        double headingSetpoint;


    public:

        // Drivetrain Constructor
        Drivetrain();

        // Update the state of the Drivetrain
        void updateDrivetrain(pros::Controller &driver);

        // Set the state of the Drivetrain
        void setState(enum DrivetrainStates);

        // Get the state of the Drivetrain
        enum DrivetrainStates getState();
        double joystickControl(double rawSpeed);
        void resetGyro();
        void resetEnc();
        void moveEncoder(double inches, int vel);
        void turnEncoder(double deg, int vel);

        void moveSeconds(int seconds, int vel);
        void setVel(int vel);
        void turnGyro(double deg, int vel);

        Pose* getRobotPose();

        // void setRobotPose(Pose pose);

        // bool isSettled();

        ~Drivetrain();
        bool isSettledTurned();
        bool isSettledMove();
        // Ses the distance setpoint
        void moveDistance(double driveSetpoint, double headingSetpoint);
       
        // Set the turn setpoint
        void turnAngle(double setpoint);
        void turnToPoint(double x, double y);

        double getGyroYaw();
        double getRightEncInches();
        double getLeftEncInches();
        double getAvgEncInches();



        // void moveRobot(Pose* velocityPose);

    private:

        

        // void setTargetPose(Pose targetPose);

        // double getAcceleration(double prevRPM, double requestedRPM);

        // std::map<std::string, double> slewPose(std::map<std::string, double> requestedRPM);

        void odometryStep(pros::Controller driver);


        // Pose calcPoseToGoal();

       

        void stop();


};

typedef struct{
Drivetrain drive;
} drive_arg;

#endif