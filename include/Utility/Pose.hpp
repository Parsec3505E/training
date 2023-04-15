#ifndef POSE_HPP
#define POSE_HPP

#include "Vector.hpp"

class Pose
{

    private:

        // The x, y (Vector) and theta components of the vector
        double x;
        double y;
        double theta;


    public:

        // Pose constructor
        Pose(Vector vector, double posTheta);

        void setXComponent(double x);
        void setYComponent(double y);
        void setThetaComponent(double theta);

        double getXComponent();
        double getYComponent();
        double getThetaComponent();

        bool comparePoses(Pose* otherPose);

        Pose* rotatePose(double angle);

        Pose* DividePose(double divisor);
        Pose* subtractPose(Pose* subtractedPose);


    


};

#endif