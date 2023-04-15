#ifndef VECTOR_HPP
#define VECTOR_HPP

class Vector
{

    private:

        // The x and y components of the vector
        double x;
        double y;

    public:

        // Vector constructor
        Vector(double posX, double posY);

        void setXComponent(double posX);
        
        void setYComponent(double posY);

        double getXComponent();

        double getYcomponent();
    

};

#endif


