#ifndef CONDUCTOR_H
#define CONDUCTOR_H

#include<cmath>

class conductor
{
public:
    virtual ~conductor();
    conductor();

    /**
     * @param
     * before calling this method
     * x, y, z - coordinates of the controlling body
     * after calling this method
     * x, y, z - vector to target point
    */
    virtual void step(float& x, float& y, float& z) = 0;
};

#endif // CONDUCTOR_H
