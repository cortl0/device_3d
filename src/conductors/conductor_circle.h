#ifndef CONDUCTOR_CIRCLE_H
#define CONDUCTOR_CIRCLE_H

#include "conductor.h"

class conductor_circle : public conductor
{
    const float c_x = 0.f;
    const float c_y = 0.f;
    const float c_z = 0.f;
public:
    virtual ~conductor_circle();
    conductor_circle();
    virtual void step(float& x, float& y, float& z) override;
};

#endif // CONDUCTOR_CIRCLE_H
