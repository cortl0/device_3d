#ifndef TEACHER_WALKING_H
#define TEACHER_WALKING_H

#include "teacher_base.h"

class teacher_walking : public teacher_base
{
    static void function(teacher_walking*);
    void inner_start() override;

public:
    virtual ~teacher_walking();
    teacher_walking();
};

#endif // TEACHER_WALKING_H
