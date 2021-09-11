#ifndef TEACHER_WALKING_H
#define TEACHER_WALKING_H

#include "teacher.h"

class teacher_walking : public teacher
{
    static void function(teacher_walking*);
    void inner_start() override;

public:
    virtual ~teacher_walking();
    teacher_walking();
};

#endif // TEACHER_WALKING_H
