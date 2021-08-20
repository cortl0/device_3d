#ifndef TEACHER_BASE_H
#define TEACHER_BASE_H

#include "config.h"
#include "Ogre.h"
#include "bnn/src/brain/brain.h"

class teacher_base
{
protected:
    _word count;
    _word count_max;
    _word data = 0;
    bnn::state state_ = bnn::state::stopped;
    std::thread thread_;
    virtual void inner_start() = 0;

public:
    virtual ~teacher_base();
    teacher_base();
    _word get_count();
    _word get_count_max();
    _word get_data();
    void start();
    void stop();
};

#endif // TEACHER_BASE_H
