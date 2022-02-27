/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_TEACHERS_TEACHER_H
#define BNN_DEVICE_3D_TEACHERS_TEACHER_H

#include "config.h"
#include "Ogre.h"
#include "bnn/src/brain/brain.h"

namespace bnn_device_3d::teachers
{

class teacher
{
protected:
    u_word count;
    u_word count_max;
    u_word data = 0;
    bnn::state state_ = bnn::state::stopped;
    std::thread thread_;
    virtual void inner_start() = 0;

public:
    virtual ~teacher();
    teacher();
    u_word get_count();
    u_word get_count_max();
    u_word get_data();
    void start();
    void stop();
};

} // namespace bnn_device_3d::teachers

#endif // BNN_DEVICE_3D_TEACHERS_TEACHER_H
