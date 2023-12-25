/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_TEACHERS_TEACHER_WALKING_H
#define BNN_DEVICE_3D_TEACHERS_TEACHER_WALKING_H

#include "teacher.h"

namespace bnn_device_3d::teachers
{

class teacher_walking : public teacher
{
    static void function(teacher_walking*);
    void inner_start() override;

public:
    virtual ~teacher_walking();
    teacher_walking();
};

} // namespace bnn_device_3d::teachers

#endif // BNN_DEVICE_3D_TEACHERS_TEACHER_WALKING_H
