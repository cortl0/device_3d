/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "creature.h"

namespace bnn_device_3d::creatures
{

creature::~creature()
{

}

creature::creature()
{

}

void creature::start()
{
    bnn_->start();

#ifdef learning_creature
    teacher_->start();
#endif
}

void creature::stop()
{
    bnn_->stop();

#ifdef learning_creature
    teacher_->stop();
#endif
}

} // namespace bnn_device_3d::creatures
