/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@yandex.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "teacher_walking.h"

namespace bnn_device_3d::teachers
{

teacher_walking::~teacher_walking()
{

}

teacher_walking::teacher_walking()
{
    count_max = 1000;
}

void teacher_walking::function(teacher_walking* t)
{
    u_word data;

    t->state_ = bnn_state::started;

    int i = 0;
    while (bnn_state::started == t->state_)
    {
        data = 1;
--t->count;
        switch (i)
        {
//        case 0:
//            data = 0b00000000;
//            std::cout << "count is " << std::to_string(t->count) << std::endl;
//            --t->count;
//            break;
        case 1:
            data = 0b00000000;
            break;
        case 2:
            data = 0b01010101;
            break;
//        case 3:
//            data = 0b00010101;
//            break;
//        case 4:
//            data = 0b01010101;
//            break;
        }

        t->data = data;

        //std::cout << data << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (!t->count)
            t->state_ = bnn_state::stopped;

        ++i;

        if(i > 2)
            i = 1;
    }
}

void teacher_walking::inner_start()
{
    count = count_max;

    thread_ = std::thread(function, this);
}

} // namespace bnn_device_3d::teachers
