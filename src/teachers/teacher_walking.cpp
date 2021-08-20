#include "teacher_walking.h"

teacher_walking::~teacher_walking()
{

}

teacher_walking::teacher_walking()
{
    count_max = 100;
}

void teacher_walking::function(teacher_walking* t)
{
    _word data;

    t->state_ = bnn::state::started;

    int i = 0;
    while (bnn::state::started == t->state_)
    {
        data = 0;

        switch (i)
        {
        case 0:
            data = 0b00000000;
            std::cout << "count is " << std::to_string(t->count) << std::endl;
            t->count--;
            break;
        case 1:
            data = 0b00000001;
            break;
        case 2:
            data = 0b00000101;
            break;
        case 3:
            data = 0b00010101;
            break;
        case 4:
            data = 0b01010101;
            break;
        }

        t->data = data;

        sleep(2);

        if (t->count == 0)
            t->state_ = bnn::state::stopped;

        i++;

        if(i > 4)
            i = 0;
    }
}

void teacher_walking::inner_start()
{
    count = 100;

    thread_ = std::thread(function, this);
}
