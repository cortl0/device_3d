#include "teacher.h"

teacher::~teacher()
{
    if(bnn::state::started == state_)
        stop();
}

teacher::teacher()
{

}

_word teacher::get_count()
{
    return count;
}

_word teacher::get_count_max()
{
    return count_max;
}

_word teacher::get_data()
{
    return data;
}

void teacher::start()
{
    if(bnn::state::stopped != state_)
        throw std::runtime_error("busines logic error in teacher_base::start()");

    count = count_max;

    state_ = bnn::state::start;

    inner_start();

    do
    {
        sleep(1);
    }
    while (bnn::state::started != state_);
}

void teacher::stop()
{
    if(bnn::state::started != state_)
        throw std::runtime_error("busines logic error in teacher_base::stop()");

    state_ = bnn::state::stop;

    do
    {
        sleep(1);
    }
    while (bnn::state::stopped != state_);
}
