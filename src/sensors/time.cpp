/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "time.h"

#include <thread>

#include "config.hpp"

namespace bnn_device_3d::sensors
{

time::time()
{

}

int time::get_data_size()
{
    return 64;
}

void time::set_inputs(bnn::architecture& brain_, u_word& count_input, std::string& debug_str, bool verbose)
{
    namespace sch = std::chrono;
    typedef sch::time_point<sch::system_clock, sch::microseconds> m_time_point;
    size_t mask = 18;

    unsigned long long int t = sch::time_point_cast<m_time_point::duration>
            (sch::system_clock::time_point(sch::system_clock::now())).time_since_epoch().count();

    for(size_t i = mask; i < sizeof(t) * QUANTITY_OF_BITS_IN_BYTE; i++)
    {
        brain_.set_input(count_input++, (t >> i) & 1);

#ifdef show_debug_data
        debug_str += std::to_string((t >> i) & 1);
#endif
    }

    for(size_t i = 0; i < mask; i++)
    {
        brain_.set_input(count_input++, 0);

#ifdef show_debug_data
        debug_str += std::to_string(0);
#endif
    }
}

} // namespace bnn_device_3d::sensors
