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

#include "lib/bnn/src/bnn/config.h"

#include "config.hpp"

namespace bnn_device_3d::sensors
{

time::time(u_word input_offset, u_word input_length)
    : sensor("time", input_offset, input_length)
{

}

void time::set_inputs(bnn::architecture& bnn, u_word& input_offset, std::string& debug_str, bool verbose)
{
    namespace sch = std::chrono;
    typedef sch::time_point<sch::system_clock, sch::microseconds> m_time_point;
    size_t mask = 18;

    unsigned long long int t = sch::time_point_cast<m_time_point::duration>
            (sch::system_clock::time_point(sch::system_clock::now())).time_since_epoch().count();

    for(size_t i = mask; i < sizeof(t) * QUANTITY_OF_BITS_IN_BYTE; i++)
    {
        bnn.set_input(input_offset++, (t >> i) & 1);

#ifdef show_debug_data
        debug_str += std::to_string((t >> i) & 1);
#endif
    }

    for(size_t i = 0; i < mask; i++)
    {
        bnn.set_input(input_offset++, 0);

#ifdef show_debug_data
        debug_str += std::to_string(0);
#endif
    }
}

} // namespace bnn_device_3d::sensors
