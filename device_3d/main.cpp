/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include <exception>

#include <submodules/logger/src/helpers/log.h>

#include "application/application.h"

int main()
{
    cortl_logger_instance.set_level(cortl::logger::logger::level::debug);

    try
    {
        bnn_device_3d::application::application().run();
    }
    catch(int i)
    {
        log_error("Caught int [%d]", i);
    }
    catch(const std::exception& e)
    {
        log_error("Caught std::exception [%s]", e.what());
    }
    catch(...)
    {
        log_error("Caught unknown error");
    }

    return 0;
}
