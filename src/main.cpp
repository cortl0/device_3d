/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include <exception>
#include <iostream>

#include "application/application.h"

int main()
{
    try
    {
        bnn_device_3d::application::application().run();
    }
    catch(const std::exception& e)
    {
        std::cerr << "Caught std::exception: " << e.what() << std::endl;
    }
    catch(...)
    {
        std::cerr << "Caught unknown error" << std::endl;
    }

    return 0;
}
