/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include <iostream>
#include <stdexcept>

#include "world_3d.h"

int main()
{
    try
    {
        world_3d app;
        app.initApp();
        app.start();
        app.getRoot()->startRendering();
        app.closeApp();
    }
    catch (const std::exception& e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
    }
    catch (const char* c)
    {
        std::cout << "Caught exception: " << c << std::endl;
    }
    catch (...)
    {
        std::cout << "unknown error" << std::endl;
    }

    return 0;
}
