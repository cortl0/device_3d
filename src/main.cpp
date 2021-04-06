#include <iostream>

#include "world_3d.h"

using namespace std;

int main()
{
    world_3d app;
    app.initApp();
    app.cycle();
    app.getRoot()->startRendering();
    app.closeApp();

    return 0;
}
