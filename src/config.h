/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef DEVICE_3D_CONFIG_H
#define DEVICE_3D_CONFIG_H

#define device_3d_GRAVITY 9.81f // [m/s^2]

#define device_3d_SCALE 0.01f
#define device_3d_MASS_SCALE 1.0f

struct config
{
    enum keyboard_keys
    {
        keyboard_key_c = 0x63,
        keyboard_key_x = 0x78,
        keyboard_key_z = 0x7A
    };

public:
    config(){}
};

#endif // DEVICE_3D_CONFIG_H
