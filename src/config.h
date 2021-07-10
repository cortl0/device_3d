/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef CONFIG_H
#define CONFIG_H

#define gravity 9.81f // [m/s^2]

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

#endif // CONFIG_H
