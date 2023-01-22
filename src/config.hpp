/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef DEVICE_3D_CONFIG_HPP
#define DEVICE_3D_CONFIG_HPP

#define device_3d_GRAVITY 9.81f // [m/s^2]

#define device_3d_SCALE 0.01f
#define device_3d_MASS_SCALE 1.0f

#define show_debug_data
//#define creature_legs_knees_is_blocked
//#define learning_creature

#define QUANTITY_OF_JOINTS_IN_LEG 2
#define COLOR_BLACK 0x222222ff
#define COLOR_DARK 0x555555ff
#define COLOR_LIGHT 0xbbbbbbff
#define COLOR_MEDIUM 0x777777ff

#define device_3d_LITTLE_TIME 1000

namespace bnn_device_3d
{

struct config
{
    enum keyboard_keys
    {
        keyboard_key_c = 0x63,
        keyboard_key_x = 0x78,
        keyboard_key_z = 0x7A,
        keyboard_key_v = 0x76,
        keyboard_key_a = 0x61,
        keyboard_key_d = 0x64,
        keyboard_key_w = 0x77,
        keyboard_key_r = 0x72,
        keyboard_key_s = 0x73
    };
};

struct keys_states
{
    bool key_a = false;
    bool key_d = false;
    bool key_w = false;
    bool key_s = false;
    bool key_left = false;
    bool key_right = false;
    bool key_up = false;
    bool key_down = false;
};

} // namespace bnn_device_3d

#endif // DEVICE_3D_CONFIG_HPP
