/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_CREATURES_BIKE_BIKE_H
#define BNN_DEVICE_3D_CREATURES_BIKE_BIKE_H

#include <vector>

#include "config.hpp"
#include "creatures/creature.h"
#include "physical_objects/sphere.h"
#include "sensors/sensor.h"
#include "sensors/time.h"
#include "sensors/video.h"

namespace bnn_device_3d::creatures::bike
{

struct force
{
    int left;
    int right;
    int forward;
    int backward;
};

struct sense
{
    int left{0};
    int right{0};
    int front{0};
    int rear{0};
    void set_default();
};

class bike : public creature
{
public:
    struct effector
    {
        u_word bits_quantity;
        float force_coefficient;
        float max_position;
    };

    struct settings
    {
        static constexpr float body_length = 100 * device_3d_SCALE;
        static constexpr float body_width = 50 * device_3d_SCALE;
        static constexpr float body_height = 100 * device_3d_SCALE;
        static constexpr float whell_radius = body_height / 2;
        static constexpr float body_mass = body_length * body_width * body_height * device_3d_MASS_SCALE;

        static constexpr float wheel_mass = 4.0 / 3.0 * M_PI *
            whell_radius * whell_radius * whell_radius * device_3d_MASS_SCALE;

        static constexpr float clearance = body_height;
        static constexpr float level = clearance + body_height / 2;
        static constexpr float front_whell_direction_angle = 0;//M_PI / 4;
        static constexpr u_word i_feel_my_velosity_quantity_bits = 0 * coordinates_count;
        static constexpr u_word i_feel_my_orientation_quantity_bits = 0 * coordinates_count;
        static constexpr u_word i_feel_my_vestibular_quantity_bits = 64 * coordinates_count;
        static constexpr u_word i_feel_time_quantity_bits = 0;

        static constexpr effector front_wheel_torque_left
        {
            .bits_quantity = 0,//2,//2 * QUANTITY_OF_BITS_IN_BYTE,
            .force_coefficient = wheel_mass * 1.f,
            .max_position = M_PI / 4
        };
        static constexpr effector front_wheel_torque_right
        {
            .bits_quantity = 0,//2,//2 * QUANTITY_OF_BITS_IN_BYTE,
            .force_coefficient = wheel_mass * 1.f,
            .max_position = M_PI / 4
        };
        static constexpr effector rear_wheel_throttle_forward
        {
            .bits_quantity = 4,//4 * QUANTITY_OF_BITS_IN_BYTE,
            .force_coefficient = wheel_mass * 5.f,
            .max_position = 1.0f
        };
        static constexpr effector rear_wheel_throttle_backward
        {
            .bits_quantity = 4,//4 * QUANTITY_OF_BITS_IN_BYTE,
            .force_coefficient = wheel_mass * 5.f,
            .max_position = 1.0f
        };
    };

    bnn_device_3d::physical_objects::cube body;
    bnn_device_3d::physical_objects::cube body_sign;
    std::vector<std::unique_ptr<bnn_device_3d::sensors::sensor>> sensors_;

    virtual ~bike();
    bike(
            Ogre::RenderWindow*,
            Ogre::SceneManager*,
            dWorldID,
            const bnn_device_3d::application::config::device_3d::bnn&
            );

    bnn_device_3d::physical_objects::figure& get_body() override;
    std::vector<bnn_device_3d::physical_objects::figure*> get_figures() override;
    Ogre::Vector3 get_camera_place() override;
    dReal get_level() override;
    void set_position(dReal x, dReal y, dReal z) override;
    void step(std::string& debug_str, bool& verbose) override;

    sense sense_;
    force force_;

private:
    physical_objects::sphere front_wheel;
    physical_objects::sphere rear_wheel;
    dJointID rear_hinge_joint_id;
    dJointID front_hinge2_joint_id;
    dSpaceID space;
    std::list<dGeomID> colliding_geoms;
    std::unique_ptr<bnn_device_3d::data_processing_methods::data_processing_method> data_processing_method_;
};

} // namespace bnn_device_3d::creatures::bike

#endif // BNN_DEVICE_3D_CREATURES_BIKE_BIKE_H
