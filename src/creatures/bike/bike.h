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

#include "config.hpp"
#include "../creature.h"
#include "physical_objects/sphere.h"
#include "sensors/gyroscope.h"
#include "sensors/time.h"
#include "sensors/velocity.h"
#include "sensors/video.h"

namespace bnn_device_3d::creatures::bike
{

class bike : public creature
{
    static constexpr float body_length = 100 * device_3d_SCALE;
    static constexpr float body_width = 50 * device_3d_SCALE;
    static constexpr float body_height = 100 * device_3d_SCALE;
    static constexpr float body_mass = body_length * body_width * body_height * device_3d_MASS_SCALE;
    static constexpr dReal clearance = body_height;
    static constexpr float whell_radius = body_height / 2;
    static constexpr float front_whell_direction_angle = M_PI / 6;
    static constexpr float front_whell_direction_quantity_bits_left = 4 * QUANTITY_OF_BITS_IN_BYTE;
    static constexpr float front_whell_direction_quantity_bits_right = 4 * QUANTITY_OF_BITS_IN_BYTE;
    static constexpr float front_whell_trotle_quantity_bits = 4 * QUANTITY_OF_BITS_IN_BYTE;
    static constexpr float rear_whell_trotle_quantity_bits = 4 * QUANTITY_OF_BITS_IN_BYTE;

public:
    static constexpr dReal level = clearance + body_height / 2;
    bnn_device_3d::physical_objects::cube body;
    bnn_device_3d::physical_objects::cube body_sign;
    sensors::gyroscope gyroscope_;
    sensors::time time_;
    sensors::velocity speedometer_;

    virtual ~bike();
    bike(Ogre::RenderWindow*, Ogre::SceneManager*, dWorldID);
    bnn_device_3d::physical_objects::figure& get_body() override;
    std::vector<bnn_device_3d::physical_objects::figure*> get_figures() override;
    Ogre::Vector3 get_camera_place() override;
    dReal get_height() override;
    void set_position(dReal x, dReal y, dReal z) override;
    void step(std::string& debug_str, bool& verbose) override;

    double front_trotle = 0;
    double rear_trotle = 0;
    double front_direction;

private:
    float wheel_size_x = body_length;
    float wheel_size_y = clearance;
    physical_objects::sphere front_wheel;
    physical_objects::sphere rear_wheel;
    dJointID rear_hinge_joint_id;
    dJointID front_hinge2_joint_id;
    u_word quantity_of_neurons_in_power_of_two = 18;
    u_word threads_count_in_power_of_two = 2;
    dSpaceID space;
    std::list<dGeomID> colliding_geoms;
    std::unique_ptr<bnn_device_3d::data_processing_methods::data_processing_method> data_processing_method_;
};

} // namespace bnn_device_3d::creatures::bike

#endif // BNN_DEVICE_3D_CREATURES_BIKE_BIKE_H
