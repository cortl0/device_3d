/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "bike.h"

namespace dpm = bnn_device_3d::data_processing_methods;
namespace pho = bnn_device_3d::physical_objects;
namespace sens = bnn_device_3d::sensors;

namespace bnn_device_3d::creatures::bike
{

bike::~bike()
{

}

bike::bike(Ogre::RenderWindow* render_window, Ogre::SceneManager* scnMgr, dWorldID world)
{
    data_processing_method_.reset(new dpm::data_processing_method_linearly());

    auto make_fixed_joint = [&](dGeomID g)
    {
        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach(j, dGeomGetBody(body.geom), dGeomGetBody(g));
        dJointSetFixed(j);
    };

    space = dSimpleSpaceCreate(nullptr);

    {
        body = pho::cube("body", scnMgr, world, space, body_mass, body_width, body_height, body_length);
        body.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));
        colliding_geoms.push_back(body.geom);
    }

    auto* p = dBodyGetPosition(body.body);

    {
        body_sign = pho::cube("body_sign", scnMgr, world, space, body_mass * 0.001, body_width * 1.05, body_length * 0.95, body_length * 0.95);
        body_sign.set_material(body_sign.create_material_body_sign(256));
        make_fixed_joint(body_sign.geom);
    }

    float whell_r = body_width;
    float wheel_mass = 4.0 / 3.0 * M_PI * whell_r * whell_r * whell_r * device_3d_MASS_SCALE;

    float wheel_pos_z_coefficient = 0.65;

    {
        front_wheel = pho::sphere("bike_front_wheel", scnMgr, world, space, wheel_mass, whell_r);
        front_wheel.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));
        colliding_geoms.push_back(front_wheel.geom);
        dBodySetPosition(front_wheel.body, p[0], p[1] - clearance, p[2] - body_height * wheel_pos_z_coefficient);

        dJointGroupID Joint_group_id = dJointGroupCreate(0);
        front_hinge2_joint_id = dJointCreateHinge2(world, Joint_group_id);
        dJointAttach(front_hinge2_joint_id, body.body, front_wheel.body);

        dJointSetHinge2Anchor(front_hinge2_joint_id, p[0], p[1] - clearance, p[2] - body_height * wheel_pos_z_coefficient);
        const dReal axis1[dSA__MAX]{0, 1, 0};
        const dReal axis2[dSA__MAX]{1, 0, 0};
        dJointSetHinge2Axes(front_hinge2_joint_id, axis1, axis2);

        dJointSetHinge2Param(front_hinge2_joint_id, dParamLoStop, -front_whell_direction_angle);
        dJointSetHinge2Param(front_hinge2_joint_id, dParamHiStop, front_whell_direction_angle);
    }

    {
        rear_wheel = pho::sphere("bike_rear_wheel", scnMgr, world, space, wheel_mass, whell_r);
        rear_wheel.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));
        colliding_geoms.push_back(rear_wheel.geom);
        dBodySetPosition(rear_wheel.body, p[0], p[1] - clearance, p[2] + body_height * wheel_pos_z_coefficient);

        dJointGroupID Joint_group_id = dJointGroupCreate(0);
        rear_hinge_joint_id = dJointCreateHinge(world, Joint_group_id);
        dJointAttach(rear_hinge_joint_id, body.body, rear_wheel.body);
        dJointSetHingeAnchor(rear_hinge_joint_id, p[0], p[1] - clearance, p[2] + body_height * wheel_pos_z_coefficient);
        dJointSetHingeAxis(rear_hinge_joint_id, 1, 0, 0);

        //make_fixed_joint(rear_wheel.geom);
    }

    u_word input_length = 0;
    u_word output_length = 0;

    input_length += front_whell_direction_quantity_bits_left;
    input_length += front_whell_direction_quantity_bits_right;

    output_length += front_whell_direction_quantity_bits_left;
    output_length += front_whell_direction_quantity_bits_right;

    // front and rear wheel bytes
    //input_length += front_whell_trotle_quantity_bits + rear_whell_trotle_quantity_bits;
    output_length += front_whell_trotle_quantity_bits + rear_whell_trotle_quantity_bits;

//    // I feel my legs (32)
//    input_length = legs.size() * QUANTITY_BITS_PER_JOINT * QUANTITY_OF_JOINTS_IN_LEG;

    // I feel my velosity [2 bytes / coordinate] (48)
    input_length += 2 * QUANTITY_OF_BITS_IN_BYTE * coordinates_count;

    // I feel time (64)
    input_length += sensors::time::get_data_size();

    // I feel my orientation [2 bytes / coordinate] (48)
    input_length += 2 * QUANTITY_OF_BITS_IN_BYTE * coordinates_count;

//    output_length = legs.size() * QUANTITY_BITS_PER_JOINT * QUANTITY_OF_JOINTS_IN_LEG;
//    force.resize(legs.size() * QUANTITY_OF_JOINTS_IN_LEG, 0.0);
//    distance.resize(legs.size() * QUANTITY_OF_JOINTS_IN_LEG, 0.0);
    auto step = 8;
    video_.reset(new bnn_device_3d::sensors::video(render_window->getWidth() / 4, render_window->getHeight() / 4, step));
    input_length += bnn_device_3d::sensors::video::length * video_->calc_data.size();

    brain_.reset(new bnn::brain_tools(quantity_of_neurons_in_power_of_two,
                                      input_length,
                                      output_length,
                                      threads_count_in_power_of_two));
}

physical_objects::figure& bike::get_body()
{
    return body;
}

std::vector<physical_objects::figure*> bike::get_figures()
{
    std::vector<pho::figure*> value;
    value.push_back(&body);
    value.push_back(&body_sign);
    value.push_back(&front_wheel);
    value.push_back(&rear_wheel);
    return value;
}

Ogre::Vector3 bike::get_camera_place()
{
    const Ogre::Quaternion ort_x(0, 0, 0, -1);
    const dReal* body_q = dBodyGetQuaternion(body.body);
    Ogre::Quaternion body_quat = Ogre::Quaternion(body_q[0], body_q[1], body_q[2], body_q[3]);
    Ogre::Quaternion body_quat_inv = body_quat.Inverse();
    Ogre::Quaternion ort_x_rel = body_quat * ort_x * body_quat_inv;
    ort_x_rel.normalise();
    const dReal* body_p = dBodyGetPosition(body.body);
    Ogre::Vector3 v(ort_x_rel.x + body_p[0], ort_x_rel.y + body_p[1], ort_x_rel.z + body_p[2]);
    return v * 1.2;
}

dReal bike::get_height()
{
    return level;
}

void bike::set_position(dReal x, dReal y, dReal z)
{
    auto* p = dBodyGetPosition(body.body);
    dReal dx = x - p[0];
    dReal dy = y - p[1];
    dReal dz = z - p[2];

    dQuaternion q{1,0,0,0};

    dBodySetPosition(body.body, x, y, z);
    dBodySetQuaternion(body.body, q);

    dBodySetPosition(body_sign.body, x, y, z);
    dBodySetQuaternion(body_sign.body, q);

    //p = dBodyGetPosition(front_wheel.body);
    dBodySetPosition(front_wheel.body, p[0], p[1] - clearance, p[2] - body_length);
    //dBodySetPosition(front_wheel.body, p[0] + dx, p[1] + dy, p[2] + dz);
    dBodySetQuaternion(front_wheel.body, q);

    //p = dBodyGetPosition(rear_wheel.body);
    dBodySetPosition(rear_wheel.body, p[0], p[1] - clearance, p[2] + body_length);
    //dBodySetPosition(rear_wheel.body, p[0] + dx, p[1] + dy, p[2] + dz);
    dBodySetQuaternion(rear_wheel.body, q);

    dBodySetLinearVel(body.body, 0, 0, 0);
    dBodySetLinearVel(body_sign.body, 0, 0, 0);
    dBodySetLinearVel(front_wheel.body, 0, 0, 0);
    dBodySetLinearVel(rear_wheel.body, 0, 0, 0);

    dBodySetAngularVel(body.body, 0, 0, 0);
    dBodySetAngularVel(body_sign.body, 0, 0, 0);
    dBodySetAngularVel(front_wheel.body, 0, 0, 0);
    dBodySetAngularVel(rear_wheel.body, 0, 0, 0);

//    {
//        front_wheel = pho::sphere("bike_front_wheel", scnMgr, world, space, body_width * body_width * body_width * 0.125, body_width / 2);
//        front_wheel.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));
//        colliding_geoms.push_back(front_wheel.geom);
//        dBodySetPosition(front_wheel.body, p[0], p[1] - clearance, p[2] - body_length);

//        dJointGroupID Joint_group_id = dJointGroupCreate(0);
//        front_hinge2_joint_id = dJointCreateHinge2(world, Joint_group_id);
//        dJointAttach(front_hinge2_joint_id, body.body, front_wheel.body);

//        dJointSetHinge2Anchor(front_hinge2_joint_id, p[0], p[1] - clearance, p[2] - body_length);
//        const dReal axis1[dSA__MAX]{0, 1, 0};
//        const dReal axis2[dSA__MAX]{1, 0, 0};
//        dJointSetHinge2Axes(front_hinge2_joint_id, axis1, axis2);

//        dJointSetHinge2Param(front_hinge2_joint_id, dParamLoStop, -M_PI / 12);
//        dJointSetHinge2Param(front_hinge2_joint_id, dParamHiStop, M_PI / 12);
//    }

//    {
//        rear_wheel = pho::sphere("bike_rear_wheel", scnMgr, world, space, body_width * body_width * body_width * 0.125, body_width / 2);
//        rear_wheel.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));
//        colliding_geoms.push_back(rear_wheel.geom);
//        dBodySetPosition(rear_wheel.body, p[0], p[1] - clearance, p[2] + body_length);

//        dJointGroupID Joint_group_id = dJointGroupCreate(0);
//        rear_hinge_joint_id = dJointCreateHinge(world, Joint_group_id);
//        dJointAttach(rear_hinge_joint_id, body.body, rear_wheel.body);
//        dJointSetHingeAnchor(rear_hinge_joint_id, p[0], p[1] - clearance, p[2] + body_length);
//        dJointSetHingeAxis(rear_hinge_joint_id, 1, 0, 0);

//        //make_fixed_joint(rear_wheel.geom);
//    }
}

void bike::step(std::string& debug_str, bool& verbose)
{
    const u_word length = 2 * QUANTITY_OF_BITS_IN_BYTE;
    static constexpr double range = 1.0f;
    static const Ogre::Quaternion ort_x(0, 1, 0, 0);
    static const Ogre::Quaternion ort_y(0, 0, 1, 0);
    static const Ogre::Quaternion ort_z(0, 0, 0, 1);

    static u_word iteration = brain_->get_iteration();
    static u_word old_iteration = 0;
    iteration = brain_->get_iteration();

    if((iteration / 128) > old_iteration)
        old_iteration = iteration / 128;
    else
        verbose = false;

    u_word i = 0;
    double front_direction_left = 0;
    double front_direction_right = 0;
    double front_trotle = 0;
    double rear_trotle = 0;
    double front_direction;

    {
        for(size_t j = 0; j < front_whell_direction_quantity_bits_left; j++)
        {
            if(verbose)
                debug_str += std::to_string(brain_->get_output(i));

            front_direction_left += static_cast<float>(brain_->get_output(i++));
            //front_direction_left += (rand() % 2);
            //front_direction_left += 0.5;
        }

        for(size_t j = 0; j < front_whell_direction_quantity_bits_right; j++)
        {
            if(verbose)
                debug_str += std::to_string(brain_->get_output(i));

            front_direction_right += static_cast<float>(brain_->get_output(i++));
            //front_direction_right += (rand() % 2);
            //front_direction_right += 0.5;
        }

        front_direction_left /= front_whell_direction_quantity_bits_left;
        front_direction_right /= front_whell_direction_quantity_bits_right;

        front_direction = -1
                * (1.0 - abs(front_direction_left - front_direction_right))
                * ((front_direction_left + front_direction_right) / 2.0);

        front_direction *= (((front_direction_left - front_direction_right)
                - (dJointGetHinge2Angle1(front_hinge2_joint_id) / front_whell_direction_angle)) / 1.0);

        front_direction += front_direction_left;
        front_direction -= front_direction_right;
        front_direction -= dJointGetHinge2Angle1(front_hinge2_joint_id) / front_whell_direction_angle;

        //front_direction /= 2;
    //    front_direction /= front_whell_direction_angle;
    }

    {
        for(size_t j = 0; j < front_whell_trotle_quantity_bits; j++)
        {
            if(verbose)
                debug_str += std::to_string(brain_->get_output(i));

            front_trotle += static_cast<float>(brain_->get_output(i++));
        }

        front_trotle /= front_whell_trotle_quantity_bits;
        front_trotle *= 2;
        front_trotle -= dJointGetHinge2Angle2Rate(front_hinge2_joint_id) / 10;
    }

    {
        for(size_t j = 0; j < rear_whell_trotle_quantity_bits; j++)
        {
            if(verbose)
                debug_str += std::to_string(brain_->get_output(i));

            rear_trotle += static_cast<float>(brain_->get_output(i++));
        }

        rear_trotle /= rear_whell_trotle_quantity_bits;
        rear_trotle *= 3;
        rear_trotle -= dJointGetHingeAngleRate(rear_hinge_joint_id) / 10;
    }

    front_trotle = 0;
    dJointAddHinge2Torques(front_hinge2_joint_id, front_direction, front_trotle);
    dJointAddHingeTorque(rear_hinge_joint_id, rear_trotle);

    front_direction = dJointGetHinge2Angle1(front_hinge2_joint_id);
    //return value_in_range(dJointGetHingeAngle(joint_id), angle_start, angle_end);
    //front_trotle = dJointGetHinge2Angle2(front_hinge2_joint_id);

    body.step();
    body_sign.step();
    front_wheel.step();
    rear_wheel.step();

//#ifdef learning_creature
//    u_word data = teacher_->get_data();
//#endif

    //#define RANDOM_STEPS
//#ifndef RANDOM_STEPS
//    for(size_t i = 0; i < legs.size(); i++)
//    {
//        fs = force[i * QUANTITY_OF_JOINTS_IN_LEG + 0];
//        st = force[i * QUANTITY_OF_JOINTS_IN_LEG + 1];

//#ifdef learning_creature
//        float c = static_cast<float>(((static_cast<int>(data) >> (i * 2)) & 1) * 2 - 1);
//        if(teacher_->get_count())
//        {
//            auto k = static_cast<float>(teacher_->get_count()) / static_cast<float>(teacher_->get_count_max());
//            fs = fs * (1.f - k) + c * k;
//            st = st * (1.f - k) + c * k;
//        }
//#endif

//        //#define REAR_LEGS_OFF
//#ifdef REAR_LEGS_OFF
//        if(NUMBER_OF_REAR_LEFT_LEG == i || NUMBER_OF_REAR_RIGHT_LEG == i)
//        {
//            fs = 0;
//            st = 0;
//        }
//#endif

//        legs[i].step(fs, st);

//        distance[i * QUANTITY_OF_JOINTS_IN_LEG + 0] = fs;
//        distance[i * QUANTITY_OF_JOINTS_IN_LEG + 1] = st;
//    }
//#else
//    // random movements
//    for(int i = 0; i < LEGS_QUANTITY; i++)
//    {
//        fs = ((float)rand() / RAND_MAX) * 2 - 1;
//        st = ((float)rand() / RAND_MAX) * 2 - 1;

//        legs[i].step(fs, st);
//    }
//#endif

    u_word count_input = 0;

    // steering wheel
    data_processing_method_->set_inputs(*brain_.get(), count_input,
                                        front_whell_direction_quantity_bits_left + front_whell_direction_quantity_bits_right,
                                        front_direction, -front_whell_direction_angle,
                                        front_whell_direction_angle, debug_str, verbose);
    debug_str += " ";

    debug_str += "\nvideo [\n";
    video_->set_inputs(*brain_.get(), count_input, bnn_device_3d::sensors::video::length, range, debug_str, verbose);

//    brain_->set_input(count_input++, front_direction);
//    brain_->set_input(count_input++, front_trotle);

//    debug_str += "] legs [ ";
//    for(size_t i = 0; i < distance.size(); i++)
//    {
//        data_processing_method_->set_inputs(*brain_, count_input, QUANTITY_BITS_PER_JOINT, distance[i], -range, range, debug_str, verbose);
//        if(i % 2)
//            debug_str += " ";
//    }

    debug_str += "]\nvel [ ";
    speedometer_.set_inputs(body.body, *brain_.get(), count_input, length, -range, range, debug_str, verbose);

    debug_str += " ]\ndir [ ";
    gyroscope_.set_inputs(body.body, *brain_.get(), count_input, length, -range, range, debug_str, verbose);

    debug_str += " ]\ntime [ ";
    time_.set_inputs(*brain_.get(), count_input, debug_str, verbose);

//    if(verbose)
//        debug_str += "] out [ ";

//    // I can move legs
//    for(u_word i = 0; i < force.size(); i++)
//    {
//        force[i] = 0;

//        for(size_t j = 0; j < QUANTITY_BITS_PER_JOINT; j++)
//        {
//            force[i] += static_cast<float>(brain_->get_output(i * QUANTITY_BITS_PER_JOINT + j) * 2 - 1);

//            if(verbose)
//                debug_str += std::to_string(brain_->get_output(i * QUANTITY_BITS_PER_JOINT + j));
//        }

//        if(verbose)
//            debug_str += " ";

//        force[i] /= QUANTITY_BITS_PER_JOINT;
//    }

//    if(verbose)
//    {
//        debug_str += "]\n";
//        brain_->get_debug_string(debug_str);
//    }
}

} // namespace bnn_device_3d::creatures::bike
