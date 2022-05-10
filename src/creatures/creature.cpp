/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "creature.h"

#include <unistd.h>

#include "config.hpp"

namespace dpm = bnn_device_3d::data_processing_methods;

namespace pho = bnn_device_3d::physical_objects;

namespace sens = bnn_device_3d::sensors;

namespace tch = bnn_device_3d::teachers;

namespace bnn_device_3d::creatures
{

creature::~creature()
{
    logging("");
    stop();
}

creature::creature(Ogre::RenderWindow* render_window, Ogre::SceneManager* scnMgr, dWorldID world)
{
    switch (1)
    {
    case 0:
        data_processing_method_.reset(new dpm::data_processing_method_binary());
        break;
    case 1:
        data_processing_method_.reset(new dpm::data_processing_method_linearly());
        break;
    case 2:
        data_processing_method_.reset(new dpm::data_processing_method_linearly_single());
        break;
    case 3:
        data_processing_method_.reset(new dpm::data_processing_method_logarithmic());
        break;
    }

#ifdef learning_creature
    teacher_.reset(new tch::teacher_walking());
#endif

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

    {
        body_sign = pho::cube("body_sign", scnMgr, world, space, 0.00001, body_width * 0.95, body_height, body_width * 0.95);
        //body_sign = cube("body_sign", scnMgr, world, space, 0.00001, body_length * 2, body_length * 2, body_length * 2);

        auto *p = dBodyGetPosition(body.body);

        dBodySetPosition(body_sign.body, p[0], p[1] + body_height / 50.0, p[2]);
        //dBodySetPosition(body_sign.body, p[0], p[1] + body_height * 5.0, p[2]);

        body_sign.set_material(body_sign.create_material_body_sign(256));

        make_fixed_joint(body_sign.geom);
    }

    if(0)
    {
        // leg_fl
        dQuaternion q = {M_SQRT1_2, 0, M_SQRT1_2, 0};
        legs.push_back(leg("leg_fl", scnMgr, world, space,
                           -body_width / 2, 0, -body_length / 2,
                           q, -1, 0x0000ffff));
    }

    if(0)
    {
        // leg_fr
        dQuaternion q = {1, 0, 0, 0};
        legs.push_back(leg("leg_fr", scnMgr, world, space,
                           body_width / 2, 0, -body_length / 2,
                           q, 1, COLOR_LIGHT));
    }

    if(1)
    {
        // middle_l
        dQuaternion q = {M_SQRT1_2, 0, M_SQRT1_2, 0};
        legs.push_back(leg("middle_l", scnMgr, world, space,
                           -body_width / 2, 0, 0,
                           q, -1, COLOR_LIGHT));
    }

    if(1)
    {
        // middle_r
        dQuaternion q = {1, 0, 0, 0};
        legs.push_back(leg("middle_r", scnMgr, world, space,
                           body_width / 2, 0, 0,
                           q, 1, COLOR_LIGHT));
    }

    if(0)
    {
        // leg_rl
        dQuaternion q = {M_SQRT1_2, 0, M_SQRT1_2, 0};
        legs.push_back(leg("leg_rl", scnMgr, world, space,
                           -body_width / 2, 0, body_length / 2,
                           q, -1, COLOR_LIGHT));
    }

    if(0)
    {
        // leg_rr
        dQuaternion q = {1, 0, 0, 0};
        legs.push_back(leg("leg_rr", scnMgr, world, space,
                           body_width / 2, 0, body_length / 2,
                           q, 1, COLOR_LIGHT));
    }

    for (size_t i = 0; i < legs.size(); i++)
    {
        colliding_geoms.push_back(legs[i].second.geom);
        colliding_geoms.push_back(legs[i].third.geom);
        make_fixed_joint(legs[i].first.geom);
    }

    // I feel my legs (32)
    input_length = legs.size() * QUANTITY_BITS_PER_JOINT * QUANTITY_OF_JOINTS_IN_LEG;

    // I feel my velosity [2 bytes / coordinate] (48)
    input_length += 2 * QUANTITY_OF_BITS_IN_BYTE * coordinates_count;

    // I feel time (64)
    input_length += sensors::time::get_data_size();

    // I feel my orientation [2 bytes / coordinate] (48)
    input_length += 2 * QUANTITY_OF_BITS_IN_BYTE * coordinates_count;

    output_length = legs.size() * QUANTITY_BITS_PER_JOINT * QUANTITY_OF_JOINTS_IN_LEG;
    force.resize(legs.size() * QUANTITY_OF_JOINTS_IN_LEG, 0.0);
    distance.resize(legs.size() * QUANTITY_OF_JOINTS_IN_LEG, 0.0);
    auto step = 16;
    video_.reset(new bnn_device_3d::sensors::video(render_window->getWidth() / 4, render_window->getHeight() / 4, step));
    input_length += bnn_device_3d::sensors::video::length * video_->calc_data.size();

    brain_.reset(new bnn::brain_tools(quantity_of_neurons_in_power_of_two,
                                      input_length,
                                      output_length,
                                      threads_count_in_power_of_two));

    //brain_->save_random();

    brain_->primary_filling();

    {
        //   dJointGroupID cg_fl = dJointGroupCreate (0);
        //    dJointGroupID cg_fr = dJointGroupCreate (0);
        //    dJointGroupID cg_rl = dJointGroupCreate (0);
        //    dJointGroupID cg_rr = dJointGroupCreate (0);

        //        c = dJointCreateBall (world, cg_fl);
        //        c = dJointCreateHinge (world, cg_fl);
        //        c = dJointCreateSlider (world, cg_fl);
        //        c = dJointCreateContact (world, cg_fl, &contact[i]);
        //        c = dJointCreateHinge2 (world, cg_fl);
        //        c = dJointCreateUniversal (world, cg_fl);
        //        c = dJointCreatePR (world, cg_fl);
        //        c = dJointCreatePU (world, cg_fl);
        //        c = dJointCreatePiston (world, cg_fl);
        //        c = dJointCreateFixed (world, cg_fl);
        //        c = dJointCreateNull (world, cg_fl);
        //        c = dJointCreateAMotor (world, cg_fl);
        //        c = dJointCreateLMotor (world, cg_fl);
        //        c = dJointCreatePlane2D (world, cg_fl);
        //        c = dJointCreateDBall (world, cg_fl);
        //        c = dJointCreateDHinge (world, cg_fl);
        //        c = dJointCreateTransmission (world, cg_fl);
    }
}

std::vector<pho::figure*> creature::get_figures()
{
    std::vector<pho::figure*> value;

    value.push_back(&body);

    value.push_back(&body_sign);

    for(auto& leg : legs)
        for(auto figure : leg.get_figures())
            value.push_back(figure);

    return value;
}

Ogre::Vector3 creature::get_camera_place()
{
    static const Ogre::Quaternion ort_x(0, 0, 0, -0.5);
    const dReal* body_q = dBodyGetQuaternion(body.body);
    Ogre::Quaternion body_quat = Ogre::Quaternion(body_q[0], body_q[1], body_q[2], body_q[3]);
    Ogre::Quaternion body_quat_inv = body_quat.Inverse();
    Ogre::Quaternion ort_x_rel = body_quat * ort_x * body_quat_inv;
    ort_x_rel.normalise();
    const dReal* body_p = dBodyGetPosition(body.body);
    //Ogre::Vector3 v(ort_x_rel.x + body_p[0], ort_x_rel.y + body_p[1], ort_x_rel.z + body_p[2]);
    Ogre::Vector3 v(body_p[0], body_p[1], body_p[2]);

    return v;
}

void creature::set_position(dReal x, dReal y, dReal z)
{
    auto *p = dBodyGetPosition(body.body);
    dReal dx = x - p[0];
    dReal dy = y - p[1];
    dReal dz = z - p[2];

    dBodySetPosition(body.body, x, y, z);

    //    auto *p0 = dBodyGetPosition (body_sph0.body);
    //    dBodySetPosition(body_sph0.body, p0[0] + dx, p0[1] + dy, p0[2] + dz);

    //    auto *p1 = dBodyGetPosition (body_sph1.body);
    //    dBodySetPosition(body_sph1.body, p1[0] + dx, p1[1] + dy, p1[2] + dz);

    //    auto *p2 = dBodyGetPosition (body_sph2.body)    if(bnn::state::started == state_);

    //    dBodySetPosition(body_sph2.body, p2[0] + dx, p2[1] + dy, p2[2] + dz);

    dQuaternion q = {1, 0, 0, 0};
    dQuaternion q_rev = {0, 0, 1, 0};

    for (size_t i = 0; i < legs.size(); i++)
    {
        if(i % 2)
            legs[i].relocate(dx, dy, dz, q);
        else
            legs[i].relocate(dx, dy, dz, q_rev);
    }

    //    legs[leg_fl].relocate(dx, dy, dz, q_rev);
    //    legs[leg_fr].relocate(dx, dy, dz, q);
    //    legs[leg_rl].relocate(dx, dy, dz, q_rev);
    //    legs[leg_rr].relocate(dx, dy, dz, q);
}

void creature::start()
{
    brain_->start();

#ifdef learning_creature
    teacher_->start();
#endif
}

void creature::step(std::string& debug_str, bool& verbose)
{
    const u_word length = 2 * QUANTITY_OF_BITS_IN_BYTE;
    static constexpr double range = 1.0f;
    static const Ogre::Quaternion ort_x(0, 1, 0, 0);
    static const Ogre::Quaternion ort_y(0, 0, 1, 0);
    static const Ogre::Quaternion ort_z(0, 0, 0, 1);
    double fs, st;



    static u_word iteration = brain_->get_iteration();
    static u_word old_iteration = 0;
    iteration = brain_->get_iteration();

    if((iteration / 128) > old_iteration)
        old_iteration = iteration / 128;
    else
        verbose = false;

    body.step();
    body_sign.step();

#ifdef learning_creature
    u_word data = teacher_->get_data();
#endif

    //#define RANDOM_STEPS
#ifndef RANDOM_STEPS
    for(size_t i = 0; i < legs.size(); i++)
    {
        fs = force[i * QUANTITY_OF_JOINTS_IN_LEG + 0];
        st = force[i * QUANTITY_OF_JOINTS_IN_LEG + 1];

#ifdef learning_creature
        float c = static_cast<float>(((static_cast<int>(data) >> (i * 2)) & 1) * 2 - 1);
        if(teacher_->get_count())
        {
            auto k = static_cast<float>(teacher_->get_count()) / static_cast<float>(teacher_->get_count_max());
            fs = fs * (1.f - k) + c * k;
            st = st * (1.f - k) + c * k;
        }
#endif

        //#define REAR_LEGS_OFF
#ifdef REAR_LEGS_OFF
        if(NUMBER_OF_REAR_LEFT_LEG == i || NUMBER_OF_REAR_RIGHT_LEG == i)
        {
            fs = 0;
            st = 0;
        }
#endif

        legs[i].step(fs, st);

        distance[i * QUANTITY_OF_JOINTS_IN_LEG + 0] = fs;
        distance[i * QUANTITY_OF_JOINTS_IN_LEG + 1] = st;
    }
#else
    // random movements
    for(int i = 0; i < LEGS_QUANTITY; i++)
    {
        fs = ((float)rand() / RAND_MAX) * 2 - 1;
        st = ((float)rand() / RAND_MAX) * 2 - 1;

        legs[i].step(fs, st);
    }
#endif

    u_word count_input = 0;

    debug_str += "\nvideo [\n";
    video_->set_inputs(*brain_.get(), count_input, bnn_device_3d::sensors::video::length, range, debug_str, verbose);

    debug_str += "] legs [ ";
    for(size_t i = 0; i < distance.size(); i++)
    {
        data_processing_method_->set_inputs(*brain_, count_input, QUANTITY_BITS_PER_JOINT, distance[i], -range, range, debug_str, verbose);
        if(i % 2)
            debug_str += " ";
    }

    debug_str += "]\nvel [ ";
    speedometer_.set_inputs(body.body, *brain_.get(), count_input, length, -range, range, debug_str, verbose);

    debug_str += " ]\ndir [ ";
    gyroscope_.set_inputs(body.body, *brain_.get(), count_input, length, -range, range, debug_str, verbose);

    debug_str += " ]\ntime [ ";
    time_.set_inputs(*brain_.get(), count_input, debug_str, verbose);

    if(verbose)
        debug_str += "] out [ ";

    // I can move legs
    for(u_word i = 0; i < force.size(); i++)
    {
        force[i] = 0;

        for(size_t j = 0; j < QUANTITY_BITS_PER_JOINT; j++)
        {
            force[i] += static_cast<float>(brain_->get_output(i * QUANTITY_BITS_PER_JOINT + j) * 2 - 1);

            if(verbose)
                debug_str += std::to_string(brain_->get_output(i * QUANTITY_BITS_PER_JOINT + j));
        }

        if(verbose)
            debug_str += " ";

        force[i] /= QUANTITY_BITS_PER_JOINT;
    }

    if(verbose)
    {
        debug_str += "]\n";
        brain_->get_debug_string(debug_str);
    }
}

void creature::stop()
{
    brain_->stop();
#ifdef learning_creature
    teacher_->stop();
#endif
}

} // namespace bnn_device_3d::creatures
