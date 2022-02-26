/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "creature.h"

#include <unistd.h>

namespace bnn_device_3d::creatures
{

creature::~creature()
{
    logging("");
    stop();
}

creature::creature(Ogre::SceneManager* scnMgr, dWorldID world)
{
    switch (1)
    {
    case 0:
        data_processing_method_.reset(new data_processing_method_binary());
        break;
    case 1:
        data_processing_method_.reset(new data_processing_method_linearly());
        break;
    case 2:
        data_processing_method_.reset(new data_processing_method_linearly_single());
        break;
    case 3:
        data_processing_method_.reset(new data_processing_method_logarithmic());
        break;
    }

#ifdef learning_creature
    teacher.reset(new teacher_walking());
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
        body = cube("body", scnMgr, world, space, body_mass, body_width, body_height, body_length);

        body.set_material(figure::create_material_chess(128, 32, 0x777777ff, 0x333333ff));

        colliding_geoms.push_back(body.geom);
    }

    {
        body_sign = cube("body_sign", scnMgr, world, space, 0.00001, body_width * 0.95, body_height, body_width * 0.95);
        //body_sign = cube("body_sign", scnMgr, world, space, 0.00001, body_length * 2, body_length * 2, body_length * 2);

        auto *p = dBodyGetPosition(body.body);

        dBodySetPosition(body_sign.body, p[0], p[1] + body_height / 50.0, p[2]);
        //dBodySetPosition(body_sign.body, p[0], p[1] + body_height * 5.0, p[2]);

        body_sign.set_material(body_sign.create_material_body_sign(256));

        make_fixed_joint(body_sign.geom);
    }

    if(1)
    {
        // leg_fl
        dQuaternion q = {M_SQRT1_2, 0, M_SQRT1_2, 0};
        legs.push_back(leg("leg_fl", scnMgr, world, space,
                           -body_width / 2, 0, -body_length / 2,
                           q, -1, 0x0000ffff));
    }

    if(1)
    {
        // leg_fr
        dQuaternion q = {1, 0, 0, 0};
        legs.push_back(leg("leg_fr", scnMgr, world, space,
                           body_width / 2, 0, -body_length / 2,
                           q, 1, 0x777777ff));
    }

    if(1)
    {
        // middle_l
        dQuaternion q = {M_SQRT1_2, 0, M_SQRT1_2, 0};
        legs.push_back(leg("middle_l", scnMgr, world, space,
                           -body_width / 2 + 25 * device_3d_SCALE, 0, 0,
                           q, -1, 0x777777ff));
    }

    if(1)
    {
        // middle_r
        dQuaternion q = {1, 0, 0, 0};
        legs.push_back(leg("middle_r", scnMgr, world, space,
                           body_width / 2 - 25 * device_3d_SCALE, 0, 0,
                           q, 1, 0x777777ff));
    }

    if(1)
    {
        // leg_rl
        dQuaternion q = {M_SQRT1_2, 0, M_SQRT1_2, 0};
        legs.push_back(leg("leg_rl", scnMgr, world, space,
                           -body_width / 2, 0, body_length / 2,
                           q, -1, 0x777777ff));
    }

    if(1)
    {
        // leg_rr
        dQuaternion q = {1, 0, 0, 0};
        legs.push_back(leg("leg_rr", scnMgr, world, space,
                           body_width / 2, 0, body_length / 2,
                           q, 1, 0x777777ff));
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

    // I feel my orientation [2 bytes / coordinate] (48)
    input_length += 2 * QUANTITY_OF_BITS_IN_BYTE * coordinates_count;

#ifdef creature_sees_world
    // I see figures with two eyes (128)
    input_length += inputs_from_world_objests;
#endif

    // input_length = 448

    // (16)
    output_length = legs.size() * QUANTITY_BITS_PER_JOINT * QUANTITY_OF_JOINTS_IN_LEG;

    force.resize(legs.size() * QUANTITY_OF_JOINTS_IN_LEG);

    distance.resize(legs.size() * QUANTITY_OF_JOINTS_IN_LEG);

    brain_.reset(new bnn::brain_tools(random_array_length_in_power_of_two,
                                      quantity_of_neurons_in_power_of_two,
                                      input_length,
                                      output_length,
                                      threads_count_in_power_of_two));

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

    for(size_t i = 0; i < force.size(); i++)
    {
        force[i] = 0;
        distance[i] = 0;
    }
}

std::vector<figure*> creature::get_figures()
{
    std::vector<figure*> value;

    value.push_back(&body);

    std::for_each(legs.begin(),legs.end(), [&](leg& l)
    {
        auto figures_of_leg = l.get_figures();

        std::for_each(figures_of_leg.begin(),figures_of_leg.end(), [&](figure* f) { value.push_back(f); });
    });

    return value;
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
    logging("creature::start() begin");

    brain_->start();

#ifdef learning_creature
    teacher->start();
#endif

    //brain_friend_->debug_out();

    logging("creature::start() end");
}

void creature::step(std::list<dGeomID>& distance_geoms, bool& verbose)
{
    std::string debug_str = "legs [ ";

    //dBodySetAngularVel(body.body, 0, 5, 0);

    double fs;
    double st;

    _word length = 2 * QUANTITY_OF_BITS_IN_BYTE;

    static double range = 1.0f;

    static const Ogre::Quaternion ort_x(0, 1, 0, 0);
    static const Ogre::Quaternion ort_y(0, 0, 1, 0);
    static const Ogre::Quaternion ort_z(0, 0, 0, 1);

    body.step();

    body_sign.step();

#ifdef learning_creature
    _word data = teacher->get_data();
#endif

    //#define RANDOM_STEPS
#ifndef RANDOM_STEPS
    for(size_t i = 0; i < legs.size(); i++)
    {
        fs = force[i * QUANTITY_OF_JOINTS_IN_LEG + 0];
        st = force[i * QUANTITY_OF_JOINTS_IN_LEG + 1];

#ifdef learning_creature
        float c = static_cast<float>(((static_cast<int>(data) >> (i * 2)) & 1) * 2 - 1);
        if(teacher->get_count())
        {
            auto k = static_cast<float>(teacher->get_count()) / static_cast<float>(teacher->get_count_max());
            fs = fs * (1.f - k) + c * k;
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

    _word count_input = 0;

    // Set inputs by legs states
    for(size_t i = 0; i < distance.size(); i++)
    {
        data_processing_method_->set_inputs(*brain_, count_input, QUANTITY_BITS_PER_JOINT, distance[i], -range, range, debug_str);
        if(i % 2)
            debug_str += " ";
    }

    debug_str += "]\nvel [ ";
    speedometer_.set_inputs(body.body, *brain_.get(), count_input, length, -range, range, debug_str);

    debug_str += " ]\ndir [ ";
    gyroscope_.set_inputs(body.body, *brain_.get(), count_input, length, -range, range, debug_str);

#ifdef creature_sees_world
#ifdef show_debug_data
    debug_str += " ]\nobj [ ";
#endif
    // I see figures with two eyes
    for_each(distance_geoms.begin(), distance_geoms.end(), [&](dGeomID& value)
    {
#define MAX_DISTANCE_TO_OBJECT 100
        distance_.set_inputs(legs[NUMBER_OF_FRONT_LEFT_LEG].first.body, value, *brain_.get(), count_input, 32, MAX_DISTANCE_TO_OBJECT, debug_str);
        distance_.set_inputs(legs[NUMBER_OF_FRONT_RIGHT_LEG].first.body, value, *brain_.get(), count_input, 32, MAX_DISTANCE_TO_OBJECT, debug_str);

        debug_str += " ";
    });
#else

#ifdef show_debug_data
    debug_str += " ";
#endif
#endif

    debug_str += "]\nout [ ";

    // I can move legs
    for(_word i = 0; i < force.size(); i++)
    {
        force[i] = 0;

        for(size_t j = 0; j < QUANTITY_BITS_PER_JOINT; j++)
        {
            force[i] += static_cast<float>(brain_->get_output(i * QUANTITY_BITS_PER_JOINT + j) * 2 - 1);

#ifdef show_debug_data
            debug_str += std::to_string(brain_->get_output(i * QUANTITY_BITS_PER_JOINT + j));
#endif
        }

#ifdef show_debug_data
            debug_str += " ";
#endif

        force[i] /= QUANTITY_BITS_PER_JOINT;
    }

    debug_str += " ]\n";
#ifdef show_debug_data
    static _word iter = 0;
    _word www = brain_->get_iteration();
    if(www > iter)
    {
        iter = www;
        debug_str += std::to_string(www);

        if(verbose)
        {
            verbose = false;
            std::cout << debug_str << std::endl;
        }
    }
#endif
}

void creature::stop()
{
    logging("creature::stop() begin");

    brain_->stop();
#ifdef learning_creature
    teacher->stop();
#endif

    logging("creature::stop() end");
}

} // bnn_device_3d::creatures
