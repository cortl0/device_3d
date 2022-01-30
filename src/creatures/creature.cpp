/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "creature.h"

#include <unistd.h>

creature::~creature()
{
    logging("");
    stop();
}

creature::creature(Ogre::SceneManager* scnMgr, dWorldID world, std::vector<_word> &input_from_world)
    : input_from_world(input_from_world)
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

    space = dSimpleSpaceCreate(nullptr);

    body = cube("body", scnMgr, world, space, body_mass, body_width, body_height, body_length);

    body.set_material(figure::create_material_chess(128, 32, 0x777777ff, 0x333333ff));

    colliding_geoms.push_back(body.geom);

    {
        // leg_fl
        dQuaternion q = {M_SQRT1_2, 0, M_SQRT1_2, 0};
        legs.push_back(leg("leg_fl", scnMgr, world, space,
                           -body_width / 2, 0, -body_length / 2,
                           q, -1, 0x000077ff));
    }

    {
        // leg_fr
        dQuaternion q = {1, 0, 0, 0};
        legs.push_back(leg("leg_fr", scnMgr, world, space,
                           body_width / 2, 0, -body_length / 2,
                           q, 1, 0x777777ff));
    }

    {
        // leg_rl
        dQuaternion q = {M_SQRT1_2, 0, M_SQRT1_2, 0};
        legs.push_back(leg("leg_rl", scnMgr, world, space,
                           -body_width / 2, 0, body_length / 2,
                           q, -1, 0x777777ff));
    }

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

        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach(j, dGeomGetBody(body.geom), dGeomGetBody(legs[i].first.geom));
        dJointSetFixed(j);
    }

    // I feel my legs (128)
    _word input_length = legs.size() * QUANTITY_OF_BITS_IN_BYTE * QUANTITY_OF_JOINTS_IN_LEG * 2;

    // I feel my velosity [2 bytes / coordinate] (48)
    input_length += 2 * QUANTITY_OF_BITS_IN_BYTE * coordinates_count;

    // I feel my orientation by the three dots on my body [2 bytes / coordinate] (144)
    input_length += 3 * 2 * QUANTITY_OF_BITS_IN_BYTE * coordinates_count;

#ifdef creature_sees_world
    // I see figures with two eyes (128)
    input_length += inputs_from_world_objests;
#endif

    // input_length = 448

    // (16)
    _word output_length = legs.size() * 2 * QUANTITY_OF_JOINTS_IN_LEG;

    brain_.reset(new bnn::brain_tools(random_array_length_in_power_of_two,
                                              quantity_of_neurons_in_power_of_two,
                                              input_length,
                                              output_length,
                                              1));

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

    for(int i = 0; i < force_distance_count; i++)
    {
        force[i] = 0;
        distance[i] = 0;
    }
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

void creature::step()
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

    dReal x_scalar;
    dReal y_scalar;
    dReal z_scalar;

    body.step();

#ifdef learning_creature
    _word data = teacher->get_data();
#endif

#if(1)
    for(size_t i = 0; i < legs.size(); i++)
    {
        fs = force[i * 2 + 0];
        st = force[i * 2 + 1];

#ifdef learning_creature
        float c = static_cast<float>(((static_cast<int>(data) >> (i * 2)) & 1) * 2 - 1);
        if(teacher->get_count())
        {
            auto k = static_cast<float>(teacher->get_count()) / static_cast<float>(teacher->get_count_max());
            fs = fs * (1.f - k) + c * k;
        }
#endif

        legs[i].step(fs, st);

        distance[i * 2 + 0] = fs;
        distance[i * 2 + 1] = st;
    }
#else
    // random movements
    for(int i = 0; i < leg_count; i++)
    {
        fs = ((float)rand() / RAND_MAX) * 2 - 1;
        st = ((float)rand() / RAND_MAX) * 2 - 1;
        legs[i].step(fs, st);
    }
#endif

    _word count_input = 0;

    // Set inputs by legs states
    for(int i = 0; i < force_distance_count; i++)
    {
        data_processing_method_->set_inputs(*brain_, count_input, QUANTITY_OF_BITS_IN_BYTE, distance[i], -range, range, debug_str);
        if(i % 2)
            debug_str += " ";
    }
    debug_str += " ] vel [ ";

    const dReal* body_q = dBodyGetQuaternion(body.body);
    Ogre::Quaternion body_quat = Ogre::Quaternion(body_q[0], body_q[1], body_q[2], body_q[3]);
    Ogre::Quaternion body_quat_inv = body_quat.Inverse();

    // Relative ort vectors
    Ogre::Quaternion ort_x_rel = body_quat * ort_x * body_quat_inv;
    Ogre::Quaternion ort_y_rel = body_quat * ort_y * body_quat_inv;
    Ogre::Quaternion ort_z_rel = body_quat * ort_z * body_quat_inv;

    ort_x_rel.normalise();
    ort_y_rel.normalise();
    ort_z_rel.normalise();

    {
        // Set inputs by velosity
        auto *vel = dBodyGetLinearVel(body.body);

        x_scalar = ort_x_rel[1] * vel[0] + ort_x_rel[2] * vel[1] + ort_x_rel[3] * vel[2];
        y_scalar = ort_y_rel[1] * vel[0] + ort_y_rel[2] * vel[1] + ort_y_rel[3] * vel[2];
        z_scalar = ort_z_rel[1] * vel[0] + ort_z_rel[2] * vel[1] + ort_z_rel[3] * vel[2];
        data_processing_method_->set_inputs(*brain_, count_input, length, x_scalar, -range, range, debug_str);
        debug_str += " ";
        data_processing_method_->set_inputs(*brain_, count_input, length, y_scalar, -range, range, debug_str);
        debug_str += " ";
        data_processing_method_->set_inputs(*brain_, count_input, length, z_scalar, -range, range, debug_str);
    }

    debug_str += " ]\ndir [ x: ";

    {
        // Set inputs by direction
        x_scalar = ort_x_rel[1] * 1 + ort_x_rel[2] * 0 + ort_x_rel[3] * 0;
        y_scalar = ort_x_rel[1] * 0 + ort_x_rel[2] * 1 + ort_x_rel[3] * 0;
        z_scalar = ort_x_rel[1] * 0 + ort_x_rel[2] * 0 + ort_x_rel[3] * 1;
        data_processing_method_->set_inputs(*brain_, count_input, length, x_scalar, -range, range, debug_str);
        debug_str += " ";
        data_processing_method_->set_inputs(*brain_, count_input, length, y_scalar, -range, range, debug_str);
        debug_str += " ";
        data_processing_method_->set_inputs(*brain_, count_input, length, z_scalar, -range, range, debug_str);
        debug_str += "  y: ";

        x_scalar = ort_y_rel[1] * 1 + ort_y_rel[2] * 0 + ort_y_rel[3] * 0;
        y_scalar = ort_y_rel[1] * 0 + ort_y_rel[2] * 1 + ort_y_rel[3] * 0;
        z_scalar = ort_y_rel[1] * 0 + ort_y_rel[2] * 0 + ort_y_rel[3] * 1;
        data_processing_method_->set_inputs(*brain_, count_input, length, x_scalar, -range, range, debug_str);
        debug_str += " ";
        data_processing_method_->set_inputs(*brain_, count_input, length, y_scalar, -range, range, debug_str);
        debug_str += " ";
        data_processing_method_->set_inputs(*brain_, count_input, length, z_scalar, -range, range, debug_str);
        debug_str += "  z: ";

        x_scalar = ort_z_rel[1] * 1 + ort_z_rel[2] * 0 + ort_z_rel[3] * 0;
        y_scalar = ort_z_rel[1] * 0 + ort_z_rel[2] * 1 + ort_z_rel[3] * 0;
        z_scalar = ort_z_rel[1] * 0 + ort_z_rel[2] * 0 + ort_z_rel[3] * 1;
        data_processing_method_->set_inputs(*brain_, count_input, length, x_scalar, -range, range, debug_str);
        debug_str += " ";
        data_processing_method_->set_inputs(*brain_, count_input, length, y_scalar, -range, range, debug_str);
        debug_str += " ";
        data_processing_method_->set_inputs(*brain_, count_input, length, z_scalar, -range, range, debug_str);
        debug_str += " ";
    }

    //    std::unique_ptr<data_processing_method_base> dpm(new data_processing_method_logarithmic());
    //std::unique_ptr<data_processing_method_base> dpm(new data_processing_method_binary());
    //std::unique_ptr<data_processing_method> dpm(new data_processing_method_linearly());

#ifdef creature_sees_world
#ifdef show_debug_data
    debug_str += " ]\nobj [ ";
#endif
    // I see figures with two eyes
    for_each(input_from_world.begin(), input_from_world.end(), [&](_word value)
    {
        //        dpm->set_inputs(*brn, count_input, _word_bits, value, 0.f, 10.0f, debug_str);
        if(0)
        {
            bool b = false;
            for(int i = QUANTITY_OF_BITS_IN_WORD - 1; i >= 0; i--)
            {
                if(!b)
                    if(value & (1 << (i - 1)))
                        b = true;

                value |= (b << (i - 1));
            }
        }

        if(1)
        {
            for(_word i = 0; i < QUANTITY_OF_BITS_IN_WORD; i++)
            {
#ifdef show_debug_data
                debug_str += std::to_string((value >> i) & 1);
#endif

                brain_->set_input(count_input++, (value >> i) & 1);
            }
        }

        debug_str += " ";

    });
#else
#ifdef show_debug_data
    debug_str += " ";
#endif
#endif

    debug_str += " ] out [ ";

    // I can move legs
    for(_word i = 0; i < force_distance_count; i++)
    {
#ifdef show_debug_data
        debug_str += std::to_string(brn->get_out(i * 2)) + std::to_string(brn->get_out(i * 2 + 1));
#endif

        force[i] = static_cast<float>(brain_->get_output(i * 2)) - static_cast<float>(brain_->get_output(i * 2 + 1));
    }

    debug_str += " ]\n";
#ifdef show_debug_data
    static _word iter = 0;
    _word www = brn->iteration;
    if(www > iter)
    {
        iter = www;
        debug_str += std::to_string(www);
        std::cout << debug_str << std::endl;
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
