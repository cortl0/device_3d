/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "creature.h"

creature::creature()
{

}

creature::creature(Ogre::SceneManager* scnMgr, dWorldID world, std::shared_ptr<std::vector<uint32>> input_from_world)
    : input_from_world(input_from_world)
{
    switch (1)
    {
    case 0:
        data_processing_method.reset(new data_processing_method_binary());
        break;
    case 1:
        data_processing_method.reset(new data_processing_method_linearly());
        break;
    case 2:
        data_processing_method.reset(new data_processing_method_linearly_single());
        break;
    }

#ifdef learning_creature
    teacher.reset(new teacher_walking());
#endif

    for(int i = 0; i < force_distance_count; i++)
    {
        force[i] = 0;
        distance[i] = 0;
    }

    space = dSimpleSpaceCreate(nullptr);

    body = cube("body", scnMgr, world, space, body_mass, body_width, body_height, body_length);

    body.set_material(figure::create_material_chess(128, 32, 0x777777ff, 0x333333ff));

    colliding_geoms.push_back(body.geom);

    //    body_sph0 = sphere("body_sph0", scnMgr, world, space, body_sph_mass, body_sph_r);
    //    body_sph1 = sphere("body_sph1", scnMgr, world, space, body_sph_mass, body_sph_r);
    //    body_sph2 = sphere("body_sph2", scnMgr, world, space, body_sph_mass, body_sph_r);

    //    dBodySetPosition (body_sph0.body, 0, 0, 101);
    //    dBodySetPosition (body_sph1.body, 0, 0, 0);
    //    dBodySetPosition (body_sph2.body, 0, 0, -101);

    //    {
    //        dJointGroupID jg = dJointGroupCreate (0);
    //        dJointID j = dJointCreateFixed (world, jg);
    //        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(body_sph0.geom));
    //        dJointSetFixed(j);
    //    }

    //    {
    //        dJointGroupID jg = dJointGroupCreate (0);
    //        dJointID j = dJointCreateFixed (world, jg);
    //        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(body_sph1.geom));
    //        dJointSetFixed(j);
    //    }

    //    {
    //        dJointGroupID jg = dJointGroupCreate (0);
    //        dJointID j = dJointCreateFixed (world, jg);
    //        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(body_sph2.geom));
    //        dJointSetFixed(j);
    //    }

    if(leg_count > 0)
        // leg_fl
    {
        dQuaternion q = {M_SQRT1_2, 0, M_SQRT1_2, 0};
        legs[leg_fl] = leg("leg_fl", scnMgr, world, space, -body_width / 2, 0, -body_length / 2, q, -1, 1, 0x000077ff);

        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(legs[leg_fl].first.geom));
        dJointSetFixed(j);
    }

    if(leg_count > 1)
        // leg_fr
    {
        dQuaternion q = {1,0,0,0};
        legs[leg_fr] = leg("leg_fr", scnMgr, world, space, body_width / 2, 0, -body_length / 2, q, 1, 1, 0x777777ff);

        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(legs[leg_fr].first.geom));
        dJointSetFixed(j);
    }

    if(leg_count > 2)
        // leg_rl
    {
        dQuaternion q = {M_SQRT1_2, 0, M_SQRT1_2, 0};
        legs[leg_rl] = leg("leg_rl", scnMgr, world, space, -body_width / 2, 0, body_length / 2, q, -1, -1, 0x777777ff);

        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(legs[leg_rl].first.geom));
        dJointSetFixed(j);
    }

    if(leg_count > 3)
        // leg_rr
    {
        dQuaternion q = {1,0,0,0};
        legs[leg_rr] = leg("leg_rr", scnMgr, world, space, body_width / 2, 0, body_length / 2, q, 1, -1, 0x777777ff);

        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(legs[leg_rr].first.geom));
        dJointSetFixed(j);
    }

    for (int i = 0; i < leg_count; i++)
    {
        colliding_geoms.push_back(legs[i].second.geom);
        colliding_geoms.push_back(legs[i].third.geom);
    }

    brn.reset(new bnn::brain(random_array_length_in_power_of_two,
                             quantity_of_neurons_in_power_of_two,
                             input_length,
                             output_length,
                             4));

    brn_frnd.reset(new bnn::brain_friend(*brn.get()));

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

void creature::set_position(dReal x, dReal y, dReal z)
{
    auto p = dBodyGetPosition (body.body);
    float dx = x - p[0];
    float dy = y - p[1];
    float dz = z - p[2];

    dBodySetPosition (body.body, x, y, z);

    //    auto p0 = dBodyGetPosition (body_sph0.body);
    //    dBodySetPosition (body_sph0.body, p0[0] + dx, p0[1] + dy, p0[2] + dz);

    //    auto p1 = dBodyGetPosition (body_sph1.body);
    //    dBodySetPosition (body_sph1.body, p1[0] + dx, p1[1] + dy, p1[2] + dz);

    //    auto p2 = dBodyGetPosition (body_sph2.body);
    //    dBodySetPosition (body_sph2.body, p2[0] + dx, p2[1] + dy, p2[2] + dz);


    dQuaternion q = {1,0,0,0};
    dQuaternion q_rev = {0,0,1,0};

    for (int i = 0; i < leg_count; i++)
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
    brn->start();
#ifdef learning_creature
    teacher->start();
#endif
}

void creature::step()
{
    float fs;
    float st;

    static float range = 1.0f;

    static const Ogre::Quaternion ort_x(0, 1, 0, 0);
    static const Ogre::Quaternion ort_y(0, 0, 1, 0);
    static const Ogre::Quaternion ort_z(0, 0, 0, 1);

    float x_scalar;
    float y_scalar;
    float z_scalar;

    std::string debug_str;

    body.step();

#ifdef learning_creature
    _word data = teacher->get_data();
#endif

#if(1)
    for(int i = 0; i < leg_count; i++)
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
        data_processing_method->set_inputs(*brn, count_input, distance[i], 1.0f, debug_str);
#ifdef show_debug_data
        if(i % 2)
            debug_str += " ";
#endif
    }

    const dReal* body_q = dBodyGetQuaternion(body.body);
    Ogre::Quaternion body_quat = Ogre::Quaternion(body_q[0], body_q[1], body_q[2], body_q[3]);
    Ogre::Quaternion body_quat_inv = body_quat.Inverse();

    // Relative ort vectors
    Ogre::Quaternion ort_x_rel = body_quat * ort_x * body_quat_inv;
    Ogre::Quaternion ort_y_rel = body_quat * ort_y * body_quat_inv;
    Ogre::Quaternion ort_z_rel = body_quat * ort_z * body_quat_inv;

    {
        // Set inputs by velosity
        auto vel = dBodyGetLinearVel(body.body);

        x_scalar = ort_x_rel[1] * vel[0] + ort_x_rel[2] * vel[1] + ort_x_rel[3] * vel[2];
        y_scalar = ort_y_rel[1] * vel[0] + ort_y_rel[2] * vel[1] + ort_y_rel[3] * vel[2];
        z_scalar = ort_z_rel[1] * vel[0] + ort_z_rel[2] * vel[1] + ort_z_rel[3] * vel[2];
        data_processing_method->set_inputs(*brn, count_input, x_scalar, range, debug_str);
        data_processing_method->set_inputs(*brn, count_input, y_scalar, range, debug_str);
        data_processing_method->set_inputs(*brn, count_input, z_scalar, range, debug_str);
#ifdef show_debug_data
        debug_str += " ";
#endif
    }

    {
        // Set inputs by direction
        x_scalar = ort_x_rel[1] * 1 + ort_x_rel[2] * 0 + ort_x_rel[3] * 0;
        y_scalar = ort_x_rel[1] * 0 + ort_x_rel[2] * 1 + ort_x_rel[3] * 0;
        z_scalar = ort_x_rel[1] * 0 + ort_x_rel[2] * 0 + ort_x_rel[3] * 1;
        data_processing_method->set_inputs(*brn, count_input, x_scalar, range, debug_str);
        data_processing_method->set_inputs(*brn, count_input, y_scalar, range, debug_str);
        data_processing_method->set_inputs(*brn, count_input, z_scalar, range, debug_str);
#ifdef show_debug_data
        debug_str += " ";
#endif

        x_scalar = ort_y_rel[1] * 1 + ort_y_rel[2] * 0 + ort_y_rel[3] * 0;
        y_scalar = ort_y_rel[1] * 0 + ort_y_rel[2] * 1 + ort_y_rel[3] * 0;
        z_scalar = ort_y_rel[1] * 0 + ort_y_rel[2] * 0 + ort_y_rel[3] * 1;
        data_processing_method->set_inputs(*brn, count_input, x_scalar, range, debug_str);
        data_processing_method->set_inputs(*brn, count_input, y_scalar, range, debug_str);
        data_processing_method->set_inputs(*brn, count_input, z_scalar, range, debug_str);
#ifdef show_debug_data
        debug_str += " ";
#endif

        x_scalar = ort_z_rel[1] * 1 + ort_z_rel[2] * 0 + ort_z_rel[3] * 0;
        y_scalar = ort_z_rel[1] * 0 + ort_z_rel[2] * 1 + ort_z_rel[3] * 0;
        z_scalar = ort_z_rel[1] * 0 + ort_z_rel[2] * 0 + ort_z_rel[3] * 1;
        data_processing_method->set_inputs(*brn, count_input, x_scalar, range, debug_str);
        data_processing_method->set_inputs(*brn, count_input, y_scalar, range, debug_str);
        data_processing_method->set_inputs(*brn, count_input, z_scalar, range, debug_str);
    }

#ifdef creature_sees_world
#ifdef show_debug_data
    debug_str += "\n";
#endif
    // I see figures with two eyes
    for_each(input_from_world->begin(), input_from_world->end(), [&](uint32 value)
    {
        for(int i = 0; i < 32; i++)
        {
#ifdef show_debug_data
            debug_str += std::to_string((value >> i) & 1);
#endif

            brn->set_in(count_input++, (value >> i) & 1);
        }

#ifdef show_debug_data
        debug_str += " ";
#endif
    });
#else
#ifdef show_debug_data
    debug_str += " ";
#endif
#endif

    // I can move legs
    for(_word i = 0; i < force_distance_count; i++)
    {
#ifdef show_debug_data
        debug_str += std::to_string(brn->get_out(i * 2)) + std::to_string(brn->get_out(i * 2 + 1));
#endif

        force[i] = static_cast<float>(brn->get_out(i * 2)) - static_cast<float>(brn->get_out(i * 2 + 1));
    }

#ifdef show_debug_data
    std::cout << debug_str << std::endl;
#endif
}

void creature::stop()
{
    brn_frnd->stop();
#ifdef learning_creature
    teacher->stop();
#endif
}
