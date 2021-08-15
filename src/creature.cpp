/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "creature.h"

creature::creature(Ogre::SceneManager* scnMgr, dWorldID world, std::shared_ptr<std::vector<uint32>> input_from_world)
    : input_from_world(input_from_world)
{
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
        dQuaternion q = {f,0,f,0};
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
        dQuaternion q = {f,0,f,0};
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
                             output_length));

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
}

bool get_bool(float from, float to, float value, int levels_number, int level)
{
#if(0)
    if(value <= from && level == 0)
        return true;

    if(value >= to && level == levels_number - 1)
        return true;

    to -= from;
    value -= from;
    from = 0;

    float level_size = to / levels_number;

    float l_low = level_size * level;

    float l_high = l_low + level_size;

    if(value >= l_low && value <= l_high)
        return true;

    return false;
#else
    to -= from;
    value -= from;
    from = 0;

    float level_size = to / levels_number;

    float l_low = level_size * level;

    if(value >= l_low)
        return true;

    return false;
#endif
}

//#define show_debug_data
#ifdef show_debug_data
static std::string s;
#endif

void set_inputs(creature* cr, _word& count, float value, float range)
{
#ifdef show_debug_data
    std::string str;
#endif
    for(uint8 j = 0; j < bits_in_byte; j++)
    {
        cr->brn->set_in(count++, get_bool(-range, range, value, bits_in_byte, j));
#ifdef show_debug_data
        str += std::to_string(get_bool(-range, range, value, bits_in_byte, j));
#endif
    }
#ifdef show_debug_data
    s += str;
#endif
};

void creature::step()
{
#ifdef show_debug_data
    s = "";
#endif
    body.step();

    float a;
    float b;

#if(1)
    for(int i = 0; i < leg_count; i++)
    {
        a = force[i * 2 + 0];
        b = force[i * 2 + 1];
        legs[i].step(a, b);
        distance[i * 2 + 0] = a;
        distance[i * 2 + 1] = b;
    }
#else
    for(int i = 0; i < leg_count; i++)
    {
        a = ((float)rand() / RAND_MAX) * 2 - 1;
        b = ((float)rand() / RAND_MAX) * 2 - 1;
        legs[i].step(a, b);
    }
#endif

    auto me = this;

    _word count_input = 0;

    // Set inputs by legs states
    for(int i = 0; i < force_distance_count; i++)
    {
        set_inputs(me, count_input, me->distance[i], 1.0f);
#ifdef show_debug_data
        if(i % 2)
            s += " ";
#endif
    }

    static const Ogre::Quaternion ort_x(0, 1, 0, 0);
    static const Ogre::Quaternion ort_y(0, 0, 1, 0);
    static const Ogre::Quaternion ort_z(0, 0, 0, 1);

    static Ogre::Quaternion ort_x_rel;
    static Ogre::Quaternion ort_y_rel;
    static Ogre::Quaternion ort_z_rel;

    static Ogre::Quaternion dbr_rot;
    static Ogre::Quaternion dbr_rot_inv;

    static float x_scalar;
    static float y_scalar;
    static float z_scalar;

    auto dbr = dBodyGetQuaternion(me->body.body);

    auto dbv = dBodyGetLinearVel(me->body.body);

    dbr_rot = Ogre::Quaternion(dbr[0], dbr[1], dbr[2], dbr[3]);
    dbr_rot_inv = dbr_rot.Inverse();

    // Relative ort vectors
    ort_x_rel = dbr_rot * ort_x * dbr_rot_inv;
    ort_y_rel = dbr_rot * ort_y * dbr_rot_inv;
    ort_z_rel = dbr_rot * ort_z * dbr_rot_inv;

    static float k;

    {
        k = 1.0f;

        // Set inputs by velosity
        x_scalar = ort_x_rel[1] * dbv[0] + ort_x_rel[2] * dbv[1] + ort_x_rel[3] * dbv[2];
        y_scalar = ort_y_rel[1] * dbv[0] + ort_y_rel[2] * dbv[1] + ort_y_rel[3] * dbv[2];
        z_scalar = ort_z_rel[1] * dbv[0] + ort_z_rel[2] * dbv[1] + ort_z_rel[3] * dbv[2];
        set_inputs(me, count_input, x_scalar, k);
        set_inputs(me, count_input, y_scalar, k);
        set_inputs(me, count_input, z_scalar, k);
#ifdef show_debug_data
        s += " ";
#endif
    }

    if(1)
    {
        k = 1.0f;

        auto dbr_vel = dBodyGetAngularVel(me->body.body);

        auto dbr_rot = Ogre::Quaternion(dbr_vel[0], dbr_vel[1], dbr_vel[2], dbr_vel[3]);
        auto dbr_rot_inv = dbr_rot.Inverse();

        auto ort_x_rel = dbr_rot * ort_x * dbr_rot_inv;
        auto ort_y_rel = dbr_rot * ort_y * dbr_rot_inv;
        auto ort_z_rel = dbr_rot * ort_z * dbr_rot_inv;

        // Set inputs by direction
        x_scalar = ort_x_rel[1] * 1 + ort_x_rel[2] * 0 + ort_x_rel[3] * 0;
        y_scalar = ort_y_rel[1] * 0 + ort_y_rel[2] * 1 + ort_y_rel[3] * 0;
        z_scalar = ort_z_rel[1] * 0 + ort_z_rel[2] * 0 + ort_z_rel[3] * 1;
        set_inputs(me, count_input, x_scalar, k);
        set_inputs(me, count_input, y_scalar, k);
        set_inputs(me, count_input, z_scalar, k);
#ifdef show_debug_data
        s += " ";
#endif

        x_scalar = ort_x_rel[1] * 0 + ort_x_rel[2] * 1 + ort_x_rel[3] * 0;
        y_scalar = ort_y_rel[1] * 0 + ort_y_rel[2] * 0 + ort_y_rel[3] * 1;
        z_scalar = ort_z_rel[1] * 1 + ort_z_rel[2] * 0 + ort_z_rel[3] * 0;
        set_inputs(me, count_input, x_scalar, k);
        set_inputs(me, count_input, y_scalar, k);
        set_inputs(me, count_input, z_scalar, k);
#ifdef show_debug_data
        s += " ";
#endif

        x_scalar = ort_x_rel[1] * 0 + ort_x_rel[2] * 0 + ort_x_rel[3] * 1;
        y_scalar = ort_y_rel[1] * 1 + ort_y_rel[2] * 0 + ort_y_rel[3] * 0;
        z_scalar = ort_z_rel[1] * 0 + ort_z_rel[2] * 1 + ort_z_rel[3] * 0;
        set_inputs(me, count_input, x_scalar, k);
        set_inputs(me, count_input, y_scalar, k);
        set_inputs(me, count_input, z_scalar, k);
    }

#ifdef creature_sees_world
#ifdef show_debug_data
    s += "\n";
#endif
    // I see three shapes at two coordinates
    for_each(me->input_from_world->begin(), me->input_from_world->end(), [&](uint32 value)
    {
        for(int i = 0; i < 32; i++)
        {
#ifdef show_debug_data
            s += std::to_string((value >> i) & 1);
#endif

            me->brn->set_in(count_input++, (value >> i) & 1);
        }

#ifdef show_debug_data
        s += " ";
#endif
    });
#else
#ifdef show_debug_data
    s += " ";
#endif
#endif

    // I can move legs
    for(_word i = 0; i < force_distance_count; i++)
    {
#ifdef show_debug_data
        s += std::to_string(me->brn->get_out(i * 2)) + std::to_string(me->brn->get_out(i * 2 + 1));
#endif

        me->force[i] = static_cast<float>(me->brn->get_out(i * 2)) - static_cast<float>(me->brn->get_out(i * 2 + 1));
    }

#ifdef show_debug_data
    std::cout << s << std::endl;
#endif
}

void creature::stop()
{
    brn_frnd->stop();
}
