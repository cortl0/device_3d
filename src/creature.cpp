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
        legs[leg_fl] = creature::leg("leg_fl", scnMgr, world, space, -body_width / 2, 0, -body_length / 2, q, -1, 1);

        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(legs[leg_fl].first.geom));
        dJointSetFixed(j);
    }

    if(leg_count > 1)
        // leg_fr
    {
        dQuaternion q = {1,0,0,0};
        legs[leg_fr] = creature::leg("leg_fr", scnMgr, world, space, body_width / 2, 0, -body_length / 2, q, 1, 1);

        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(legs[leg_fr].first.geom));
        dJointSetFixed(j);
    }

    if(leg_count > 2)
        // leg_rl
    {
        dQuaternion q = {f,0,f,0};
        legs[leg_rl] = creature::leg("leg_rl", scnMgr, world, space, -body_width / 2, 0, body_length / 2, q, -1, -1);

        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(legs[leg_rl].first.geom));
        dJointSetFixed(j);
    }

    if(leg_count > 3)
        // leg_rr
    {
        dQuaternion q = {1,0,0,0};
        legs[leg_rr] = creature::leg("leg_rr", scnMgr, world, space, body_width / 2, 0, body_length / 2, q, 1, -1);

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

    _word random_array_length_in_power_of_two = 27;

    _word random_max_value_in_power_of_two = 31;

    _word quantity_of_neurons_in_power_of_two = 16;

    brn.reset(new bnn::brain(random_array_length_in_power_of_two,
                             random_max_value_in_power_of_two,
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

void creature::start(void* owner, void (*owner_clock_cycle_handler)(void* owner))
{
    this->owner = owner;
    this->owner_clock_cycle_handler = owner_clock_cycle_handler;

    brn->start(this, brain_clock_cycle_handler);
}

void creature::step()
{
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
}

void creature::stop()
{
    brn_frnd->stop();
}

creature::leg::leg(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal x, dReal y, dReal z, dQuaternion q, float dir_lr, float dir_fr)
{
    first = cube(name + "_first", scnMgr, world, space, first_mass, first_x, first_y, first_z);
    second = cube(name + "_second", scnMgr, world, space, second_mass, second_x, second_y, second_z);
    third = cube(name + "_third", scnMgr, world, space, third_mass, third_x, third_y, third_z);

    //    dBodySetPosition (first.body, 0, 0, dir_fr * first_z / 2);
    //dBodySetPosition (first.body, 0, 0, 0);
    dBodySetPosition (second.body, dir_lr * second_x / 2, 0, 0);
    dBodySetPosition (third.body, dir_lr * (second_x + third_x / 2), 0, 0);

    dJointGroupID jg_fs = dJointGroupCreate (0);
    j_fs = dJointCreateHinge (world, jg_fs);
    dJointAttach (j_fs, dGeomGetBody(first.geom), dGeomGetBody(second.geom));
    dJointSetHingeAnchor (j_fs, 0, 0, 0);
    dJointSetHingeAxis (j_fs, 0, 1 * dir_lr, 0);
    dJointSetHingeParam (j_fs,dParamLoStop, -M_PI / 6 );
    dJointSetHingeParam (j_fs,dParamHiStop, M_PI / 6);

    if(1)
    {
        dJointGroupID jg_st = dJointGroupCreate (0);
        j_st = dJointCreateHinge (world, jg_st);
        dJointAttach (j_st, dGeomGetBody(second.geom), dGeomGetBody(third.geom));
        dJointSetHingeAnchor (j_st, dir_lr * second_x, 0, 0);
        dJointSetHingeAxis (j_st, 0, 0, dir_lr * 1);
        dJointSetHingeParam (j_st,dParamLoStop, -M_PI * 1 / 2);
        dJointSetHingeParam (j_st,dParamHiStop, +M_PI * 1 / 2);
    }
    else
    {
        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach (j, dGeomGetBody(second.geom), dGeomGetBody(third.geom));
        dJointSetFixed(j);
    }

    relocate(x, y, z, q);

    first.node->setPosition(0, -60, 0);

    //    dJointSetHinge2Param (j_st,dParamLoStop,-0.1);
    //    dJointSetHinge2Param (j_st,dParamHiStop,+0.1);


    //    dJointGroupID cg_st = dJointGroupCreate (0);
    //    dJointID j_st = dJointCreateHinge (world, cg_st);
    //    dJointAttach (j_st, dGeomGetBody(first.geom), dGeomGetBody(second.geom));
    //    dJointSetHingeAnchor (j_st, 50, 0, 0);
    //    dJointSetHingeAxis (j_st, 0, 0, 1);

}

void creature::leg::relocate(dReal dx, dReal dy, dReal dz, dQuaternion q)
{
    auto p0 = dBodyGetPosition (first.body);
    auto p1 = dBodyGetPosition (second.body);
    auto p2 = dBodyGetPosition (third.body);

    dBodySetPosition (first.body, p0[0] + dx, p0[1] + dy, p0[2] + dz);
    dBodySetPosition (second.body, p1[0] + dx, p1[1] + dy, p1[2] + dz);
    dBodySetPosition (third.body, p2[0] + dx, p2[1] + dy, p2[2] + dz);

    // TODO
    //    dBodySetQuaternion(first.body, q);
    //    dBodySetQuaternion(second.body, q);
    //    dBodySetQuaternion(third.body, q);

    //    Ogre::Vector3 pf(p0[0], p0[1], p0[2]);
    //    Ogre::Vector3 ps(p1[0], p1[1], p1[2]);
    //    Ogre::Vector3 pt(p2[0], p2[1], p2[2]);

    //    Ogre::Vector3 dps(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    //    Ogre::Vector3 dpt(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);

    //    Ogre::Quaternion q(dq[0], dq[1], dq[2], dq[3]);
    //    Ogre::Quaternion q_rez_
    //crtr.hip_fl.geom, crtr.hip_fr.geom, crtr.hip_rl.geom, crtr.hip_rr.geom,s = q * Ogre::Quaternion(0, dps[0], dps[1], dps[2]) * q.Inverse();
    //    Ogre::Quaternion q_rez_t = q * Ogre::Quaternion(0, dpt[0], dpt[1], dpt[2]) * q.Inverse();
    //    ps[0] = p0[0] + q_rez_s[1]; ps[1] = p0[1] + q_rez_s[2]; ps[2] = p0[2] + q_rez_s[3];
    //    pt[0] = p0[0] + q_rez_t[1]; pt[1] = p0[1] + q_rez_t[2]; pt[2] = p0[2] + q_rez_t[3];

    //    dBodySetPosition(first.body, p0[0] + dx, p0[1] + dy, p0[2] + dz);
    //    dBodySetPosition(second.body, ps[0] + dx, ps[1] + dy, ps[2] + dz);
    //    dBodySetPosition(third.body, pt[0] + dx, pt[1] + dy, pt[2] + dz);

    //    dBodySetQuaternion(first.body, dq);
    //    dBodySetQuaternion(second.body, dq);
    //    dBodySetQuaternion(third.body, dq);


    //Ogre::Vector3 v(dx,dy,dz);
    //Ogre::Quaternion p(0, p0[0], p0[1], p0[2]);
    //    Ogre::Quaternion q_rez = q1 * Ogre::Quaternion(0, q[0], q[1], q[2]) * q1.Inverse();
    //    Ogre::Vector3 p_rez(q_rez[1], q_rez[2], q_rez[3]);
    //dBodySetPosition (first.body, p_rez[0] + dx, p_rez[1] + dy, p_rez[2] + dz);
}

void creature::leg::step(float& fs, float& st/*, float& tf*/)
{
    //return;
    fs = fs * (first_z / 2 + second_x / 2);
    st = st * (second_x / 2 + third_x / 2);

    float torque_coef = 500.0f;

    dJointAddHingeTorque(j_fs, fs * torque_coef);
    dJointAddHingeTorque(j_st, st * torque_coef);

    fs = dJointGetHingeAngle (j_fs) / M_PI * 6;
    st = dJointGetHingeAngle (j_st) / M_PI * 2;

    //first.step();
    second.step();
    third.step();
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

const int count_max = 128;
int count_current = count_max;

void creature::brain_clock_cycle_handler(void* me_void)
{
    //std::cout << "1" << std::endl;
    //#define show_debug_data
    void (*set_inputs)(creature*, int&, float , float) = [](creature* cr, int& count, float value, float range)
    {
#ifdef show_debug_data
        std::string s;
#endif
        for(uint8 j = 0; j < bits_in_byte; j++)
        {
            cr->brn->set_in(count++, get_bool(-range, range, value, bits_in_byte, j));
#ifdef show_debug_data
            s += std::to_string((int)get_bool(-k, k, value, bits_in_byte, j));
#endif
        }
#ifdef show_debug_data
        std::cout << s;
#endif
    };

    uint8 u;

    auto me = static_cast<creature*>(me_void);

    int count_input = 0;

    // Set inputs by legs states
    for(int i = 0; i < force_distance_count; i++)
    {
        set_inputs(me, count_input, me->distance[i], 1);
#ifdef show_debug_data
        if(i % 2)
            std::cout << " ";
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
        k = 10;

        // Set inputs by velosity
        x_scalar = ort_x_rel[1] * dbv[0] + ort_x_rel[2] * dbv[1] + ort_x_rel[3] * dbv[2];
        y_scalar = ort_y_rel[1] * dbv[0] + ort_y_rel[2] * dbv[1] + ort_y_rel[3] * dbv[2];
        z_scalar = ort_z_rel[1] * dbv[0] + ort_z_rel[2] * dbv[1] + ort_z_rel[3] * dbv[2];
        set_inputs(me, count_input, x_scalar, k);
        set_inputs(me, count_input, y_scalar, k);
        set_inputs(me, count_input, z_scalar, k);
#ifdef show_debug_data
        std::cout << " ";
#endif
    }

    if(1)
    {
        k = 2;

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
        std::cout << " ";
#endif

        x_scalar = ort_x_rel[1] * 0 + ort_x_rel[2] * 1 + ort_x_rel[3] * 0;
        y_scalar = ort_y_rel[1] * 0 + ort_y_rel[2] * 0 + ort_y_rel[3] * 1;
        z_scalar = ort_z_rel[1] * 1 + ort_z_rel[2] * 0 + ort_z_rel[3] * 0;
        set_inputs(me, count_input, x_scalar, k);
        set_inputs(me, count_input, y_scalar, k);
        set_inputs(me, count_input, z_scalar, k);
#ifdef show_debug_data
        std::cout << " ";
#endif

        x_scalar = ort_x_rel[1] * 0 + ort_x_rel[2] * 0 + ort_x_rel[3] * 1;
        y_scalar = ort_y_rel[1] * 1 + ort_y_rel[2] * 0 + ort_y_rel[3] * 0;
        z_scalar = ort_z_rel[1] * 0 + ort_z_rel[2] * 1 + ort_z_rel[3] * 0;
        set_inputs(me, count_input, x_scalar, k);
        set_inputs(me, count_input, y_scalar, k);
        set_inputs(me, count_input, z_scalar, k);
#ifdef show_debug_data
        std::cout << " ";
#endif
    }
#ifdef show_debug_data
    //std::cout << std::to_string(own->input_length) << std::endl;
    std::cout << std::endl;
#endif
    {

//        for_each(me->input_from_world->begin(), me->input_from_world->end(), [&](uint32 value)
//        {
//            for(int j = 0; j < 32; j++)
//                me->brn->set_in(count_input++, (value >> j) & 1);
//        });
    }

    int count_output = 0;
    for(int i = 0; i < force_distance_count; i++)
    {
        int8 u = 0;
        for(int j = 0; j < 2; j++)
            u |= (me->brn->get_out(count_output++) << j);
        //std::cout << std::to_string(i) << " = " << std::to_string(u) << std::endl;

        me->force[i] = (float)me->brn->get_out(count_output - 1) - (float)me->brn->get_out(count_output - 2);
    }

    if(--count_current % count_max == 0)
    {
        std::cout << std::to_string(me->brn->iteration)
                  << " = "
                  << std::to_string(me->brn->quantity_of_neurons_in_power_of_two)
                  << " = "
                  << std::to_string(me->brn->quantity_of_initialized_neurons_binary)
                  << " = "
                  << std::to_string(me->brn->rndm->debug_count_put)
                  << " = "
                  << std::to_string(me->brn->rndm->debug_count_get)
                     //                  << " body\t"
                     //                  << std::to_string((int)dBodyGetPosition(own->body.body)[0])
                     //                << "\t"
                     //                << std::to_string((int)dBodyGetPosition(own->body.body)[1])
                     //                << "\t"
                     //                << std::to_string((int)dBodyGetPosition(own->body.body)[2])
                  << std::endl;

        count_current = count_max;
    }

    me->owner_clock_cycle_handler(me->owner);
}
