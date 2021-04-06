#include "creature.h"

creature::creature(Ogre::SceneManager* scnMgr, dWorldID world, std::shared_ptr<std::vector<uint32>> input_from_world)
    : input_from_world(input_from_world)
{
    for(int i = 0; i < for_dis_count; i++)
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


    //    if(leg_count > 2)
    //        // leg_ml
    //    {
    //        dQuaternion q = {f,0,f,0};
    //        legs[leg_ml] = creature::leg("leg_ml", scnMgr, world, space, -body_width / 2, 0, 0, q, -1, 1);

    //        dJointGroupID jg = dJointGroupCreate (0);
    //        dJointID j = dJointCreateFixed (world, jg);
    //        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(legs[leg_ml].first.geom));
    //        dJointSetFixed(j);
    //    }

    //    if(leg_count > 3)
    //        // leg_mr
    //    {
    //        dQuaternion q = {1,0,0,0};
    //        legs[leg_mr] = creature::leg("leg_mr", scnMgr, world, space, body_width / 2, 0, 0, q, 1, -1);

    //        dJointGroupID jg = dJointGroupCreate (0);
    //        dJointID j = dJointCreateFixed (world, jg);
    //        dJointAttach (j, dGeomGetBody(body.geom), dGeomGetBody(legs[leg_mr].first.geom));
    //        dJointSetFixed(j);
    //    }

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
        //colliding_geoms.push_back(legs[i].fourth.geom);
    }

    _word random_array_length_in_power_of_two = 24;
    _word random_max_value_in_power_of_two = 31;
    _word quantity_of_neurons_in_power_of_two = 16;

    brn.reset(new bnn::brain(random_array_length_in_power_of_two,
                             random_max_value_in_power_of_two,
                             quantity_of_neurons_in_power_of_two,
                             input_length,
                             output_length,
                             brain_clock_cycle_handler));

    brn_frnd.reset(new bnn::brain_friend(*brn.get()));

    //brn->start(this);

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
    //    legs[leg_ml].relocate(dx, dy, dz, q_rev);
    //    legs[leg_mr].relocate(dx, dy, dz, q);
    //    legs[leg_rl].relocate(dx, dy, dz, q_rev);
    //    legs[leg_rr].relocate(dx, dy, dz, q);
}

//std::mutex mtx;

void creature::step()
{
    body.step();
    //    body_sph0.step();
    //    body_sph1.step();
    //    body_sph2.step();

    //    float coef_a;
    //    float coef_b;
    float a;
    float b;


#if(1)
    for(int i = 0; i < leg_count; i++)
    {
        a = force[i * 3 + 0];
        b = force[i * 3 + 1];
        legs[i].step(a, b);
        distance[i * 3 + 0] = a;
        distance[i * 3 + 1] = b;
    }

    if(ft)
    {
        brn->start(this);
        ft = false;
    }
#else
    float coef = 0.01f;

    a = ((float)rand() / RAND_MAX * 255 - 128) * coef;
    b = ((float)rand() / RAND_MAX * 255 - 128) * coef;
    legs[leg_fl].step(a, b);
    a = ((float)rand() / RAND_MAX * 255 - 128) * coef;
    b = ((float)rand() / RAND_MAX * 255 - 128) * coef;
    legs[leg_fr].step(a, b);
    a = ((float)rand() / RAND_MAX * 255 - 128) * coef;
    b = ((float)rand() / RAND_MAX * 255 - 128) * coef;
//    legs[leg_ml].step(a, b);
//    a = ((float)rand() / RAND_MAX * 255 - 128) * coef;
//    b = ((float)rand() / RAND_MAX * 255 - 128) * coef;
//    legs[leg_mr].step(a, b);
//    a = ((float)rand() / RAND_MAX * 255 - 128) * coef;
//    b = ((float)rand() / RAND_MAX * 255 - 128) * coef;
    legs[leg_rl].step(a, b);
    a = ((float)rand() / RAND_MAX * 255 - 128) * coef;
    b = ((float)rand() / RAND_MAX * 255 - 128) * coef;
    legs[leg_rr].step(a, b);

    if(ft)
    {
        brn->start(this);
        ft = false;
    }
#endif
}

creature::leg::leg(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal x, dReal y, dReal z, dQuaternion q, float dir_lr, float dir_fr)
{
    first = cube(name + "_first", scnMgr, world, space, first_mass, first_x, first_y, first_z);
    second = cube(name + "_second", scnMgr, world, space, second_mass, second_x, second_y, second_z);
    third = cube(name + "_third", scnMgr, world, space, third_mass, third_x, third_y, third_z);
    //fourth = cube(name + "_fourth", scnMgr, world, space, fourth_mass, fourth_x, fourth_y, fourth_z);
    //fifth = sphere(name + "_fifth", scnMgr, world, space, fifth_mass, fifth_r);

    //    dBodySetPosition (first.body, 0, 0, dir_fr * first_z / 2);
    //dBodySetPosition (first.body, 0, 0, 0);
    dBodySetPosition (second.body, dir_lr * second_x / 2, 0, 0);
    dBodySetPosition (third.body, dir_lr * (second_x + third_x / 2), 0, 0);
    //dBodySetPosition (fourth.body, dir_lr * (second_x + third_x + fourth_x / 2), 0, 0);
    //dBodySetPosition (fifth.body, dir * (second_x + third_x + fourth_x), 0, 0);

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

//    {
//        dJointGroupID jg_tf = dJointGroupCreate (0);
//        j_tf = dJointCreateHinge (world, jg_tf);
//        dJointAttach (j_tf, dGeomGetBody(third.geom), dGeomGetBody(fourth.geom));
//        dJointSetHingeAnchor (j_tf, dir_lr * (second_x + third_x), 0, 0);
//        dJointSetHingeAxis (j_tf, 0, 0, dir_lr * 1);
//        dJointSetHingeParam (j_tf,dParamLoStop, M_PI * 1 / 12);
//        dJointSetHingeParam (j_tf,dParamHiStop, M_PI * 10 / 12);
//    }

    //dJointAddHingeTorque(j_tf, 5);

    //    {
    //        dJointGroupID jg_tf1 = dJointGroupCreate (0);
    //        dJointID j_tf1 = dJointCreateFixed(world, jg_tf1);
    //        dJointAttach (j_tf1, dGeomGetBody(fourth.geom), dGeomGetBody(fifth.geom));
    //        dJointSetFixed (j_tf1);
    //    }

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
    //auto p3 = dBodyGetPosition (fourth.body);
    //auto p4 = dBodyGetPosition (fifth.body);

    dBodySetPosition (first.body, p0[0] + dx, p0[1] + dy, p0[2] + dz);
    dBodySetPosition (second.body, p1[0] + dx, p1[1] + dy, p1[2] + dz);
    dBodySetPosition (third.body, p2[0] + dx, p2[1] + dy, p2[2] + dz);
    //dBodySetPosition (fourth.body, p3[0] + dx, p3[1] + dy, p3[2] + dz);
    //dBodySetPosition (fifth.body, p3[0] + dx, p3[1] + dy, p3[2] + dz);

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
    fs = fs * (first_z / 2 + second_x / 2);
    st = st * (second_x / 2 + third_x / 2);
    //tf = tf * (third_x / 2 + fourth_x / 2);

    float torque_coef = 500.0f;

    dJointAddHingeTorque(j_fs, fs * torque_coef);
    dJointAddHingeTorque(j_st, st * torque_coef);
    //dJointAddHingeTorque(j_tf, tf * torque_coef);

    fs = dJointGetHingeAngle (j_fs) / M_PI;
    st = dJointGetHingeAngle (j_st) / M_PI;
    //tf = dJointGetHingeAngle (j_tf) / M_PI;

    //first.step();
    second.step();
    third.step();
    //fourth.step();
    //fifth.step();
}

const int cccc = 128;
int ccc = cccc;

void creature::brain_clock_cycle_handler(void* owner)
{
    //mtx.lock();


    auto own = static_cast<creature*>(owner);

    while (own->cycle);

    int count_input = 0;
    for(int i = 0; i < for_dis_count; i++)
    {
        uint8 u = (uint8)(own->distance[i] * 128 + 128);
        //std::cout << std::to_string(i) << " = " << std::to_string(own->distance[i]) << std::endl;
        for(int j = 0; j < 8; j++)
            own->brn->set_in(count_input++, (u >> j) & 1);
    }


    {
        {
            auto pos_body = dGeomGetPosition(own->body.geom);
            uint8 u = (uint8)(pos_body[0]);
            //std::cout << std::to_string(i) << " = " << std::to_string(own->distance[i]) << std::endl;
            for(int j = 0; j < 8; j++)
                own->brn->set_in(count_input++, (u >> j) & 1);

            u = (uint8)(pos_body[1]);
            //std::cout << std::to_string(i) << " = " << std::to_string(own->distance[i]) << std::endl;
            for(int j = 0; j < 8; j++)
                own->brn->set_in(count_input++, (u >> j) & 1);

            u = (uint8)(pos_body[2]);
            //std::cout << std::to_string(i) << " = " << std::to_string(own->distance[i]) << std::endl;
            for(int j = 0; j < 8; j++)
                own->brn->set_in(count_input++, (u >> j) & 1);
        }

        {
            auto pos = dGeomGetPosition(own->legs[leg_fl].first.geom);
            uint8 u = (uint8)(pos[0]);
            //std::cout << std::to_string(i) << " = " << std::to_string(own->distance[i]) << std::endl;
            for(int j = 0; j < 8; j++)
                own->brn->set_in(count_input++, (u >> j) & 1);

            u = (uint8)(pos[1]);
            //std::cout << std::to_string(i) << " = " << std::to_string(own->distance[i]) << std::endl;
            for(int j = 0; j < 8; j++)
                own->brn->set_in(count_input++, (u >> j) & 1);

            u = (uint8)(pos[2]);
            //std::cout << std::to_string(i) << " = " << std::to_string(own->distance[i]) << std::endl;
            for(int j = 0; j < 8; j++)
                own->brn->set_in(count_input++, (u >> j) & 1);
        }

        {
            auto pos = dGeomGetPosition(own->legs[leg_fr].first.geom);
            uint8 u = (uint8)(pos[0]);
            //std::cout << std::to_string(i) << " = " << std::to_string(own->distance[i]) << std::endl;
            for(int j = 0; j < 8; j++)
                own->brn->set_in(count_input++, (u >> j) & 1);

            u = (uint8)(pos[1]);
            //std::cout << std::to_string(i) << " = " << std::to_string(own->distance[i]) << std::endl;
            for(int j = 0; j < 8; j++)
                own->brn->set_in(count_input++, (u >> j) & 1);

            u = (uint8)(pos[2]);
            //std::cout << std::to_string(i) << " = " << std::to_string(own->distance[i]) << std::endl;
            for(int j = 0; j < 8; j++)
                own->brn->set_in(count_input++, (u >> j) & 1);
        }

        {
            for_each(own->input_from_world->begin(), own->input_from_world->end(), [&](uint32 value)
            {
                for(int j = 0; j < 32; j++)
                    own->brn->set_in(count_input++, (value >> j) & 1);
            });
        }
    }

    int count_output = 0;
    for(int i = 0; i < for_dis_count; i++)
    {
        int8 u = 0;
        for(int j = 0; j < 8; j++)
            u |= (own->brn->get_out(count_output++) << j);
        //std::cout << std::to_string(i) << " = " << std::to_string(u) << std::endl;


        own->force[i] = (float)u / 128;
    }


    //mtx.unlock();

    if(--ccc % cccc == 0)
    {
        std::cout << std::to_string(own->brn->iteration)
                  << " = "
                  << std::to_string(own->brn->quantity_of_neurons_in_power_of_two)
                  << " = "
                  << std::to_string(own->brn->quantity_of_initialized_neurons_binary)
                     //                  << " body\t"
                     //                  << std::to_string((int)dBodyGetPosition(own->body.body)[0])
                     //                << "\t"
                     //                << std::to_string((int)dBodyGetPosition(own->body.body)[1])
                     //                << "\t"
                     //                << std::to_string((int)dBodyGetPosition(own->body.body)[2])
                  << std::endl;


        //        for (int i = 0; i<6; i++)
        //        {
        //            std::cout << "leg=" << std::to_string(i)
        //                      << "\t"
        //                      << std::to_string((int)dBodyGetPosition(own->legs[i].first.body)[0])
        //                    << "\t"
        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].first.body)[1])
        //                    << "\t"
        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].first.body)[2])
        //                    << "|\t"

        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].second.body)[0])
        //                    << "\t"
        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].second.body)[1])
        //                    << "\t"
        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].second.body)[2])
        //                    << "|\t"

        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].third.body)[0])
        //                    << "\t"
        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].third.body)[1])
        //                    << "\t"
        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].third.body)[2])
        //                    << "|\t"

        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].fourth.body)[0])
        //                    << "\t"
        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].fourth.body)[1])
        //                    << "\t"
        //                    << std::to_string((int)dBodyGetPosition(own->legs[i].fourth.body)[2])
        //                    << "|\t"

        //                    << std::endl;
        //            std::cout << "leg=" << std::to_string(i)
        //                      << "\t"
        //                      << std::to_string((int)own->legs[i].first.node->getPosition()[0])
        //                    << "\t"
        //                    << std::to_string((int)own->legs[i].first.node->getPosition()[1])
        //                    << "\t"
        //                    << std::to_string((int)own->legs[i].first.node->getPosition()[2])
        //                    << "|\t"

        //                    << std::to_string((int)own->legs[i].second.node->getPosition()[0])
        //                    << "\t"
        //                    << std::to_string((int)own->legs[i].second.node->getPosition()[1])
        //                    << "\t"
        //                    << std::to_string((int)own->legs[i].second.node->getPosition()[2])
        //                    << "|\t"

        //                    << std::to_string((int)own->legs[i].third.node->getPosition()[0])
        //                    << "\t"
        //                    << std::to_string((int)own->legs[i].third.node->getPosition()[1])
        //                    << "\t"
        //                    << std::to_string((int)own->legs[i].third.node->getPosition()[2])
        //                    << "|\t"

        //                    << std::to_string((int)own->legs[i].fourth.node->getPosition()[0])
        //                    << "\t"
        //                    << std::to_string((int)own->legs[i].fourth.node->getPosition()[1])
        //                    << "\t"
        //                    << std::to_string((int)own->legs[i].fourth.node->getPosition()[2])
        //                    << "|\t"

        //                    << std::endl;






        //        }

        ccc = cccc;
    }

    own->cycle = true;
}
