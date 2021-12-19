/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "leg.h"

leg::leg(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal x, dReal y, dReal z, dQuaternion q, float direction, uint32 color)
{
    first = cube(name + "_first", scnMgr, world, space, first_mass, first_x, first_y, first_z);
    second = cube(name + "_second", scnMgr, world, space, second_mass, second_x, second_y, second_z);
    third = cube(name + "_third", scnMgr, world, space, third_mass, third_x, third_y, third_z);

    first.set_material(figure::create_material_chess(128, 32, color, 0x333333ff));
    second.set_material(figure::create_material_chess(128, 32, 0x777777ff, 0x333333ff));
    third.set_material(figure::create_material_chess(128, 32, 0x777777ff, 0x333333ff));

    //    dBodySetPosition (first.body, 0, 0, dir_fr * first_z / 2);
    //dBodySetPosition (first.body, 0, 0, 0);
    dBodySetPosition (second.body, direction * second_x / 2, 0, 0);
    dBodySetPosition (third.body, direction * (second_x + third_x / 2), 0, 0);

    joint_fs.reset(new joint(world, dGeomGetBody(first.geom), dGeomGetBody(second.geom), 1 * direction, 0, 0, 0, 0));

    joint_st.reset(new joint(world, dGeomGetBody(second.geom), dGeomGetBody(third.geom), 0, 1 * direction, direction * second_x, 0, 0));

    if(0)
    {
        dJointID j_st = dJointCreateFixed (world, jg_st);
        dJointAttach (j_st, dGeomGetBody(second.geom), dGeomGetBody(third.geom));
        dJointSetFixed(j_st);
    }

    //SetHingeParams(-M_PI / 6, M_PI / 6, -M_PI * 1 / 2, +M_PI * 1 / 2);
    //SetHingeParams(-M_PI / 24, M_PI / 24, +M_PI * 1 / 4, +M_PI * 1 / 3);
    set_joints_prams(-M_PI / 24, M_PI / 24, +M_PI * 1 / 6, +M_PI * 1 / 5);

#ifdef creature_legs_knees_is_blocked
    SetHingeParams(-M_PI / 24, M_PI / 24, +M_PI * 1 / 5, +M_PI * 1 / 5);
#endif

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

void leg::set_joints_prams(float first_start_angle, float first_end_angle, float second_start_angle, float second_end_angle)
{
    joint_fs->set_params(first_start_angle, first_end_angle);
    joint_st->set_params(second_start_angle, second_end_angle);
}

void leg::relocate(dReal dx, dReal dy, dReal dz, dQuaternion q)
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

float value_in_range(const float& value, const float& range_start, const float& range_end)
{
    static const float default_range_start = -1;
    static const float default_range_end = 1;
    static const float default_range_delta = default_range_end - default_range_start;

    return default_range_start + ((value - range_start) / (range_end - range_start)) * default_range_delta;
}

void leg::step(float& fs, float& st)
{
    fs = fs * (first_z / 2 + second_x / 2);
    st = st * (second_x / 2 + third_x / 2);

    dJointAddHingeTorque(joint_fs->joint_id, fs * LEG_FIRST_JOINT_TORQUE_COEFFICENT / (1 + abs(dJointGetHingeAngleRate (joint_fs->joint_id))));

#ifndef creature_legs_knees_is_blocked
    dJointAddHingeTorque(joint_st->joint_id, st * LEG_SECOND_JOINT_TORQUE_COEFFICENT / (1 + abs(dJointGetHingeAngleRate (joint_st->joint_id))));
#endif

    fs = value_in_range(dJointGetHingeAngle (joint_fs->joint_id), joint_fs->angle_start, joint_fs->angle_end);
    st = value_in_range(dJointGetHingeAngle (joint_st->joint_id), joint_st->angle_start, joint_st->angle_end);

    first.step();
    second.step();
    third.step();
}
