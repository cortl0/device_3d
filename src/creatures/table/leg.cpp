/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "leg.h"

namespace pho = bnn_device_3d::physical_objects;

namespace bnn_device_3d::creatures::table
{

leg::leg(std::string name, Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space,
         dReal x, dReal y, dReal z,
         dQuaternion q, float direction, uint32_t color)
{
    first = pho::sphere(name + "_first", scnMgr, world, space, first_mass, first_r);
    second = pho::cube(name + "_second", scnMgr, world, space, second_mass, second_x, second_y, second_z);
    third = pho::cube(name + "_third", scnMgr, world, space, third_mass, third_x, third_y, third_z);
    foot = pho::sphere(name + "_foot", scnMgr, world, space, knee_mass, knee_r);
    knee = pho::sphere(name + "_knee", scnMgr, world, space, knee_mass, knee_r);

    first.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, color));
    second.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));
    third.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));
    foot.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, color));
    knee.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, color));

    //    dBodySetPosition (first.body, 0, 0, dir_fr * first_z / 2);
    //dBodySetPosition (first.body, 0, 0, 0);
    dBodySetPosition(second.body, direction * second_x / 2, 0, 0);
    dBodySetPosition(third.body, direction * (second_x + third_x / 2), 0, 0);
    dBodySetPosition(foot.body, direction * (second_x + third_x), 0, 0);
    dBodySetPosition(knee.body, direction * (second_x), 0, 0);

    joints.push_back(joint(world, dGeomGetBody(first.geom), dGeomGetBody(second.geom),
                           1 * direction, 0, 0,
                           0, 0,
                           -M_PI / 12, M_PI / 12,
                           LEG_FIRST_JOINT_TORQUE_COEFFICENT));

    joints.push_back(joint(world, dGeomGetBody(second.geom), dGeomGetBody(third.geom),
                           0, 1 * direction, direction * second_x,
                           0, 0,
                           //+M_PI * 1 / 6, +M_PI * 2 / 6,
                           +M_PI * -1 / 6, +M_PI * 2 / 6,
                           LEG_SECOND_JOINT_TORQUE_COEFFICENT));

    if(0)
    {
        dJointID j_st = dJointCreateFixed (world, jg_st);
        dJointAttach (j_st, dGeomGetBody(second.geom), dGeomGetBody(third.geom));
        dJointSetFixed(j_st);
    }

    auto make_fixed_joint = [&](dGeomID g1, dGeomID g2)
    {
        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach(j, dGeomGetBody(g1), dGeomGetBody(g2));
        dJointSetFixed(j);
    };

    make_fixed_joint(third.geom, foot.geom);

    make_fixed_joint(second.geom, knee.geom);

#ifdef creature_legs_knees_is_blocked
    SetHingeParams(-M_PI / 24, M_PI / 24, +M_PI * 1 / 5, +M_PI * 1 / 5);
#endif

    relocate(x, y, z, q);

    //    dJointSetHinge2Param (j_st,dParamLoStop,-0.1);
    //    dJointSetHinge2Param (j_st,dParamHiStop,+0.1);


    //    dJointGroupID cg_st = dJointGroupCreate (0);
    //    dJointID j_st = dJointCreateHinge (world, cg_st);
    //    dJointAttach (j_st, dGeomGetBody(first.geom), dGeomGetBody(second.geom));
    //    dJointSetHingeAnchor (j_st, 50, 0, 0);
    //    dJointSetHingeAxis (j_st, 0, 0, 1);

}

std::vector<physical_objects::figure*> leg::get_figures()
{
    std::vector<physical_objects::figure*> value;

    value.push_back(&first);
    value.push_back(&second);
    value.push_back(&third);
    value.push_back(&foot);
    value.push_back(&knee);

    return value;
}

void leg::relocate(dReal dx, dReal dy, dReal dz, dQuaternion q)
{
    auto p0 = dBodyGetPosition(first.body);
    auto p1 = dBodyGetPosition(second.body);
    auto p2 = dBodyGetPosition(third.body);

    dBodySetPosition(first.body, p0[0] + dx, p0[1] + dy, p0[2] + dz);
    dBodySetPosition(second.body, p1[0] + dx, p1[1] + dy, p1[2] + dz);
    dBodySetPosition(third.body, p2[0] + dx, p2[1] + dy, p2[2] + dz);
    dBodySetPosition(foot.body, p2[0] + dx, p2[1] + dy, p2[2] + dz);
    dBodySetPosition(knee.body, p2[0] + dx, p2[1] + dy, p2[2] + dz);

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

void leg::step(double& fs, double& st)
{
    fs = fs * (first_r / 2 + second_x / 2);
    st = st * (second_x / 2 + third_x / 2);

    joints[FIRST_JOINT].set_torque(fs);

#ifndef creature_legs_knees_is_blocked
    joints[SECOND_JOINT].set_torque(st);
#endif

    fs = joints[FIRST_JOINT].get_angle();
    st = joints[SECOND_JOINT].get_angle();

    first.step();
    second.step();
    third.step();
    foot.step();
    knee.step();
}

} // namespace bnn_device_3d::creatures::table
