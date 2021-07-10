/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "tripod.h"

tripod::tripod(dWorldID world, Ogre::SceneNode* cam_node, dBodyID target)
    : target(target), cam_node(cam_node)
{
    cam_dz = cam_node->getPosition()[1] - dBodyGetPosition(target)[1];

    {
        detector = dBodyCreate(world);
        dMass m;
        dMassSetSphereTotal(&m, 1, 1);
        dMassAdjust (&m, 1);
        dBodySetMass(detector, &m);
        dBodySetPosition(detector, dBodyGetPosition(target)[0], dBodyGetPosition(target)[1], dBodyGetPosition(target)[2]);
    }

    {
        upper_detector = dBodyCreate(world);
        dMass m;
        dMassSetSphereTotal(&m, 1, 1);
        dMassAdjust (&m, 1);
        dBodySetMass(upper_detector, &m);
        dBodySetPosition(upper_detector, dBodyGetPosition(target)[0], cam_node->getPosition()[1], dBodyGetPosition(target)[2]);
    }

    {
        cam_base = dBodyCreate(world);
        dMass m;
        dMassSetSphereTotal(&m, 1, 1);
        dMassAdjust (&m, 2);
        dBodySetMass(cam_base, &m);
        dBodySetPosition(cam_base, cam_node->getPosition()[0], cam_node->getPosition()[1], cam_node->getPosition()[2]);
    }

    cam_node->lookAt(Ogre::Vector3(dBodyGetPosition(detector)[0],dBodyGetPosition(detector)[1],dBodyGetPosition(detector)[2]), Ogre::Node::TS_PARENT);
    dQuaternion q = {cam_node->getOrientation().w,
                cam_node->getOrientation().x,
                cam_node->getOrientation().y,
                cam_node->getOrientation().z};
    dBodySetQuaternion(cam_base, q);

    {
        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed(world, jg);
        dJointAttach (j, cam_base, detector);
        dJointSetFixed(j);
    }

    {
        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed(world, jg);
        dJointAttach (j, detector, upper_detector);
        dJointSetFixed(j);
    }
}

void tripod::step()
{
    dBodyAddForce(detector, 0, gravity, 0);
    dBodyAddForce(upper_detector, 0, gravity, 0);
    dBodyAddForce(cam_base, 0, gravity, 0);

    tar = dBodyGetPosition(target);

    look = dBodyGetPosition(detector);
    look_vel = dBodyGetLinearVel(detector);
    dBodySetLinearVel(detector, look_vel[0]/2, look_vel[1]/2, look_vel[2]/2);

    look_up = dBodyGetPosition(upper_detector);
    look_up_vel = dBodyGetLinearVel(detector);
    dBodySetLinearVel(detector, look_up_vel[0]/2, look_up_vel[1]/2, look_up_vel[2]/2);

    bas = dBodyGetPosition(cam_base);
    bas_vel = dBodyGetLinearVel(cam_base);
    dBodySetLinearVel(cam_base, bas_vel[0]/2, bas_vel[1]/2, bas_vel[2]/2);

    dx = (tar[0] - look[0]);
    dy = (tar[1] - look[1]);
    dz = (tar[2] - look[2]);

    if((abs(dx) - velosity_ignore_coef > 0) || (abs(dy) - velosity_ignore_coef > 0) || (abs(dz) - velosity_ignore_coef > 0))
        dBodyAddForce(detector, dx * force_coef, dy * force_coef, dz * force_coef);

    dBodySetPosition(upper_detector, look[0], look[1] + cam_dz, look[2]);

    dBodySetPosition(cam_base, bas[0], look_up[1], bas[2]);

    cam_node->setPosition(bas[0], bas[1], bas[2]);

    cam_node->setOrientation(dBodyGetQuaternion(cam_base)[0], dBodyGetQuaternion(cam_base)[1], dBodyGetQuaternion(cam_base)[2], dBodyGetQuaternion(cam_base)[3]);
}
