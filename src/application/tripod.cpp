/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "tripod.h"

#include "config.hpp"

namespace bnn_device_3d::application
{

tripod::tripod(dWorldID world, Ogre::SceneNode* cam_node, dBodyID target)
    : target(target), cam_node(cam_node)
{
    detector = dBodyCreate(world);
    upper = dBodyCreate(world);
    lower = dBodyCreate(world);

    dBodySetPosition(detector, dBodyGetPosition(target)[0], dBodyGetPosition(target)[1], dBodyGetPosition(target)[2]);
    dBodySetPosition(upper, cam_node->getPosition()[0], cam_node->getPosition()[1], cam_node->getPosition()[2]);
    dBodySetPosition(lower, cam_node->getPosition()[0], 2.f * dBodyGetPosition(target)[1] - cam_node->getPosition()[1], cam_node->getPosition()[2]);

    cam_node->lookAt(Ogre::Vector3(dBodyGetPosition(detector)[0],dBodyGetPosition(detector)[1],dBodyGetPosition(detector)[2]), Ogre::Node::TS_PARENT);
    dQuaternion q = {
        cam_node->getOrientation().w,
        cam_node->getOrientation().x,
        cam_node->getOrientation().y,
        cam_node->getOrientation().z};
    dBodySetQuaternion(upper, q);

    {
        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed(world, jg);
        dJointAttach (j, upper, detector);
        dJointSetFixed(j);
    }

    {
        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed(world, jg);
        dJointAttach (j, lower, detector);
        dJointSetFixed(j);
    }
}

void tripod::step()
{
    dBodyAddForce(detector, 0, device_3d_GRAVITY, 0);
    dBodyAddForce(upper, 0, device_3d_GRAVITY, 0);
    dBodyAddForce(lower, 0, device_3d_GRAVITY, 0);

    target_pos = dBodyGetPosition(target);

    detector_pos = dBodyGetPosition(detector);
    detector_vel = dBodyGetLinearVel(detector);

    lower_pos = dBodyGetPosition(lower);
    lower_vel = dBodyGetLinearVel(lower);

    upper_pos = dBodyGetPosition(upper);
    upper_vel = dBodyGetLinearVel(upper);
    upper_quat = dBodyGetQuaternion(upper);

    detector_dx = target_pos[0] - detector_pos[0];
    detector_dy = target_pos[1] - detector_pos[1];
    detector_dz = target_pos[2] - detector_pos[2];

    if(1)
        if(abs(detector_dx) < 1.f && abs(detector_dy) < 1.f && abs(detector_dz) < 1.f)
        {
            detector_dx = 0.f;
            detector_dy = 0.f;
            detector_dz = 0.f;
        }

    dBodyAddForce(detector, detector_dx, detector_dy, detector_dz);
    dBodyAddForce(detector, -detector_vel[0] / 2, -detector_vel[1] / 2, -detector_vel[2] / 2);

    dx = upper_pos[0] - lower_pos[0];
    dy = (upper_pos[1] + lower_pos[1]) / 2 - detector_pos[1];
    dz = upper_pos[2] - lower_pos[2];

    dBodyAddForce(upper, -dx, -dy, -dz);
    dBodyAddForce(upper, -upper_vel[0] / 2, -upper_vel[1] / 2, -upper_vel[2] / 2);

    dBodyAddForce(lower, dx, -dy, dz);
    dBodyAddForce(lower, -lower_vel[0] / 2, -lower_vel[1] / 2, -lower_vel[2] / 2);

    cam_node->setPosition(upper_pos[0], upper_pos[1], upper_pos[2]);

    cam_node->setOrientation(upper_quat[0], upper_quat[1], upper_quat[2], upper_quat[3]);
}

void tripod::set_position(dReal x, dReal y, dReal z)
{
    auto p = dBodyGetPosition(detector);
    dReal dx = x - p[0];
    dReal dy = y - p[1];
    dReal dz = z - p[2];

    auto set = [&](dBodyID id)
    {
        p = dBodyGetPosition(id);
        dBodySetPosition(id, p[0] + dx, p[1] + dy, p[2] + dz);
    };

    set(detector);
    set(lower);
    set(upper);

    dBodySetLinearVel(detector, 0, 0, 0);
    dBodySetLinearVel(lower, 0, 0, 0);
    dBodySetLinearVel(upper, 0, 0, 0);
}

} // namespace bnn_device_3d::application
