/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "scene.h"

#include <lib/logger/src/helpers/log.h>

namespace bnn_device_3d::scenes
{

scene::~scene()
{
    dSpaceDestroy(space);
    dCleanupODEAllDataForThread();
    dCloseODE();
    //Ogre::RTShader::ShaderGenerator::getSingletonPtr()->removeSceneManager(scene_manager);
}

scene::scene(Ogre::RenderWindow* render_window, Ogre::SceneManager* scene_manager)
    : render_window(render_window), scene_manager(scene_manager)
{
    Ogre::RTShader::ShaderGenerator::getSingletonPtr()->addSceneManager(scene_manager);
    dInitODE2(0);
    world = dWorldCreate();
    contact_group = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, -device_3d_GRAVITY, 0);
    space = dSimpleSpaceCreate(nullptr);
}

void scene::load(std::ifstream& ifs)
{
    tripod_->load(ifs);

    if(creature_->bnn_->load(ifs))
        log_info("loaded")
    else
        log_error("load error")

    for(const auto& figure : creature_->get_figures())
        figure->load(ifs);

    for(auto& figure : stepping_figures)
        figure.load(ifs);
}

void scene::save(std::ofstream& ofs) const
{
    tripod_->save(ofs);

    if(creature_->bnn_->save(ofs))
        log_info("saved")
    else
        log_error("save error")

    for(const auto& figure : creature_->get_figures())
        figure->save(ofs);

    for(const auto& figure : stepping_figures)
        figure.save(ofs);
}

void scene::start()
{
    creature_->start();
}

void scene::stop()
{
    creature_->stop();
}

void scene::create_light(Ogre::Real x, Ogre::Real y, Ogre::Real z, const std::string name)
{
    auto light = scene_manager->createLight(name);
    auto lightNode = scene_manager->getRootSceneNode()->createChildSceneNode();
    lightNode->setPosition(x, y, z);
    lightNode->attachObject(light);
    //light->setSpecularColour(0.9, 0.5, 0.5);
    //light->setDiffuseColour(1, 1, 1);
    //light->setPowerScale(-100);
    //light->setSpotlightFalloff(0.1f);
}

void scene::collide_action()
{
    for(auto sg : stationary_colliding_geoms)
    {
        for(auto mg : movable_colliding_geoms)
            collide_action2(sg, mg);

        for(auto cg : creature_colliding_geoms)
            collide_action2(sg, cg);
    }

    for(auto it_0 = movable_colliding_geoms.begin(); it_0 != movable_colliding_geoms.end(); it_0++)
    {
        auto it_1 = it_0;
        while(++it_1 != movable_colliding_geoms.end())
            collide_action2(*it_0, *it_1);
    }

    for(auto cg : creature_colliding_geoms)
        for(auto mg : movable_colliding_geoms)
            collide_action2(cg, mg);
}

void scene::collide_action2(dGeomID o1, dGeomID o2)
{
    int i,n;
    const int N = 1;
    dContact contact[N];
    n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

    for(i = 0; i < n; i++)
    {
        contact[i].surface.mode = 0
                | dContactMu2
#if(0)
                | dContactSlip1
                | dContactSlip2
                | dContactSoftERP
                | dContactSoftCFM
                | dContactApprox1
                | dContactApprox1_N
                | dContactBounce
                | dContactMu2
                | dContactAxisDep
                | dContactFDir1
                | dContactBounce
                | dContactSoftERP
                | dContactSoftCFM
                | dContactMotion1
                | dContactMotion2
                | dContactMotionN
                | dContactSlip1
                | dContactSlip2
                | dContactRolling
                | dContactApprox0
                | dContactApprox1_11
                | dContactApprox1_2
                | dContactApprox1_N
                | dContactApprox1
#endif
                ;

        contact[i].surface.mu = 4;
        contact[i].surface.mu2 = 4;

#if(0)
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = dInfinity;
        contact[i].surface.bounce = 0.1f;
        contact[i].surface.bounce_vel = 0.0f;
        contact[i].surface.slip1 = 1;
        contact[i].surface.slip2 = 1;
        contact[i].surface.soft_erp = 1.0f;
        contact[i].surface.soft_cfm = 0.01;
#endif

        dJointAttach(dJointCreateContact(world, contact_group, &contact[i]),
                     dGeomGetBody(contact[i].geom.g1),
                     dGeomGetBody(contact[i].geom.g2));
    }
}

} // namespace bnn_device_3d::scenes
