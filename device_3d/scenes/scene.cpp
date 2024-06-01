/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "scene.h"

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
    space = dSimpleSpaceCreate(nullptr);
}

void scene::load(std::ifstream& ifs)
{
    tripod_->load(ifs);
}

void scene::save(std::ofstream& ofs) const
{
    tripod_->save(ofs);
}

void scene::start()
{
    creature_->start();
}

void scene::stop()
{
    creature_->stop();
}

void scene::create_light(Real x, Real y, Real z, const std::string name)
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

} // namespace bnn_device_3d::scenes
