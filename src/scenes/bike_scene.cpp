/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "bike_scene.h"

#include "scenes/bike_scene.h"
#include "scenes/table_scene.h"
#include "config.hpp"
#include "conductors/conductor_circle.h"
#include "conductors/kick.h"
#include "conductors/tail.h"
#include "conductors/dream.h"
#include "physical_objects/cube.h"
#include "physical_objects/sphere.h"
#include "creatures/bike/bike.h"

namespace cond = bnn_device_3d::conductors;
namespace pho = bnn_device_3d::physical_objects;

namespace bnn_device_3d::scenes
{

bike::~bike()
{

}

bike::bike(Ogre::RenderWindow* render_window, Ogre::SceneManager* scene_manager)
    : scene(render_window, scene_manager)
{

}

void bike::setup(
        std::list<dGeomID>& stationary_colliding_geoms,
        std::list<dGeomID>& movable_colliding_geoms,
        std::list<dGeomID>& creature_colliding_geoms,
        std::list<Ogre::SceneNode*>& bounding_nodes,
        std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
        dWorldID world)
{
    // ogre
    {
        create_light(10000, 10000, 10000, "MainLight0");
        create_light(-10000, 10000, 10000, "MainLight1");
        create_light(-10000, 10000, -10000, "MainLight2");
        create_light(10000, 10000, -10000, "MainLight3");
        create_light(0, -10000, 0, "MainLight4");
        third_person_camera_node = scene_manager->getRootSceneNode()->createChildSceneNode();
        creature_camera_node = scene_manager->getRootSceneNode()->createChildSceneNode();

        switch (0)
        {
        case 0:
            third_person_camera_node->setPosition(0, 400 * device_3d_SCALE, 1000 * device_3d_SCALE);
            break;
        case 1:
            third_person_camera_node->setPosition(0, 800, 0);
            third_person_camera_node->setOrientation(-f, f, 0, 0);
            break;
        }

        third_person_camera = scene_manager->createCamera("third_person_camera");
        third_person_camera->setNearClipDistance(1);
        third_person_camera->setAutoAspectRatio(true);
        third_person_camera_node->attachObject(third_person_camera);

        creature_camera = scene_manager->createCamera("creature_camera");
        creature_camera->setNearClipDistance(0.1);
        creature_camera->setAutoAspectRatio(true);
        creature_camera_node->attachObject(creature_camera);

        render_window->addViewport(third_person_camera, 0, 0.0f, 0.0f, 1.0f, 1.0f);
        render_window->addViewport(creature_camera, 1, 0.0f, 0.0f, 0.25f, 0.25f);

        render_window->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
        render_window->getViewport(1)->setBackgroundColour(Ogre::ColourValue::White);

        auto *ent_plane = scene_manager->createEntity("plane", Ogre::SceneManager::PrefabType::PT_PLANE);
        auto *node_plane = scene_manager->getRootSceneNode()->createChildSceneNode();
        node_plane->setScale(Ogre::Vector3(8, 8, 8));
        node_plane->setDirection(0,-1,0);
        node_plane->attachObject(ent_plane);
        ent_plane->setMaterial(pho::figure::create_material_chess(1024 * 8, 8, COLOR_BLACK, COLOR_DARK));
    }

    // fill_it_up
    {
        conductors.push_back(std::make_unique<cond::conductor_circle>(2.0f, 10));
        conductors.push_back(std::make_unique<cond::conductor_circle>(2.0f, 5));
        conductors.push_back(std::make_unique<cond::kick>());
        conductors.push_back(std::make_unique<cond::tail>());
        conductors.push_back(std::make_unique<cond::dream>());

        // creating stationary objects
        {
            stationary_colliding_geoms.push_back(dCreatePlane(space, 0.0f, 1.0f, 0.0f, 0.0f));

//            {
//                float r = 4.0;

//                stepping_figures.push_back(pho::cube("cube_jump", scene_manager, world, space,
//                                                     r*r*r * device_3d_MASS_SCALE, r, r, r));

//                stepping_figures.back().set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));

//                dBodySetPosition(stepping_figures.back().body, 0, -1.9, -40);

//                dQuaternion q = {
//                    1,
//                    1.1,
//                    0,
//                    0};
//                dBodySetQuaternion(stepping_figures.back().body, q);

//                stationary_colliding_geoms.push_back(stepping_figures.back().geom);

//                bounding_nodes.push_back(stepping_figures.back().node);

//                {
//                    dJointGroupID jg = dJointGroupCreate (0);
//                    dJointID j = dJointCreateFixed (world, jg);
//                    dJointAttach (j, nullptr, dGeomGetBody(stepping_figures.back().geom));
//                    dJointSetFixed(j);
//                }
//            }
        }

        // creating movable objects
        {
            {
                float r = 120.0 * device_3d_SCALE;// ((static_cast<float>(rand()) / RAND_MAX) * 20 + 100) * device_3d_SCALE;

                stepping_figures.push_back(pho::cube("cube_qqq", scene_manager, world, space,
                                                     r*r*r * device_3d_MASS_SCALE, r, r, r));

                stepping_figures.back().set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));

                dBodySetPosition(stepping_figures.back().body,
                                 ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 2 * 500 * device_3d_SCALE,
                                 ((static_cast<float>(rand()) / RAND_MAX)) * 500 * device_3d_SCALE,
                                 ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 2 * 500 * device_3d_SCALE);

                movable_colliding_geoms.push_back(stepping_figures.back().geom);

                bounding_nodes.push_back(stepping_figures.back().node);
            }

            for(int i = 0; i < 6; i++)
            {
                float r = ((static_cast<float>(rand()) / RAND_MAX) * 50 + 50) * device_3d_SCALE;

                stepping_figures.push_back(pho::sphere("sphere_" + std::to_string(i), scene_manager, world, space,
                                                       r*r*r * device_3d_MASS_SCALE, r));

                stepping_figures.back().set_material(pho::figure::create_material_chess(256, 32, COLOR_MEDIUM, COLOR_LIGHT));

                dBodySetPosition(stepping_figures.back().body,
                                 ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 2 * 1500 * device_3d_SCALE,
                                 ((static_cast<float>(rand()) / RAND_MAX)) * 1500 * device_3d_SCALE,
                                 ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 2 * 1500 * device_3d_SCALE);

                movable_colliding_geoms.push_back(stepping_figures.back().geom);

                bounding_nodes.push_back(stepping_figures.back().node);
            }
        }

        // creating creature
        {

            creature_.reset(new creatures::bike::bike(render_window, scene_manager, world));
            creature_->set_position(2, creature_->get_height(), 0);

            if(0)
            {
                dJointGroupID jg = dJointGroupCreate (0);
                dJointID j = dJointCreateFixed (world, jg);
                dJointAttach(j, dGeomGetBody(creature_->get_body().geom), nullptr);
                dJointSetFixed(j);
            }

            //creature_colliding_geoms.push_back(creature_->get_body().geom);

            for(const auto& aa : creature_->get_figures())
            {
                creature_colliding_geoms.push_back(aa->geom);
                bounding_nodes.push_back(aa->node);
            }

            if(0)
            {
                dJointGroupID gc_body = dJointGroupCreate(0);
                dJointID j_body = dJointCreateFixed(world, gc_body);
                dJointAttach (j_body, 0, dGeomGetBody(creature_->get_body().geom));
                dJointSetFixed(j_body);
            }
        }

        auto *body = creature_->get_body().body;

        third_person_camera_node->setPosition(dBodyGetPosition(body)[0] + 0,
                dBodyGetPosition(body)[1] + 4,
                dBodyGetPosition(body)[2] + 10);

        tripod_.reset(new application::tripod(world, third_person_camera_node, body));
    }
}

void bike::step(
        std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
        std::list<Ogre::SceneNode*>& bounding_nodes,
        keys_states& keys_states_)
{
    for(auto& figure : stepping_figures)
        figure.step();

    if(1)
    {
        static bool go = false;
        if(go)
        {
            const Ogre::Quaternion ort_x(0, 0, 1, 0);
            const dReal* body_q = dBodyGetQuaternion(creature_->get_body().body);
            Ogre::Quaternion body_quat = Ogre::Quaternion(body_q[0], body_q[1], body_q[2], body_q[3]);
            Ogre::Quaternion body_quat_inv = body_quat.Inverse();
            Ogre::Quaternion ort_x_rel = body_quat * ort_x * body_quat_inv;
            ort_x_rel.normalise();

            if(ort_x_rel.y < 0.1)
                go = false;
        }
        else
        {
            static float max = 0;
            auto pp = dBodyGetPosition(creature_->get_body().body);
            float dx,dz;
            dx = pp[0] - 2;
            dz = pp[2] - 0;
            float ff = pow((dx*dx + dz*dz), 0.5);

            if(ff>max)
            {
                max = ff;
                logging(std::to_string(ff));
            }

            //dBodySetForce(creature_->get_body().body, 0, 0, -1000);
            creature_->set_position(2, creature_->get_height(), 0);
            tripod_->set_position(2, creature_->get_height(), 0);
            go = true;
        }
    }

    bool verbose = false;//this->verbose;
    std::string debug_str;
    creature_->step(debug_str, verbose);
    tripod_->step();

    if(verbose)
        std::cout << debug_str << std::endl;

    auto pl = creature_->get_camera_place();
    creature_camera_node->setPosition(static_cast<Ogre::Real>(pl[0]), static_cast<Ogre::Real>(pl[1]), static_cast<Ogre::Real>(pl[2]));

    const dReal* dir = dBodyGetQuaternion(creature_->get_body().body);
    creature_camera_node->setOrientation(static_cast<Ogre::Real>(dir[0]), static_cast<Ogre::Real>(dir[1]), static_cast<Ogre::Real>(dir[2]), static_cast<Ogre::Real>(dir[3]));

    for(auto& node : bounding_nodes)
        node->_updateBounds();

    dBodyAddForce(creature_->get_body().body, 0, 0, 0);

    {
        float force_cube = 50;
        float force_creature = 1;
        auto it = stepping_figures.begin();
        std::advance(it, 4);

        if(keys_states_.key_down)
            dBodyAddForce(it->body, 0, 0, +force_cube);

        if(keys_states_.key_up)
            dBodyAddForce(it->body, 0, 0, -force_cube);

        if(keys_states_.key_left)
            dBodyAddForce(it->body, -force_cube, 0, 0);

        if(keys_states_.key_right)
            dBodyAddForce(it->body, +force_cube, 0, 0);

        if(keys_states_.key_a)
            dBodyAddForce(creature_->get_body().body, -force_creature, 0, 0);

        if(keys_states_.key_d)
            dBodyAddForce(creature_->get_body().body, force_creature, 0, 0);

        if(keys_states_.key_w)
            dBodyAddForce(creature_->get_body().body, 0, 0, -force_creature);

        if(keys_states_.key_s)
            dBodyAddForce(creature_->get_body().body, 0, 0, force_creature);
    }
}

} // namespace bnn_device_3d::scenes
