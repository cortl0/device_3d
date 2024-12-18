/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "table_scene.h"

#include <lib/logger/src/helpers/log.h>

#include "scenes/bike_scene.h"
#include "scenes/table_scene.h"
#include "config.hpp"
#include "conductors/conductor_circle.h"
#include "conductors/kick.h"
#include "conductors/tail.h"
#include "conductors/dream.h"
#include "physical_objects/cube.h"
#include "physical_objects/sphere.h"
#include "creatures/table/table.h"

namespace cond = bnn_device_3d::conductors;
namespace pho = bnn_device_3d::physical_objects;

namespace bnn_device_3d::scenes::table
{

table::table(Ogre::RenderWindow* render_window, Ogre::SceneManager* scene_manager)
    : scene(render_window, scene_manager)
{

}

void table::setup(const bnn_device_3d::application::config::device_3d::bnn& config_bnn)
{
    ogre_setup();

    // fill_it_up
    {
        conductors.push_back(std::make_unique<cond::conductor_circle>(2.0f, 10));
        conductors.push_back(std::make_unique<cond::conductor_circle>(2.0f, 5));
        conductors.push_back(std::make_unique<cond::kick>());
        conductors.push_back(std::make_unique<cond::tail>());
        conductors.push_back(std::make_unique<cond::dream>());

        creating_stationary_objects();
        creating_movable_objects();
        creating_creature(config_bnn);

        auto body = creature_->get_body().body;

        if(0)
        {
            auto it = stepping_figures.begin();
            std::advance(it, 7);
            //std::advance(it, 4);
            body = it->body;
        }

        third_person_camera_node->setPosition(dBodyGetPosition(body)[0] + 0,
                dBodyGetPosition(body)[1] + 4,
                dBodyGetPosition(body)[2] + 10);

        tripod_.reset(new application::tripod(world, third_person_camera_node, body));
    }
}

void table::ogre_setup()
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

    auto ent_plane = scene_manager->createEntity("plane", Ogre::SceneManager::PT_PLANE);
    auto node_plane = scene_manager->getRootSceneNode()->createChildSceneNode();
    node_plane->setScale(Ogre::Vector3(8, 8, 8));
    node_plane->setDirection(0,-1,0);
    node_plane->attachObject(ent_plane);
    ent_plane->setMaterial(pho::figure::create_material_chess(1024 * 8, 8, COLOR_BLACK, COLOR_DARK));
}

void table::creating_stationary_objects()
{
    stationary_colliding_geoms.push_back(dCreatePlane(space, 0.0f, 1.0f, 0.0f, 0.0f));

    float size = 5000.0f;
    float height = 100.0f;
    float koef_size = 50.0f;

    // creating walls
    for(int i = 0; i < 4; i++)
    {
        float x,y,z;
        x=(i & 1) ? size * device_3d_SCALE : size / koef_size * device_3d_SCALE;
        y=height * device_3d_SCALE;
        z=(i & 1) ? size * device_3d_SCALE / koef_size : size * device_3d_SCALE;
        stepping_figures.push_back(pho::cube("wall_" + std::to_string(i) + std::to_string(i),
                                             scene_manager, world, space, x * y * z * device_3d_MASS_SCALE,
                                             x, y, z));

        stepping_figures.back().set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));

        dBodySetPosition(stepping_figures.back().body,
                         !(i & 1) * ((i >> 1) * 2 - 1) * (size / 2 + size / koef_size/2 + 1) * device_3d_SCALE,
                         (height / 2 + 1) * device_3d_SCALE,
                         (i & 1) * ((i >> 1) * 2 - 1) * (size / 2 + size / koef_size/2 + 1) * device_3d_SCALE);

        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach (j, nullptr, dGeomGetBody(stepping_figures.back().geom));
        dJointSetFixed(j);

        stationary_colliding_geoms.push_back(stepping_figures.back().geom);

        bounding_nodes.push_back(stepping_figures.back().node);
    }
}

void table::creating_movable_objects()
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

    auto create_sphere = [&](
            std::string name,
            float r,
            Ogre::MaterialPtr material,
            float x, float y, float z)
    {
        stepping_figures.push_back(pho::sphere(name, scene_manager, world, space, r*r*r * device_3d_MASS_SCALE, r));
        stepping_figures.back().set_material(material);
        dBodySetPosition(stepping_figures.back().body, x, y, z);
        movable_colliding_geoms.push_back(stepping_figures.back().geom);
        bounding_nodes.push_back(stepping_figures.back().node);
    };

    create_sphere("sphere_circle_1", 100.0f * device_3d_SCALE,
                  pho::figure::create_material_chess(256, 32, COLOR_MEDIUM, COLOR_LIGHT),
                  0, 1, -10);

    create_sphere("sphere_circle_2", 100.0f * device_3d_SCALE,
                  pho::figure::create_material_chess(256, 32, COLOR_MEDIUM, COLOR_LIGHT),
                  0, 1, -5);

    create_sphere("sphere_kick", 50.0f * device_3d_SCALE,
                  pho::figure::create_material_chess(256, 32, COLOR_MEDIUM, COLOR_LIGHT),
                  -15, 1, 0);

    auto create_cube = [&](
            std::string name,
            float r,
            Ogre::MaterialPtr material,
            float x, float y, float z)
    {
        stepping_figures.push_back(pho::cube(name, scene_manager, world, space, r*r*r * device_3d_MASS_SCALE / 10.0f, r, r, r));
        stepping_figures.back().set_material(material);
        dBodySetPosition(stepping_figures.back().body, x, y, z);
        movable_colliding_geoms.push_back(stepping_figures.back().geom);
        bounding_nodes.push_back(stepping_figures.back().node);
    };

    create_cube("cube_tail",
                80.0 * device_3d_SCALE,
                pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT),
                -3.0f, 1.0f, 3.0f);

    create_cube("cube_dream",
                80.0 * device_3d_SCALE,
                pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT),
                4.0f, 1.0f, -4.0f);

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

void table::creating_creature(const bnn_device_3d::application::config::device_3d::bnn& config_bnn)
{
    creature_.reset(new creatures::table::table(render_window, scene_manager, world, config_bnn));
    creature_->set_position(0, 0.5, 0);

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

    //creature_->get_body().node->showBoundingBox(true);
}

void table::step(keys_states& keys_state, dReal frame_length_dReal)
{
    dJointGroupEmpty(contact_group);

    for(auto& figure : stepping_figures)
        figure.update_visual();

    circle_step(5, 0, stepping_figures);
    circle_step(6, 1, stepping_figures);
    kick_step(stepping_figures);
    tail_step(stepping_figures);
    dream_step(stepping_figures);

    bool verbose = false;//this->verbose;
    std::string debug_str;
    creature_->step(debug_str, verbose);
    tripod_->step();

    if(verbose)
        log_debug("%s", debug_str.c_str());

    auto pl = creature_->get_camera_place();
    creature_camera_node->setPosition(static_cast<Ogre::Real>(pl[0]), static_cast<Ogre::Real>(pl[1]), static_cast<Ogre::Real>(pl[2]));

    const dReal* dir = dBodyGetQuaternion(creature_->get_body().body);
    creature_camera_node->setOrientation(static_cast<Ogre::Real>(dir[0]), static_cast<Ogre::Real>(dir[1]), static_cast<Ogre::Real>(dir[2]), static_cast<Ogre::Real>(dir[3]));

    for(auto& node : bounding_nodes)
        node->_updateBounds();

    keyboard_polling(keys_state);
    collide_action();
    dWorldStep(world, frame_length_dReal);
}

void table::circle_step(
        size_t figure_number,
        size_t conductor_number,
        std::list<bnn_device_3d::physical_objects::figure>& stepping_figures
        )
{
    auto it = stepping_figures.begin();
    std::advance(it, figure_number);
    const auto pos = dBodyGetPosition(it->body);
    float x = pos[0];
    float y = pos[1];
    float z = pos[2];
    conductors[conductor_number]->step(x, y, z);
    auto vel = dBodyGetLinearVel(it->body);
    dBodyAddForce(it->body, x - vel[0], y - vel[1], z - vel[2]);
}

void table::kick_step(std::list<bnn_device_3d::physical_objects::figure>& stepping_figures)
{
    auto it = stepping_figures.begin();
    std::advance(it, 7);
    const auto pos = dBodyGetPosition(it->body);
    float x = pos[0];
    float y = pos[1];
    float z = pos[2];

    const auto creature_pos = dBodyGetPosition(creature_->get_body().body);

    cond::kick::data kd
    {
        .body_x = static_cast<float>(creature_pos[0]),
        .body_y = static_cast<float>(creature_pos[1]),
        .body_z = static_cast<float>(creature_pos[2]),
    };

    conductors[2]->step(x, y, z, &kd);
    if(kd.conduct)
    {
        auto vel = dBodyGetLinearVel(it->body);
        dBodyAddForce(it->body, x - vel[0], y - vel[1], z - vel[2]);
    }
}

void table::tail_step(std::list<bnn_device_3d::physical_objects::figure>& stepping_figures)
{
    auto it = stepping_figures.begin();
    std::advance(it, 8);
    const auto pos = dBodyGetPosition(it->body);
    float x = pos[0];
    float y = pos[1];
    float z = pos[2];

    const auto creature_pos = dBodyGetPosition(creature_->get_body().body);

    cond::tail::data kd
    {
        .body_x = static_cast<float>(creature_pos[0]),
        .body_y = static_cast<float>(creature_pos[1]),
        .body_z = static_cast<float>(creature_pos[2]),
    };

    conductors[3]->step(x, y, z, &kd);
    if(kd.conduct)
    {
        auto vel = dBodyGetLinearVel(it->body);
        dBodyAddForce(it->body, x - vel[0], y - vel[1], z - vel[2]);
    }
}

void table::dream_step(std::list<bnn_device_3d::physical_objects::figure>& stepping_figures)
{
    auto it = stepping_figures.begin();
    std::advance(it, 9);
    const auto pos = dBodyGetPosition(it->body);
    float x = pos[0];
    float y = pos[1];
    float z = pos[2];

    const auto creature_pos = dBodyGetPosition(creature_->get_body().body);

    cond::dream::data kd
    {
        .body_x = static_cast<float>(creature_pos[0]),
        .body_y = static_cast<float>(creature_pos[1]),
        .body_z = static_cast<float>(creature_pos[2]),
    };

    conductors[4]->step(x, y, z, &kd);
    if(kd.conduct)
    {
        auto vel = dBodyGetLinearVel(it->body);
        dBodyAddForce(it->body, x - vel[0], y - vel[1], z - vel[2]);
    }
}

void table::keyboard_polling(keys_states& keys_states)
{
    float force_cube = 50;
    float force_creature = 10;
    auto it = stepping_figures.begin();
    std::advance(it, 4);

    if(keys_states.key_down)
        dBodyAddForce(it->body, 0, 0, +force_cube);

    if(keys_states.key_up)
        dBodyAddForce(it->body, 0, 0, -force_cube);

    if(keys_states.key_left)
        dBodyAddForce(it->body, -force_cube, 0, 0);

    if(keys_states.key_right)
        dBodyAddForce(it->body, +force_cube, 0, 0);

    if(keys_states.key_a)
        dBodyAddForce(creature_->get_body().body, -force_creature, 0, 0);

    if(keys_states.key_d)
        dBodyAddForce(creature_->get_body().body, force_creature, 0, 0);

    if(keys_states.key_w)
        dBodyAddForce(creature_->get_body().body, 0, 0, -force_creature);

    if(keys_states.key_s)
        dBodyAddForce(creature_->get_body().body, 0, 0, force_creature);
}

} // namespace bnn_device_3d::scenes
