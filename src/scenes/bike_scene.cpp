/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "bike_scene.h"

#include <OgreOverlaySystem.h>
#include <OgreOverlayManager.h>
#include <OgreOverlayElement.h>
#include <OgreRenderQueueListener.h>
#include <OgreOverlayContainer.h>
#include <OgreOverlay.h>
#include <OgreFontManager.h>

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

static const std::string font_name{"app_font_name"s};

void load_font()
{
    static const std::string resource_group{"General"s};

    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("../src/resources", "FileSystem");
   // Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    Ogre::FontPtr mFont = Ogre::FontManager::getSingleton().create(font_name, resource_group
    //,Ogre::OverlayElement::DEFAULT_RESOURCE_GROUP
                                                                       );
    mFont->setType(Ogre::FT_TRUETYPE);
    mFont->setSource("Hack-Regular.ttf");
    mFont->setParameter("size","26");
    mFont->setParameter("resolution","96");
    mFont->load();
}

Ogre::OverlayContainer* create_panel(Ogre::OverlayManager& overlayManager)
{
    auto panel = static_cast<Ogre::OverlayContainer*>
        (overlayManager.createOverlayElement("Panel", "PanelName"));

    panel->setMetricsMode(Ogre::GMM_PIXELS);
    panel->setPosition(0.0, 0.0);
    panel->setDimensions(0.0, 0.0);

//    panel->setMetricsMode(Ogre::GMM_RELATIVE);
//    panel->setPosition(0.0, 0.75);
//    panel->setDimensions(1.0, 0.25);

    //panel->setMaterialName("BaseWhite");
    panel->show();

    return panel;
}

Ogre::TextAreaOverlayElement* create_text_area(Ogre::OverlayManager& overlayManager)
{
    const std::string text_area_name{"TextAreaName"s};
    const std::string ogre_text_area_type_name{"TextArea"s};

    auto text = static_cast<Ogre::TextAreaOverlayElement*>
        (overlayManager.createOverlayElement(ogre_text_area_type_name, text_area_name));

    text->setFontName(font_name);
    text->setMetricsMode(Ogre::GMM_RELATIVE);
    //text->setMetricsMode(Ogre::GMM_PIXELS);
    text->setPosition(0.0, 0.0);
    text->setDimensions(1.0, 1.0);
    text->setHorizontalAlignment(Ogre::GuiHorizontalAlignment::GHA_LEFT);
    text->setVerticalAlignment(Ogre::GuiVerticalAlignment::GVA_TOP);
    text->setCharHeight(0.05);
    //text->setCharHeight(0.2);
    //text->setColour(Ogre::ColourValue(1, 1, 1));

    text->setCaption("some text");
    text->show();
    return text;
}

Ogre::Overlay* create_overlay(Ogre::OverlayManager& overlayManager)
{
    const std::string overlay_name{"OverlayName"s};
    auto overlay = overlayManager.create(overlay_name);
    overlay->setZOrder(0);
    overlay->show();
    return overlay;
}

Ogre::TextAreaOverlayElement* bike::create_text_panel()
{
    //    scene_manager->addSpecialCaseRenderQueue(Ogre::RENDER_QUEUE_OVERLAY);
    //    scene_manager->setSpecialCaseRenderQueueMode(Ogre::SceneManager::SCRQM_INCLUDE);

    load_font();
    auto& overlayManager = Ogre::OverlayManager::getSingleton();
    text_area = create_text_area(overlayManager);
    panel = create_panel(overlayManager);
    auto overlay = create_overlay(overlayManager);
    panel->addChild(text_area);
    overlay->add2D(panel);
    auto pOverlaySystem = Ogre::OverlaySystem::getSingletonPtr();
    scene_manager->addRenderQueueListener(pOverlaySystem);
    return text_area;
}

void bike::set_text(std::string& str)
{
    text_area->setCaption(str);
}

void bike::setup(
        std::list<dGeomID>& stationary_colliding_geoms,
        std::list<dGeomID>& movable_colliding_geoms,
        std::list<dGeomID>& creature_colliding_geoms,
        std::list<Ogre::SceneNode*>& bounding_nodes,
        std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
        dWorldID world
        )
{
    ogre_setup();

    // fill_it_up
    {
        conductors.push_back(std::make_unique<cond::conductor_circle>(2.0f, 10));
        conductors.push_back(std::make_unique<cond::conductor_circle>(2.0f, 5));
        conductors.push_back(std::make_unique<cond::kick>());
        conductors.push_back(std::make_unique<cond::tail>());
        conductors.push_back(std::make_unique<cond::dream>());

        creating_stationary_objects(stationary_colliding_geoms, bounding_nodes, stepping_figures, world);
        creating_movable_objects(stepping_figures, movable_colliding_geoms, bounding_nodes, world);
        creating_creature(creature_colliding_geoms, bounding_nodes, world);

        auto body = creature_->get_body().body;

        third_person_camera_node->setPosition(dBodyGetPosition(body)[0] + 0,
                dBodyGetPosition(body)[1] + 4,
                dBodyGetPosition(body)[2] + 10);

        tripod_.reset(new application::tripod(world, third_person_camera_node, body));
    }
}

void bike::ogre_setup()
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

    int ZOrder = 0;
    render_window->addViewport(third_person_camera, ZOrder, 0.0f, 0.0f, 1.0f, 1.0f);
    render_window->getViewport(ZOrder)->setBackgroundColour(Ogre::ColourValue::White);

    ++ZOrder;
    render_window->addViewport(creature_camera, ZOrder, 0.0f, 0.0f, 0.25f, 0.25f);
    render_window->getViewport(ZOrder)->setBackgroundColour(Ogre::ColourValue::White);

    auto ent_plane = scene_manager->createEntity("plane", Ogre::SceneManager::PrefabType::PT_PLANE);
    auto node_plane = scene_manager->getRootSceneNode()->createChildSceneNode();
    node_plane->setScale(Ogre::Vector3(8, 8, 8));
    node_plane->setDirection(0,-1,0);
    node_plane->attachObject(ent_plane);
    ent_plane->setMaterial(pho::figure::create_material_chess(1024 * 8, 8, COLOR_BLACK, COLOR_DARK));

    text_area = create_text_panel();
}

void bike::creating_stationary_objects(
        std::list<dGeomID>& stationary_colliding_geoms,
        std::list<Ogre::SceneNode*>& bounding_nodes,
        std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
        dWorldID world
        )
{

    auto create_stationarity_cube = [&](std::string name, dReal x, dReal y, dReal z, dReal xx, dReal yy, dReal zz, dReal mass)
    {
        stepping_figures.push_back(pho::cube(name, scene_manager, world, space, mass, x, y, z));

        stepping_figures.back().set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));

        dBodySetPosition(stepping_figures.back().body, xx, yy, zz);

        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach (j, nullptr, dGeomGetBody(stepping_figures.back().geom));
        dJointSetFixed(j);

        stationary_colliding_geoms.push_back(stepping_figures.back().geom);

        bounding_nodes.push_back(stepping_figures.back().node);
    };

    stationary_colliding_geoms.push_back(dCreatePlane(space, 0.0f, 1.0f, 0.0f, 0.0f));

    {
        dReal size = 1;
        create_stationarity_cube("cube_qqq1",
                                 size * 20, size * 2, size * 0.5,
                                 30, 1, -50, 20.0 * device_3d_MASS_SCALE);

        create_stationarity_cube("cube_qqq2",
                                 size * 20, size * 2, size * 0.5,
                                 30, 1, -70, 20.0 * device_3d_MASS_SCALE);
    }

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

void bike::creating_movable_objects(
        std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
        std::list<dGeomID>& movable_colliding_geoms,
        std::list<Ogre::SceneNode*>& bounding_nodes,
        dWorldID world
        )
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

void bike::creating_creature(
        std::list<dGeomID>& creature_colliding_geoms,
        std::list<Ogre::SceneNode*>& bounding_nodes,
        dWorldID world
        )
{

    creature_.reset(new creatures::bike::bike(render_window, scene_manager, world));
    creature_->set_position(2, creature_->get_height() * 20, 0);

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

void bike::panel_to_place()
{
    width = render_window->getViewport(0)->getActualWidth();
    height = render_window->getViewport(0)->getActualHeight();

    auto x0 = 0;
    auto x1 = width;
    auto y0 = height * 3.0 / 4.0;
    auto y1 = height / 4.0;
    panel->setPosition(x0, y0);
    panel->setDimensions(x1, y1);
}

void bike::step(
        std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
        std::list<Ogre::SceneNode*>& bounding_nodes,
        keys_states& keys_state
        )
{
    panel_to_place();

    for(auto& figure : stepping_figures)
        figure.step();

    if(1)
    {
//        constexpr int start_force_start_value = 1000;
//        constexpr int up_value_devider = 100;
        static bool go = false;
//        static int start_force = start_force_start_value;
        if(go)
        {
            if(is_fail())
            {
                go = false;
//                start_force = start_force_start_value;
            }

//            if(--start_force < 0)
//                start_force = 0;
        }
        else
        {
            print_distance();

            //dBodySetForce(creature_->get_body().body, 0, 0, -1000);
            dBodySetForce(creature_->get_body().body, (rand()%2) * 2 - 1, 0, 0);
            creature_->set_position(2, creature_->get_height(), 0);
            tripod_->set_position(2, creature_->get_height(), 0);
            go = true;
        }

        //dBodySetForce(creature_->get_body().body, 0, start_force / up_value_devider, -start_force / up_value_devider / 4);
    }

    bool verbose = false;//this->verbose;
    std::string debug_str;
    creature_->step(debug_str, verbose);
    tripod_->step();

    auto cr = static_cast<bnn_device_3d::creatures::bike::bike*>(creature_.get());

    static uint i = 0;
    std::string d(
                std::to_string(++i) +
                "\n" +(cr->front_trotle < 0 ? " v" : " ^") +
                "\n" +(cr->front_direction < 0 ? "<- " : " ->") +
                "\n" +(cr->rear_trotle < 0 ? " v" : " ^") + "\n"
                );
    set_text(d);

    if(verbose)
        std::cout << debug_str << std::endl;

    auto pl = creature_->get_camera_place();
    creature_camera_node->setPosition(static_cast<Ogre::Real>(pl[0]), static_cast<Ogre::Real>(pl[1]), static_cast<Ogre::Real>(pl[2]));

    const dReal* dir = dBodyGetQuaternion(creature_->get_body().body);
    creature_camera_node->setOrientation(static_cast<Ogre::Real>(dir[0]), static_cast<Ogre::Real>(dir[1]), static_cast<Ogre::Real>(dir[2]), static_cast<Ogre::Real>(dir[3]));

    for(auto& node : bounding_nodes)
        node->_updateBounds();

    dBodyAddForce(creature_->get_body().body, 0, 0, 0);

    keyboard_polling(stepping_figures, keys_state);
}

bool bike::is_fail()
{
    const Ogre::Quaternion ort_x(0, 0, 1, 0);
    const dReal* body_q = dBodyGetQuaternion(creature_->get_body().body);
    Ogre::Quaternion body_quat = Ogre::Quaternion(body_q[0], body_q[1], body_q[2], body_q[3]);
    Ogre::Quaternion body_quat_inv = body_quat.Inverse();
    Ogre::Quaternion ort_x_rel = body_quat * ort_x * body_quat_inv;
    ort_x_rel.normalise();

    if(ort_x_rel.y < 0.1)
        return true;

    return false;
}

void bike::print_distance()
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
}

void bike::keyboard_polling(
        std::list<bnn_device_3d::physical_objects::figure>& stepping_figures,
        keys_states& keys_states
        )
{
    {
        float force_cube = 50;
        float force_creature = 1;
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
}

} // namespace bnn_device_3d::scenes
