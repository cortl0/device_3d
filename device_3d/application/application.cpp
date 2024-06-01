/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "application.h"

#include <filesystem>
#include <thread>

#include <OgreOverlay.h>
#include <OgreOverlayManager.h>
#include <OgreOverlayContainer.h>

#include <submodules/logger/src/helpers/log.h>

#include "config.hpp"
#include "conductors/conductor_circle.h"
#include "conductors/kick.h"
#include "conductors/tail.h"
#include "conductors/dream.h"
#include "scenes/bike_scene.h"
#include "scenes/table_scene.h"

namespace fs = std::filesystem;

namespace cond = bnn_device_3d::conductors;
namespace pho = bnn_device_3d::physical_objects;
namespace scn = bnn_device_3d::scenes;
namespace sch = std::chrono;
typedef sch::time_point<sch::system_clock, sch::microseconds> m_time_point;

namespace bnn_device_3d::application
{

using namespace std::chrono_literals;

application::~application()
{
    log_place
    stop();
    // dJointGroupEmpty(contact_group);
    // dJointGroupDestroy(contact_group);
    // dWorldDestroy(world);
    dCloseODE();
    log_place
}

application::application() : OgreBites::ApplicationContext("bnn_test_app")
{
    if(!config_.parse())
        exit(EXIT_FAILURE);

    config_.print();
    world = dWorldCreate();
    contact_group = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, -device_3d_GRAVITY, 0);
}

void application::collide_action()
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

void application::collide_action2(dGeomID o1, dGeomID o2)
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

dJointGroupID application::get_gontact_group() const
{
    return contact_group;
}

dWorldID application::get_world() const
{
    return world;
}

bool application::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    switch (evt.keysym.sym)
    {
    case OgreBites::SDLK_DOWN:
        keys_states_.key_down = true;
        break;
    case OgreBites::SDLK_UP:
        keys_states_.key_up = true;
        break;
    case OgreBites::SDLK_LEFT:
        keys_states_.key_left = true;
        break;
    case OgreBites::SDLK_RIGHT:
        keys_states_.key_right = true;
        break;
    case keyboard_key_a: // left
        keys_states_.key_a = true;
        break;
    case keyboard_key_d: // right
        keys_states_.key_d = true;
        break;
    case keyboard_key_w: // up
        keys_states_.key_w = true;
        break;
    case keyboard_key_s: // down
        keys_states_.key_s = true;
        break;
    }

    return true;
}

bool application::keyReleased(const OgreBites::KeyboardEvent& evt)
{
    switch (evt.keysym.sym)
    {
    case OgreBites::SDLK_ESCAPE:
        std::thread([this](){ stop(); getRoot()->queueEndRendering(); }).detach();
        break;
    case OgreBites::SDLK_DOWN:
        keys_states_.key_down = false;
        break;
    case OgreBites::SDLK_UP:
        keys_states_.key_up = false;
        break;
    case OgreBites::SDLK_LEFT:
        keys_states_.key_left = false;
        break;
    case OgreBites::SDLK_RIGHT:
        keys_states_.key_right = false;
        break;
    case keyboard_key_c: // load
        std::thread(&application::load, this).detach();
        break;
    case keyboard_key_z: // save
        std::thread(&application::save, this).detach();
        break;
    case keyboard_key_r: // save random
        std::thread(&application::save_random, this).detach();
        break;
    case keyboard_key_x: // stop <-> start
        if(bnn::state::started == state_)
            std::thread(&application::stop, this).detach();
        else if(bnn::state::stopped == state_)
            std::thread(&application::start, this).detach();
        break;
    case keyboard_key_v: // verbose
        verbose = !verbose;
        break;
    case keyboard_key_a: // left
        keys_states_.key_a = false;
        break;
    case keyboard_key_d: // right
        keys_states_.key_d = false;
        break;
    case keyboard_key_w: // up
        keys_states_.key_w = false;
        break;
    case keyboard_key_s: // down
        keys_states_.key_s = false;
        break;
    }

    return true;
}

void application::load()
{
    auto state = state_;
    stop();
    std::list<std::string> l;

    for(auto& p : fs::directory_iterator(fs::current_path()))
        if(p.path().filename().extension() == ".bnn")
            l.push_back(p.path().filename());

    if(!l.size())
        return;

    l.sort();
    std::ifstream ifs(fs::current_path() / l.back(), std::ios::binary);
    scene->load(ifs);

    if(scene->creature_->bnn_->load(ifs))
    {
        log_info("loaded");

        if(l.size() > 8)
            fs::remove_all(fs::current_path() / l.front());
    }
    else
    {
        log_error("load error");
    }

//    auto load_figure = [&ifs](dBodyID body_id)
//    {
//        dReal pos[3];
//        dReal dir[4];
//        ifs.read(reinterpret_cast<char*>(pos), sizeof(dReal) * 3);
//        ifs.read(reinterpret_cast<char*>(dir), sizeof(dReal) * 4);
//        dBodySetPosition(body_id, pos[0], pos[1], pos[2]);
//        dBodySetQuaternion(body_id, dir);
//    };

    for(const auto& figure : scene->creature_->get_figures())
        figure->load(ifs);

    for(auto& figure : stepping_figures)
        figure.load(ifs);

    ifs.close();

    if(bnn::state::started == state)
        start();
}

void application::run()
{
    initApp();

    try
    {
        auto width = getRenderWindow()->getWidth();
        auto height = getRenderWindow()->getHeight();
        long delta;
        const long frame_length = 1000000 / 60;
        const dReal frame_length_dReal = (dReal)frame_length / 1000000.0 * config_.device_3d_.time_coefficient;
        m_time_point current_time = sch::time_point_cast<m_time_point::duration>(sch::system_clock::time_point(sch::system_clock::now()));
        m_time_point time_old = current_time;
        auto root = getRoot();
        scene->creature_->bnn_->initialize();
        std::thread(&application::start, this).detach();

        while(!root->endRenderingQueued())
        {
            if(bnn::state::stop == state_)
                state_ = bnn::state::stopped;

            if(bnn::state::start == state_)
                state_ = bnn::state::started;

            if(bnn::state::started != state_)
            {
                std::this_thread::sleep_for(device_3d_LITTLE_TIMEms);
                time_old = sch::time_point_cast<m_time_point::duration>(sch::system_clock::time_point(sch::system_clock::now()));
                root->renderOneFrame();
                continue;
            }

            dJointGroupEmpty(contact_group);
            scene->step(stepping_figures, bounding_nodes, keys_states_);
            collide_action();
            dWorldStep(world, frame_length_dReal);
            root->renderOneFrame();

//            {
//                static Image img(getRenderWindow()->suggestPixelFormat(), width, height);
//                static PixelBox pb = img.getPixelBox();

//                getRenderWindow()->copyContentsToMemory(pb, pb);

//                Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pb.data);
//                scene->creature_->video_->calculate_data(pDest, width, height);

//                //img.save("filename5.png");
//            }

            current_time = sch::time_point_cast<m_time_point::duration>(sch::system_clock::time_point(sch::system_clock::now()));
            delta = (current_time - time_old).count();

            if(delta < frame_length)
                std::this_thread::sleep_for(std::chrono::microseconds(frame_length - delta));

            time_old += sch::microseconds((long long int)frame_length);
        }

        stop();
    }
    catch (...)
    {
        log_error("unknown error");
    }

    closeApp();
}

void application::save()
{
    auto state = state_;
    stop();
    static char time_buffer[15];
    std::time_t time = std::time(nullptr);
    std::strftime(time_buffer, 15, "%Y%m%d%H%M%S", std::localtime(&time));
    std::string s(time_buffer);
    s += ".bnn";
    std::ofstream ofs(fs::current_path() / s, std::ios::binary);
    scene->save(ofs);

    if(scene->creature_->bnn_->save(ofs))
        log_info("saved")
    else
        log_error("save error")

//    auto save_figure = [&ofs](dBodyID body_id)
//    {
//        const dReal* pos = dBodyGetPosition(body_id);
//        const dReal* dir = dBodyGetQuaternion(body_id);
//        ofs.write(reinterpret_cast<const char*>(pos), sizeof(dReal) * 3);
//        ofs.write(reinterpret_cast<const char*>(dir), sizeof(dReal) * 4);
//    };

    for(const auto& figure : scene->creature_->get_figures())
        figure->save(ofs);

    for(const auto& figure : stepping_figures)
        figure.save(ofs);

    ofs.close();

    if(bnn::state::started == state)
        start();
}

void application::save_random()
{
#if(0)
    auto state = state_;

    stop();

    //std::ofstream ofs(fs::current_path() / "random.rnd", std::ios::binary);

    creature_->brain_->save_random();

    //ofs.close();

    //    std::ofstream ofs_csv(fs::current_path() / "random.csv", std::ios::binary);

    //    creature_->brain_->save_random_csv(ofs_csv);

    //    ofs_csv.close();

    if(bnn::state::started == state)
        start();
#endif
}

void application::start()
{
    log_place

    if(bnn::state::stopped != state_)
        return;

    scene->start();
    state_ = bnn::state::start;

    while(bnn::state::start == state_)
        std::this_thread::sleep_for(device_3d_LITTLE_TIMEms);

    log_place
}

void application::stop()
{
    log_place

    if(bnn::state::started != state_)
        return;

    state_ = bnn::state::stop;

    while(bnn::state::stop == state_)
        std::this_thread::sleep_for(device_3d_LITTLE_TIMEms);

    scene->stop();

    log_place
}

void application::setup()
{
    OgreBites::ApplicationContext::setup();
    addInputListener(this);

    switch(config_.device_3d_.scene_)
    {
    case config::device_3d::scene::bike_scene:
        scene.reset(new scn::bike::bike(getRenderWindow(), getRoot()->createSceneManager()));
        break;
    case config::device_3d::scene::table_scene:
        scene.reset(new scn::table::table(getRenderWindow(), getRoot()->createSceneManager()));
        break;
    default:
        exit(~0);
    }

    scene->setup(
                stationary_colliding_geoms,
                movable_colliding_geoms,
                creature_colliding_geoms,
                bounding_nodes,
                stepping_figures,
                world,
                config_.device_3d_.bnn_
                );
}

void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    //    int i,n;
    //    cout << "aaaaaaaaaaaaaaa" << endl;
    //    const int N = 10;
    //    dContact contact[N];
    //    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    //    if (n > 0)
    //    {
    //        cout << to_string(n) << endl;
    //        for (i=0; i<n; i++)
    //        {
    //            contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
    //                    dContactSoftERP | dContactSoftCFM | dContactApprox1;
    //            contact[i].surface.mu = dInfinity;
    //            contact[i].surface.slip1 = 0.1;
    //            contact[i].surface.slip2 = 0.1;
    //            contact[i].surface.soft_erp = 0.5;
    //            contact[i].surface.soft_cfm = 0.3;
    //            dJointID c = dJointCreateContact (world_st, contactgroup_st, &contact[i]);
    //            dJointAttach (c,
    //                          dGeomGetBody(contact[i].geom.g1),
    //                          dGeomGetBody(contact[i].geom.g2));
    //        }
    //    }
    //    cout << "bbbbbbbbbbbb" << endl;
    return;

    // TODO
    if (dGeomIsSpace (o1) || dGeomIsSpace (o2))
    {
        // colliding a space with something :
        dSpaceCollide2 (o1, o2, data, &nearCallback);

        // collide all geoms internal to the space(s)
        if (dGeomIsSpace (o1))
            dSpaceCollide (reinterpret_cast<dSpaceID>(o1), data, &nearCallback);

        if (dGeomIsSpace (o2))
            dSpaceCollide (reinterpret_cast<dSpaceID>(o2), data, &nearCallback);
    }
    else
    {
        //       #define max_contacts 100;
        //        #define contact_array 100;
        //        typedef struct dContactGeom {
        //            dVector3 pos;          /*< contact position*/
        //            dVector3 normal;       /*< normal vector*/
        //            dReal depth;           /*< penetration depth*/
        //            dGeomID g1,g2;         /*< the colliding geoms*/
        //            int side1,side2;       /*< (to be documented)*/
        //        } dContactGeom;
        //dContactGeom cg[5];
        //        cg->g1 = o1;
        //        cg->g2 = o2;
        // colliding two non-space geoms, so generate contact
        // points between o1 and o2
        //return;
        try
        {
            //int num_contact = dCollide (o1, o2, 1, cg, 1);
            const int N = 32;
            dContact contact[N];
            int number_of_contact_points = dCollide(o1, o2, N, &(contact[0].geom), sizeof(dContact));
            log_verbose("number of contact points [%d]", number_of_contact_points);

            if(number_of_contact_points > 0)
            {
                for(int i = 0; i < 1; i++)
                {
                    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
                    contact[i].surface.mu = dInfinity;
                    contact[i].surface.mu2 = 5;
                    contact[i].surface.bounce = 0.5;
                    contact[i].surface.bounce_vel = 0.5;
                    contact[i].surface.soft_cfm = 0.5;
                    //                    contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                    //                            dContactSoftERP | dContactSoftCFM | dContactApprox1;
                    //                    contact[i].surface.mu = dInfinity;
                    //                    contact[i].surface.slip1 = 0.1;
                    //                    contact[i].surface.slip2 = 0.1;
                    //                    contact[i].surface.soft_erp = 0.5;
                    //                    contact[i].surface.soft_cfm = 0.3;
                    auto app = (bnn_device_3d::application::application*)data;
                    dJointID c = dJointCreateContact(app->get_world(), app->get_gontact_group(), &contact[i]);
                    dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
                }
                //                for (int i=0; i<n; i++)
                //                {
                //                    cout << "ccccssss = " << contact << endl;
                //                    //cout << "ccccssss = " << to_string(contact[i].geom.g1.depth) << endl;
                //                    //contact[i].geom
                //                    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
                //                    contact[i].surface.mu = dInfinity;
                //                    contact[i].surface.mu2 = 0;
                //                    contact[i].surface.bounce = 0.01;
                //                    contact[i].surface.bounce_vel = 0.1;
                //                    contact[i].surface.soft_cfm = 0.01;
                //                    //           contact[i].surface.slip1 = 0.7;
                //                    //           contact[i].surface.slip2 = 0.7;
                //                    //           contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
                //                    //           contact[i].surface.mu = 50.0; // was: dInfinity
                //                    //           contact[i].surface.soft_erp = 0.96;
                //                    //           contact[i].surface.soft_cfm = 0.04;
                //                    //           dJointID c = dJointCreateContact(World, contactgroup, contact + i);
                //                    //                       dJointAttach(c, b1, b2);
                //                    //dJointID c = dJointCreateContact (world_st, contactgroup_st, &contact[i]);
                //                    //dJointID c = dJointCreateUniversal (world_st, contactgroup_st);
                //                    //dJointID c = dJointCreateHinge2 (world_st, contactgroup_st);
                //                    //                    dJointID c = dJointCreatePR (world_st, contactgroup_st);
                //                    //                    dJointAttach (c,
                //                    //                                  dGeomGetBody(contact[i].geom.g1),
                //                    //                                  dGeomGetBody(contact[i].geom.g2));
                //                }

            }
        }
        catch (...)
        {
            log_error("in:");
            log_place
        }
    }
}

} // namespace bnn_device_3d::application
