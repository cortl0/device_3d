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

#include <lib/logger/src/helpers/log.h>

#include "config.hpp"
#include "conductors/conductor_circle.h"
#include "conductors/kick.h"
#include "conductors/tail.h"
#include "conductors/dream.h"
#include "scenes/bike_scene.h"
#include "scenes/table_scene.h"

namespace fs = std::filesystem;

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

void application::run()
{
    namespace sch = std::chrono;
    typedef sch::time_point<sch::system_clock, sch::microseconds> m_time_point;

    initApp();

    try
    {
        // auto width = getRenderWindow()->getWidth();
        // auto height = getRenderWindow()->getHeight();
        long delta;
        const long frame_length = 1000000 / 60;
        const dReal frame_length_dReal = (dReal)frame_length / 1000000.0 * config_.device_3d_.time_coefficient;
        m_time_point current_time = sch::time_point_cast<m_time_point::duration>(sch::system_clock::time_point(sch::system_clock::now()));
        m_time_point time_old = current_time;
        scene->step(keys_states_, frame_length_dReal);
        scene->creature_->update_visual();
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

            scene->step(keys_states_, frame_length_dReal);
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

    if(l.size() > 8)
        fs::remove_all(fs::current_path() / l.front());

    std::ifstream ifs(fs::current_path() / l.back(), std::ios::binary);
    scene->load(ifs);

//    auto load_figure = [&ifs](dBodyID body_id)
//    {
//        dReal pos[3];
//        dReal dir[4];
//        ifs.read(reinterpret_cast<char*>(pos), sizeof(dReal) * 3);
//        ifs.read(reinterpret_cast<char*>(dir), sizeof(dReal) * 4);
//        dBodySetPosition(body_id, pos[0], pos[1], pos[2]);
//        dBodySetQuaternion(body_id, dir);
//    };

    ifs.close();

    if(bnn::state::started == state)
        start();
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

//    auto save_figure = [&ofs](dBodyID body_id)
//    {
//        const dReal* pos = dBodyGetPosition(body_id);
//        const dReal* dir = dBodyGetQuaternion(body_id);
//        ofs.write(reinterpret_cast<const char*>(pos), sizeof(dReal) * 3);
//        ofs.write(reinterpret_cast<const char*>(dir), sizeof(dReal) * 4);
//    };

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
    namespace scn = bnn_device_3d::scenes;

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

    scene->setup(config_.device_3d_.bnn_);
}

} // namespace bnn_device_3d::application