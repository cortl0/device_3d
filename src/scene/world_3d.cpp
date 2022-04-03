/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "world_3d.h"

#include <unistd.h>

#include "config.hpp"

namespace sch = std::chrono;

typedef sch::time_point<sch::system_clock, sch::microseconds> m_time_point;

namespace bnn_device_3d::scene
{

static dWorldID world_st;

static dJointGroupID contactgroup_st;

world_3d::~world_3d()
{
    logging("");
    //while(bnn::state::stopped != state_)
    //    usleep(BNN_LITTLE_TIME);
    //stop();
//    if(!shutdown)
//        getRoot()->queueEndRendering();
    //    dWorldDestroy (world);
    //            dCloseODE();
}

world_3d::world_3d() : OgreBites::ApplicationContext("bnn_test_app")
{

}

void world_3d::collide_action2(world_3d *me, dGeomID o1, dGeomID o2)
{
    int i,n;
    const int N = 1;
    dContact contact[N];
    n = dCollide (o1, o2, N, &contact[0].geom, sizeof(dContact));
    for (i=0; i<n; i++)
    {
        contact[i].surface.mode =
                0
                | dContactMu2
                //                        | dContactSlip1
                //                        | dContactSlip2
                //                        | dContactSoftERP
                //                        | dContactSoftCFM
                //                        | dContactApprox1
                //| dContactApprox1_N
                //                        | dContactBounce


                //                        | dContactMu2
                //                        | dContactAxisDep
                //                        | dContactFDir1
                //                        | dContactBounce
                //                        | dContactSoftERP
                //                        | dContactSoftCFM
                //                        | dContactMotion1
                //                        | dContactMotion2
                //                        | dContactMotionN
                //                        | dContactSlip1
                //                        | dContactSlip2
                //                        | dContactRolling
                //                        | dContactApprox0
                //                        | dContactApprox1_11
                //                        | dContactApprox1_2
                //                        | dContactApprox1_N
                //                        | dContactApprox1


                ;
        //        contact[i].surface.mu = dInfinity;
        //        contact[i].surface.mu2 = dInfinity;


        contact[i].surface.mu = 4;
        contact[i].surface.mu2 = 4;


        //                contact[i].surface.bounce = 0.1f;
        //                contact[i].surface.bounce_vel = 0.0f;
        //                contact[i].surface.slip1 = 1;
        //                contact[i].surface.slip2 = 1;
        //                contact[i].surface.soft_erp = 1.0f;
        //                contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact (world_st, me->contactgroup, &contact[i]);
        dJointAttach (c,
                      dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
}

void world_3d::collide_action()
{
    for_each(stationary_colliding_geoms.begin(), stationary_colliding_geoms.end(), [&](dGeomID sg)
    {
        for_each(movable_colliding_geoms.begin(), movable_colliding_geoms.end(), [&](dGeomID mg)
        {
            collide_action2(this, sg, mg);
        });

        for_each(creature_colliding_geoms.begin(), creature_colliding_geoms.end(), [&](dGeomID cg)
        {
            collide_action2(this, sg, cg);
        });
    });

    auto it_0 = movable_colliding_geoms.begin();
    while (it_0 != movable_colliding_geoms.end())
    {
        auto it_1 = it_0;
        it_1++;
        while (it_1 != movable_colliding_geoms.end())
        {
            collide_action2(this, *it_0, *it_1);
            it_1++;
        }
        it_0++;
    }

    for_each(creature_colliding_geoms.begin(), creature_colliding_geoms.end(), [&](dGeomID sg)
    {
        for_each(movable_colliding_geoms.begin(), movable_colliding_geoms.end(), [&](dGeomID mg)
        {
            collide_action2(this, sg, mg);
        });
    });
}

bool world_3d::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    float force = 100;
    switch (evt.keysym.sym)
    {
    case OgreBites::SDLK_DOWN:
    {
        //std::cout << "SDLK_DOWN" << std::endl;
        auto it = stepping_figures.begin();
        std::advance(it, 4);
        dBodyAddForce(it->body, 0, 0, +force);
        break;
    }
    case OgreBites::SDLK_UP:
    {
        //std::cout << "SDLK_UP" << std::endl;
        auto it = stepping_figures.begin();
        std::advance(it, 4);
        dBodyAddForce(it->body, 0, 0, -force);
        break;
    }
    case OgreBites::SDLK_LEFT:
    {
        //std::cout << "SDLK_LEFT" << std::endl;
        auto it = stepping_figures.begin();
        std::advance(it, 4);
        dBodyAddForce(it->body, -force, 0, 0);
        break;
    }
    case OgreBites::SDLK_RIGHT:
    {
        //std::cout << "SDLK_RIGHT" << std::endl;
        auto it = stepping_figures.begin();
        std::advance(it, 4);
        dBodyAddForce(it->body, +force, 0, 0);
        break;
    }
    }

    return true;
}

bool world_3d::keyReleased(const OgreBites::KeyboardEvent& evt)
{
    float force = 500.0;

    switch (evt.keysym.sym)
    {
    case OgreBites::SDLK_ESCAPE:
        getRoot()->queueEndRendering();
        break;
    case config::keyboard_key_c: // load
        load();
        break;
    case config::keyboard_key_z: // save
        save();
        break;
    case config::keyboard_key_r: // save random
        save_random();
        break;
    case config::keyboard_key_x: // stop <-> start
        if(bnn::state::started == state_)
            stop();
        else if(bnn::state::stopped == state_)
            start();
        break;
    case config::keyboard_key_v: // verbose
        //getRenderWindow()->writeContentsToFile("filename8.png");
        if(!verbose)
            verbose = true;
        break;
    case config::keyboard_key_a: // left
        dBodyAddForce(creature_->body.body, force, 0, 0);
        break;
    case config::keyboard_key_d: // right
        dBodyAddForce(creature_->body.body, -force, 0, 0);
        break;
    case config::keyboard_key_w: // up
        dBodyAddForce(creature_->body.body, 0, 0, -force);
        break;
    case config::keyboard_key_s: // down
        dBodyAddForce(creature_->body.body, 0, 0, force);
        break;
    }

    return true;
}

void create_light(Ogre::SceneManager* scnMgr, Real x, Real y, Real z, const std::string name)
{
    Ogre::Light* light = scnMgr->createLight(name);
    Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->setPosition(x, y, z);
    lightNode->attachObject(light);
    //light->setSpecularColour(0.9, 0.5, 0.5);
    //light->setDiffuseColour(1, 1, 1);
    //light->setPowerScale(-100);
    //light->setSpotlightFalloff(0.1f);
}

void world_3d::setup_ogre()
{
    // do not forget to call the base first
    OgreBites::ApplicationContext::setup();

    // register for input events
    addInputListener(this);

    // get a pointer to the already created root
    root = getRoot();
    scnMgr = root->createSceneManager();

    Ogre::RTShader::ShaderGenerator* shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    create_light(scnMgr, 10000, 10000, 10000, "MainLight0");
    create_light(scnMgr, -10000, 10000, 10000, "MainLight1");
    create_light(scnMgr, -10000, 10000, -10000, "MainLight2");
    create_light(scnMgr, 10000, 10000, -10000, "MainLight3");
    create_light(scnMgr, 0, -10000, 0, "MainLight4");

    // also need to tell where we are
    third_person_camera_node = scnMgr->getRootSceneNode()->createChildSceneNode();
    creature_camera_node = scnMgr->getRootSceneNode()->createChildSceneNode();

    switch (0)
    {
    case 0:
        third_person_camera_node->setPosition(0, 400 * device_3d_SCALE, 1000 * device_3d_SCALE);
        break;
    case 1:
        third_person_camera_node->setPosition(0, 800, 0);
        third_person_camera_node->setOrientation(Quaternion(-f, f, 0, 0));
        break;
    }

    third_person_camera = scnMgr->createCamera("myCam");
    third_person_camera->setNearClipDistance(1);
    third_person_camera->setAutoAspectRatio(true);
    third_person_camera_node->attachObject(third_person_camera);

    creature_camera = scnMgr->createCamera("myCam1");
    creature_camera->setNearClipDistance(0.1);
    creature_camera->setAutoAspectRatio(true);
    creature_camera_node->attachObject(creature_camera);

    getRenderWindow()->addViewport(third_person_camera, 0, 0.0f, 0.0f, 1.0f, 1.0f);
    getRenderWindow()->addViewport(creature_camera, 1, 0.0f, 0.0f, 0.25f, 0.25f);

    getRenderWindow()->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    getRenderWindow()->getViewport(1)->setBackgroundColour(Ogre::ColourValue::White);

    auto *ent_plane = scnMgr->createEntity("plane", Ogre::SceneManager::PrefabType::PT_PLANE);
    auto *node_plane = scnMgr->getRootSceneNode()->createChildSceneNode();
    node_plane->setScale(Ogre::Vector3(1, 1, 1));
    node_plane->setDirection(0,-1,0);
    node_plane->attachObject(ent_plane);
    ent_plane->setMaterial(pho::figure::create_material_chess(1024, 8, COLOR_BLACK, COLOR_DARK));
}

void world_3d::setup_ode()
{
    dInitODE2(0);
    world = dWorldCreate();
    dWorldSetGravity (world, 0, -device_3d_GRAVITY, 0);

    space = dSimpleSpaceCreate(nullptr);

    contactgroup = dJointGroupCreate (0);
    contactgroup_st = contactgroup;
    world_st = world;

    {
        //        auto fff = dWorldGetERP(world);
        //        auto fff1 = dWorldGetCFM(world);
        //dWorldSetERP(world, 0.2f);
        //dWorldSetCFM(world, 1e-5);

        //for_each(walls_list.begin(), walls_list.end(), [&](std::unique_ptr<figure>& fig){ fig->step(); });

        //    figure_list.push_back(std::unique_ptr<cube>(new cube("cube", scnMgr, world, space, 1.0f, 100.0f, 100.0f, 100.0f)));
        //    dBodySetPosition (figure_list.back()->body, 0, 200, -800);

        //    {
        //        dJointID c;
        //        //        c = dJointCreateBall (world_st, contactgroup_st);
        //        //        c = dJointCreateHinge (world_st, contactgroup_st);
        //        //        c = dJointCreateSlider (world_st, contactgroup_st);
        //        //        c = dJointCreateContact (world_st, contactgroup_st, &contact[i]);
        //        //        c = dJointCreateHinge2 (world_st, contactgroup_st);
        //        //        c = dJointCreateUniversal (world_st, contactgroup_st);
        //        //        c = dJointCreatePR (world_st, contactgroup_st);
        //        //        c = dJointCreatePU (world_st, contactgroup_st);
        //        //        c = dJointCreatePiston (world_st, contactgrodBodySetPosition (walls_list.back()->bodyup_st);
        //        //        c = dJointCreateFixed (world_st, contactgroup_st);

        //        //        c = dJointCreateNull (world_st, contactgroup_st);
        //        //        c = dJointCreateAMotor (world_st, contactgroup_st);
        //        //        c = dJointCreateLMotor (world_st, contactgroup_st);
        //        //        c = dJointCreatePlane2D (world_st, contactgroup_st);

        //                c = dJointCreateDBall (world_st, contactgroup_st1);
        //        //        c = dJointCreateDHinge (world_st, contactgroup_st);
        //        //        c = dJointCreateTransmission (world_st, contactgroup_st);


        //                        dJointAttach (c,
        //                                      dGeomGetBody(sphr_0.geom),
        //                                      dGeomGetBody(sphr_1.geom));

        //    }

        //dSpaceCollide (space,0,&nearCallback);
    }
}

void world_3d::load()
{
    auto state = state_;

    stop();

    std::list<std::string> l;

    for (auto & p : fs::directory_iterator(fs::current_path()))
        if(p.path().filename().extension() == ".bnn")
            l.push_back(p.path().filename());

    if(!l.size())
        return;

    l.sort();

    std::ifstream ifs(fs::current_path() / l.back(), std::ios::binary);

    auto load_figure = [&ifs](dBodyID id)
    {
        double pos[3];
        ifs.read(reinterpret_cast<char*>(pos), sizeof(pos));
        dBodySetPosition(id, pos[0], pos[1], pos[2]);

        dReal dir[4];
        ifs.read(reinterpret_cast<char*>(dir), sizeof(dir));
        dBodySetQuaternion(id, dir);
    };

    load_figure(tripod_->detector);
    load_figure(tripod_->upper);
    load_figure(tripod_->lower);

    auto figures = creature_->get_figures();
    for_each(figures.begin(), figures.end(), [&](const pho::figure* f) { load_figure(f->body); });

    for_each(stepping_figures.begin(), stepping_figures.end(), [&](const pho::figure& f) { load_figure(f.body); });

    if(creature_->brain_->load(ifs))
    {
        std::cout << "loaded" << std::endl;
        if(l.size() > 8)
            fs::remove_all(fs::current_path() / l.front());
    }
    else
        std::cout << "load error" << std::endl;

    ifs.close();

    if(bnn::state::started == state)
        start();
}

void world_3d::run()
{
    initApp();

    try
    {
        start();

        auto width = getRenderWindow()->getWidth();
        auto height = getRenderWindow()->getHeight();
        long delta;
        const long frame_length = 1000000 / 60;
        const dReal frame_length_dReal = (dReal)frame_length / 1000000.0;

        while(bnn::state::started != state_)
            usleep(BNN_LITTLE_TIME);

        m_time_point current_time = sch::time_point_cast<m_time_point::duration>(sch::system_clock::time_point(sch::system_clock::now()));
        m_time_point time_old = current_time;

        while(!getRoot()->endRenderingQueued())
        {
            if(bnn::state::started == state_)
            {
                dJointGroupEmpty(contactgroup);

                for(auto& figure : stepping_figures)
                    figure.step();

                {
                    auto it = stepping_figures.begin();
                    std::advance(it, 5);
                    auto *pos = dBodyGetPosition(it->body);
                    float x = pos[0];
                    float y = pos[1];
                    float z = pos[2];
                    conductor_->step(x, y, z);
                    auto *vel = dBodyGetLinearVel(it->body);
                    dBodyAddForce(it->body, x - vel[0], y - vel[1], z - vel[2]);
                }

                creature_->step(movable_colliding_geoms, verbose);

                tripod_->step();

                const dReal* pos_l = dBodyGetPosition(creature_->legs[NUMBER_OF_FRONT_LEFT_LEG].first.body);
                const dReal* pos_r = dBodyGetPosition(creature_->legs[NUMBER_OF_FRONT_RIGHT_LEG].first.body);
                const dReal* pos_b = dBodyGetPosition(creature_->body.body);

                dReal pos[3];
                for(int i = 0; i < 3; i++)
                {
                    pos[i] = (pos_l[i] + pos_r[i]) / 2;
                    pos[i] = (pos[i] - pos_b[i]) / 10 * 8.75 + pos_b[i];
                }

                const dReal* dir = dBodyGetQuaternion(creature_->body.body);
                creature_camera_node->setPosition(static_cast<Ogre::Real>(pos[0]), static_cast<Ogre::Real>(pos[1]), static_cast<Ogre::Real>(pos[2]));
                creature_camera_node->setOrientation(static_cast<Ogre::Real>(dir[0]), static_cast<Ogre::Real>(dir[1]), static_cast<Ogre::Real>(dir[2]), static_cast<Ogre::Real>(dir[3]));

                for_each(bounding_nodes.begin(), bounding_nodes.end(), [&](Ogre::SceneNode* node){ node->_updateBounds(); });

                collide_action();

                dWorldStep(world, frame_length_dReal);
            }

            //***************

            getRoot()->renderOneFrame();

            {
                static Image img(getRenderWindow()->suggestPixelFormat(), width, height);
                static PixelBox pb = img.getPixelBox();

                getRenderWindow()->copyContentsToMemory(pb, pb);

                Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pb.data);
                creature_->video_->calculate_data(pDest, width, height);

                //img.save("filename5.png");
            }

            current_time = sch::time_point_cast<m_time_point::duration>(sch::system_clock::time_point(sch::system_clock::now()));
            delta = (current_time - time_old).count();

            if(delta < frame_length)
                usleep(frame_length - delta);

            time_old += sch::microseconds((long long int)frame_length);
        }

        stop();

        //        dWorldDestroy (world);
        //        dCloseODE();
    }
    catch (...)
    {
        logging("unknown error");
    }

    closeApp();
}

void world_3d::save()
{
    auto state = state_;

    stop();

    static char time_buffer[15];
    std::time_t time = std::time(nullptr);
    std::strftime(time_buffer, 15, "%Y%m%d%H%M%S", std::localtime(&time));
    std::string s(time_buffer);
    s += ".bnn";
    std::ofstream ofs(fs::current_path() / s, std::ios::binary);

    auto save_figure = [&ofs](dBodyID id)
    {
        auto* pos = dBodyGetPosition(id);
        ofs.write(reinterpret_cast<const char*>(pos), sizeof(pos) * 3);

        auto* dir = dBodyGetQuaternion(id);
        ofs.write(reinterpret_cast<const char*>(dir), sizeof(dir) * 4);
    };

    save_figure(tripod_->detector);
    save_figure(tripod_->upper);
    save_figure(tripod_->lower);

    auto figures = creature_->get_figures();
    for_each(figures.begin(), figures.end(), [&](const pho::figure* f) { save_figure(f->body); });

    for_each(stepping_figures.begin(), stepping_figures.end(), [&](const pho::figure& f) { save_figure(f.body); });

    if(creature_->brain_->save(ofs))
        std::cout << "saved" << std::endl;
    else
        std::cout << "save error" << std::endl;

    ofs.close();

    if(bnn::state::started == state)
        start();
}

void world_3d::save_random()
{
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
}

void world_3d::start()
{
    logging("world_3d::start() begin");

    if(bnn::state::stopped != state_)
        return;

    state_ = bnn::state::start;

    creature_->start();

    state_ = bnn::state::started;

    logging("world_3d::start() end");
}

void world_3d::stop()
{
    logging("world_3d::stop() begin");

    if(bnn::state::started != state_)
        return;

    state_ = bnn::state::stop;

    creature_->stop();

    state_ = bnn::state::stopped;

    logging("world_3d::stop() end");
}

void world_3d::fill_it_up()
{
    conductor_.reset(new cond::conductor_circle());

    // creating stationary objects
    {
        plane = dCreatePlane(space, 0.0f, 1.0f, 0.0f, 0.0f);

        stationary_colliding_geoms.push_back(plane);

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
                                                 scnMgr, world, space, x * y * z * device_3d_MASS_SCALE,
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

    // creating movable objects
    {
        {
            float r = 120.0 * device_3d_SCALE;// ((static_cast<float>(rand()) / RAND_MAX) * 20 + 100) * device_3d_SCALE;

            stepping_figures.push_back(pho::cube("cube_qqq", scnMgr, world, space,
                                                 r*r*r * device_3d_MASS_SCALE, r, r, r));

            stepping_figures.back().set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));

            dBodySetPosition(stepping_figures.back().body,
                             ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 2 * 500 * device_3d_SCALE,
                             ((static_cast<float>(rand()) / RAND_MAX)) * 500 * device_3d_SCALE,
                             ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 2 * 500 * device_3d_SCALE);

            movable_colliding_geoms.push_back(stepping_figures.back().geom);

            bounding_nodes.push_back(stepping_figures.back().node);
        }

        if(1)
        {
            float r = ((static_cast<float>(rand()) / RAND_MAX) * 20 + 50) * device_3d_SCALE;

            stepping_figures.push_back(pho::sphere("sphere_ssss", scnMgr, world, space, r*r*r, r));

            stepping_figures.back().set_material(pho::figure::create_material_chess(256, 32, COLOR_MEDIUM, COLOR_LIGHT));

            dBodySetPosition(stepping_figures.back().body, 0, 1, -10);

            movable_colliding_geoms.push_back(stepping_figures.back().geom);

            bounding_nodes.push_back(stepping_figures.back().node);
        }

        for(int i = 0; i < 7; i++)
        {
            float r = ((static_cast<float>(rand()) / RAND_MAX) * 50 + 50) * device_3d_SCALE;

            stepping_figures.push_back(pho::sphere("sphere_" + std::to_string(i), scnMgr, world, space,
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
        creature_.reset(new creatures::creature(getRenderWindow(), scnMgr, world));

        creature_->set_position(0, 1, 0);

        creature_colliding_geoms.push_back(creature_->body.geom);

        bounding_nodes.push_back(creature_->body.node);

        bounding_nodes.push_back(creature_->body_sign.node);

        for (size_t i = 0; i < creature_->legs.size(); i++)
        {
            creature_colliding_geoms.push_back(creature_->legs[i].first.geom);
            creature_colliding_geoms.push_back(creature_->legs[i].second.geom);
            creature_colliding_geoms.push_back(creature_->legs[i].third.geom);

            bounding_nodes.push_back(creature_->legs[i].first.node);
            bounding_nodes.push_back(creature_->legs[i].second.node);
            bounding_nodes.push_back(creature_->legs[i].third.node);
        }

        if(0)
        {
            dJointGroupID gc_body = dJointGroupCreate(0);
            dJointID j_body = dJointCreateFixed(world, gc_body);
            dJointAttach (j_body, 0, dGeomGetBody(creature_->body.geom));
            dJointSetFixed(j_body);
        }

        //bool ff = creature_->body.node->getShowBoundingBox();
    }

    auto *body = creature_->body.body;

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

    tripod_.reset(new tripod(world, third_person_camera_node, body));
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
    if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) {
        std::cout << "bbbbbbbbbbb" << std::endl;
        // colliding a space with something :
        dSpaceCollide2 (o1, o2, data, &nearCallback);

        // collide all geoms internal to the space(s)
        if (dGeomIsSpace (o1))
            dSpaceCollide (reinterpret_cast<dSpaceID>(o1), data, &nearCallback);
        if (dGeomIsSpace (o2))
            dSpaceCollide (reinterpret_cast<dSpaceID>(o2), data, &nearCallback);

    } else {

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
        std::cout << "ccccccccccccc" << std::endl;
        // colliding two non-space geoms, so generate contact
        // points between o1 and o2
        //return;
        try {
            //int num_contact = dCollide (o1, o2, 1, cg, 1);
            const int N = 32;
            dContact contact[N];



            int n = dCollide (o1, o2, N, &(contact[0].geom), sizeof(dContact));
            std::cout << "fffffffffffff" << std::endl;
            std::cout << std::to_string(n) << std::endl;
            if (n > 0)
            {
                for (int i=0; i<1; i++) {
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
                    dJointID c = dJointCreateContact (world_st, contactgroup_st, &contact[i]);
                    dJointAttach (c,
                                  dGeomGetBody(contact[i].geom.g1),
                                  dGeomGetBody(contact[i].geom.g2));
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

        } catch (...) {
            std::cout << "try" << std::endl;
        }
        // add these contact points to the simulation ...
        std::cout << "ccccccccccccc12" << std::endl;
        //cout << to_string(num_contact) << endl;
    }
}

void world_3d::setup(void)
{
    setup_ogre();
    setup_ode();
    fill_it_up();
}

} // namespace bnn_device_3d::scene
