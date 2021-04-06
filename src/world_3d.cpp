#include "world_3d.h"

world_3d::world_3d() : OgreBites::ApplicationContext("bnn_test_app")
{
}

void collide_action2(dGeomID o1, dGeomID o2)
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
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = dInfinity;
        //                contact[i].surface.bounce = 0.1f;
        //                contact[i].surface.bounce_vel = 0.0f;
        //                contact[i].surface.slip1 = 1;
        //                contact[i].surface.slip2 = 1;
        //                contact[i].surface.soft_erp = 1.0f;
        //                contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact (world_st, contactgroup_st, &contact[i]);
        dJointAttach (c,
                      dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
}

void world_3d::collide_action()
{
    dJointGroupEmpty (contactgroup_st);

    for_each(stationary_colliding_geoms.begin(), stationary_colliding_geoms.end(), [&](dGeomID sg)
    {
        for_each(movable_colliding_geoms.begin(), movable_colliding_geoms.end(), [&](dGeomID mg)
        {
            collide_action2(sg, mg);
        });

        for_each(creature_colliding_geoms.begin(), creature_colliding_geoms.end(), [&](dGeomID cg)
        {
            collide_action2(sg, cg);
        });
    });

    auto it_0 = movable_colliding_geoms.begin();
    while (it_0 != movable_colliding_geoms.end())
    {
        auto it_1 = it_0;
        it_1++;
        while (it_1 != movable_colliding_geoms.end())
        {
            collide_action2(*it_0, *it_1);
            it_1++;
        }
        it_0++;
    }

    for_each(creature_colliding_geoms.begin(), creature_colliding_geoms.end(), [&](dGeomID sg)
    {
        for_each(movable_colliding_geoms.begin(), movable_colliding_geoms.end(), [&](dGeomID mg)
        {
            collide_action2(sg, mg);
        });
    });
}

static bool keyboard_key_flag_a = false;
static bool keyboard_key_flag_w = false;
static bool keyboard_key_flag_s = false;
static bool keyboard_key_flag_d = false;
static bool keyboard_key_flag_lshift = false;
static bool keyboard_key_flag_space = false;

void world_3d::setup_ogre()
{
    // do not forget to call the base first
    OgreBites::ApplicationContext::setup();
    // register for input events
    addInputListener(this);
    // get a pointer to the already created root
    root = getRoot();
    scnMgr = root->createSceneManager();
    // register    //    for(int i = 0; i < 50; i++)
    //    {
    //        //((float)rand() / RAND_MAX) * 400;
    //        figure_list.push_back(std::unique_ptr<sphere>(new sphere("sphere_" + std::to_string(i), scnMgr, world, space,
    //                                                                 1.0f, ((float)rand() / RAND_MAX) * 70 + 10)));
    //        dBodySetPosition (figure_list.back()->body,
    //                          (((float)rand() / RAND_MAX) - 0.5f) * 2 * 1000,
    //                          (((float)rand() / RAND_MAX)) * 1000,
    //                          (((float)rand() / RAND_MAX) - 0.5f) * 2 * 1000);
    //    } our scene with the RTSS
    Ogre::RTShader::ShaderGenerator* shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    //dAllocateODEDataForThread(dAllocateMaskAll);
    Ogre::ResourceGroupManager::getSingletonPtr()->createResourceGroup("name1");
    Ogre::ResourceGroupManager::getSingletonPtr()->addResourceLocation("../my_first", "FileSystem", "name1", false);
    Ogre::ResourceGroupManager::getSingletonPtr()->initialiseResourceGroup("name1");
    Ogre::ResourceGroupManager::getSingletonPtr()->loadResourceGroup("name1");

    {
        //**
        //getRenderWindow()->setFullscreen(true, 1920, 1080);
        //**
    }

    //    MyGUI::PointerManager::getInstance().setVisible(false);
    {
        //        Ogre::NameValuePairList opts;
        //        opts.insert( Ogre::NameValuePairList::value_type( "resolution", "1920x1080" ) );
        //        opts.insert( Ogre::NameValuePairList::value_type( "fullscreen", "true" ) );
        //        opts.insert( Ogre::NameValuePairList::value_type( "vsync", "true" ) );
        //        getRoot()->createRenderWindow("TestWin", 1920, 1080, true, &opts );
    }

    {
        Ogre::Light* light = scnMgr->createLight("MainLight0");
        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->setPosition(10000, 10000, 10000);
        lightNode->attachObject(light);
        //light->setSpecularColour(0.9, 0.5, 0.5);
        //light->setDiffuseColour(1, 1, 1);
        //light->setPowerScale(-100);
        //light->setSpotlightFalloff(0.1f);
    }
    {
        Ogre::Light* light = scnMgr->createLight("MainLight1");
        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->setPosition(-10000, 10000, 10000);
        lightNode->attachObject(light);
        //light->setSpecularColour(0.9, 0.5, 0.5);
        //light->setDiffuseColour(1, 1, 1);
        //light->setPowerScale(-100);
        //light->setSpotlightFalloff(0.1f);
    }
    {
        Ogre::Light* light = scnMgr->createLight("MainLight2");
        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->setPosition(-10000, 10000, -10000);
        lightNode->attachObject(light);
        //light->setSpecularColour(0.9, 0.5, 0.5);
        //light->setDiffuseColour(1, 1, 1);
        //light->setPowerScale(-100);
        //light->setSpotlightFalloff(0.1f);
    }
    {
        Ogre::Light* light = scnMgr->createLight("MainLight3");
        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->setPosition(10000, 10000, -10000);
        lightNode->attachObject(light);
        //light->setSpecularColour(0.9, 0.5, 0.5);
        //light->setDiffuseColour(1, 1, 1);
        //light->setPowerScale(-100);
        //light->setSpotlightFalloff(0.1f);
    }
    {
        // without light we would just get a black screen
        Ogre::Light* light = scnMgr->createLight("MainLight4");
        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->setPosition(0, -10000, 0);
        lightNode->attachObject(light);
        //light->setSpecularColour(0.9, 0.5, 0.5);
        //light1->setDiffuseColour(1, 1, 1);
        //light1->setPowerScale(-100.0f);
        //light1->setSpotlightFalloff(0.1f);
    }

    // also need to tell where we are
    camNode = scnMgr->getRootSceneNode()->createChildSceneNode();

    switch (0)
    {
    case 0:
        camNode->setPosition(0, 400 * scale, 1000 * scale);
        break;
    case 1:
        camNode->setPosition(0, 800, 0);
        camNode->setOrientation(Quaternion(-f, f, 0, 0));
        break;
    }

    //camNode->lookAt(Ogre::Vector3(0, 0, -1), Ogre::Node::TS_PARENT);

    // create the camera
    cam = scnMgr->createCamera("myCam");
    cam->setNearClipDistance(50); // specific to this sample

    cam->setAutoAspectRatio(true);

    //cam->setUseRenderingDistance(false);
    //auto dd = cam->projectSphere();

    //    ogreMesh->_setBoundingSphereRadius(Ogre::Math::Sqrt(this->maxRadius)); /// this is actually squared radius
    //    ogreMesh->_setBounds(Ogre::AxisAlignedBox(this->minCorner, this->maxCorner),false);


    camNode->attachObject(cam);
    camNode = camNode;
    // and tell it to render into the main window
    getRenderWindow()->addViewport(cam);


    auto ent_plane = scnMgr->createEntity("plane", Ogre::SceneManager::PrefabType::PT_PLANE);
    auto node_plane = scnMgr->getRootSceneNode()->createChildSceneNode();
    node_plane->setScale(Ogre::Vector3(100, 100, 100));
    node_plane->setDirection(0,-1,0);
    node_plane->attachObject(ent_plane);
    ent_plane->setMaterial(figure::create_material(1024, 8, 255, 191));
}

void world_3d::setup_ode()
{
    dInitODE2(0);
    world = dWorldCreate();
    dWorldSetGravity (world, 0, -gravity, 0);

    space = dSimpleSpaceCreate(nullptr);

    contactgroup = dJointGroupCreate (0);
    contactgroup_st = contactgroup;
    world_st = world;

    {
        auto fff = dWorldGetERP(world);
        auto fff1 = dWorldGetCFM(world);
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
    std::list<std::string> l;

    for (auto & p : fs::directory_iterator(fs::current_path()))
        if(p.path().filename().extension() == ".bnn")
            l.push_back(p.path().filename());

    if(!l.size())
        return;

    l.sort();

    std::ifstream in(fs::current_path() / l.back(), std::ios::binary);

//    if(crtr.brn_frnd->load(in))
//    {
//        if(l.size() > 8)
//            fs::remove_all(fs::current_path() / l.front());
//    }
}

void world_3d::save()
{
    cout << "save()" << endl;

    if(start_stop_flag)
        start_stop();

    static char time_buffer[15];
    std::time_t time = std::time(nullptr);
    std::strftime(time_buffer, 15, "%Y%m%d%H%M%S", std::localtime(&time));
    std::string s(time_buffer);
    s += ".bnn";
    std::ofstream out(fs::current_path() / s, std::ios::binary);

    if(crtr.brn_frnd->save(out))
        cout << "saved" << endl;
    else
        cout << "save error" << endl;

    start_stop();
}

void world_3d::start_stop()
{//return;
    if(start_stop_flag)
    {
        crtr.brn_frnd->stop();
        cout << "stopped" << endl;
    }
    else
    {

        //crtr.ft = false;
        crtr.brn->start(this);
        cout << "started" << endl;
    }
    start_stop_flag = !start_stop_flag;
}

void world_3d::fill_it_up()
{
    // creating stationary objects
    {
        plane = dCreatePlane(space, 0.0f, 1.0f, 0.0f, 0.0f);

        stationary_colliding_geoms.push_back(plane);

        float size = 2500.0f;
        float height = 100.0f;
        float koef_size = 25.0f;

        for(int i = 0; i < 4; i++)
        {
            float x,y,z;
            x=(i & 1) ? size : size / koef_size;
            y=height;
            z=(i & 1) ? size / koef_size : size;
            stepping_figures.push_back(std::unique_ptr<cube>(new cube("wall_" + std::to_string(i) + std::to_string(i),
                                                                      scnMgr, world, space, x * y * z * mass_scale,
                                                                      x*scale, y*scale, z*scale, 255)));

            dBodySetPosition(stepping_figures.back()->body,
                             !(i & 1) * ((i >> 1) * 2 - 1) * (size / 2 + size / koef_size/2 + 1)*scale,
                             (height / 2 + 1)*scale,
                             (i & 1) * ((i >> 1) * 2 - 1) * (size / 2 + size / koef_size/2 + 1)*scale);

            dJointGroupID jg = dJointGroupCreate (0);
            dJointID j = dJointCreateFixed (world, jg);
            dJointAttach (j, nullptr, dGeomGetBody(stepping_figures.back()->geom));
            dJointSetFixed(j);

            stationary_colliding_geoms.push_back(stepping_figures.back()->geom);

            bounding_nodes.push_back(stepping_figures.back()->node);
        }

        {
            auto w0 = stationary_colliding_geoms.rbegin();
            auto w1 = stationary_colliding_geoms.rbegin();
            advance(w1, 1);
            auto w2 = stationary_colliding_geoms.rbegin();
            advance(w2, 2);
            auto w3 = stationary_colliding_geoms.rbegin();
            advance(w3, 3);

            {
                dJointGroupID jg = dJointGroupCreate (0);
                dJointID j = dJointCreateFixed (world, jg);
                dJointAttach (j, dGeomGetBody(*w0), dGeomGetBody(*w1));
                dJointSetFixed(j);
            }
            {
                dJointGroupID jg = dJointGroupCreate (0);
                dJointID j = dJointCreateFixed (world, jg);
                dJointAttach (j, dGeomGetBody(*w1), dGeomGetBody(*w2));
                dJointSetFixed(j);
            }
            {
                dJointGroupID jg = dJointGroupCreate (0);
                dJointID j = dJointCreateFixed (world, jg);
                dJointAttach (j, dGeomGetBody(*w2), dGeomGetBody(*w3));
                dJointSetFixed(j);
            }
            {
                dJointGroupID jg = dJointGroupCreate (0);
                dJointID j = dJointCreateFixed (world, jg);
                dJointAttach (j, dGeomGetBody(*w3), dGeomGetBody(*w0));
                dJointSetFixed(j);
            }
        }
    }

    // creating movable objects
    {
        {
            float r = ((float)rand() / RAND_MAX) * 20 + 100;

            stepping_figures.push_back(std::unique_ptr<cube>(new cube("cube_qqq", scnMgr, world, space,
                                                                      r*r*r*mass_scale, r*scale, r*scale, r*scale)));
            dBodySetPosition (stepping_figures.back()->body,
                              (((float)rand() / RAND_MAX) - 0.5f) * 2 * 500*scale,
                              (((float)rand() / RAND_MAX)) * 500*scale,
                              (((float)rand() / RAND_MAX) - 0.5f) * 2 * 500*scale);

            movable_colliding_geoms.push_back(stepping_figures.back()->geom);

            bounding_nodes.push_back(stepping_figures.back()->node);
        }

        for(int i = 0; i < 2; i++)
        {
            float r = ((float)rand() / RAND_MAX) * 20 + 50;

            stepping_figures.push_back(std::unique_ptr<sphere>(new sphere("sphere_" + std::to_string(i), scnMgr, world, space,
                                                                          r*r*r*mass_scale, r*scale)));
            dBodySetPosition (stepping_figures.back()->body,
                              (((float)rand() / RAND_MAX) - 0.5f) * 2 * 1500*scale,
                              (((float)rand() / RAND_MAX)) * 1500*scale,
                              (((float)rand() / RAND_MAX) - 0.5f) * 2 * 1500*scale);

            movable_colliding_geoms.push_back(stepping_figures.back()->geom);

            bounding_nodes.push_back(stepping_figures.back()->node);
        }
    }

    input_from_world.reset(new std::vector<uint32>(6));
    for_each(input_from_world->begin(), input_from_world->end(), [&](uint32& i){ i = 0; });

    // creating creature
    {
        crtr = creature(scnMgr, world, input_from_world);
        crtr.set_position(0, 50 * scale, 0);

        creature_colliding_geoms.push_back(crtr.body.geom);

        bounding_nodes.push_back(crtr.body.node);

        for (int i = 0; i < leg_count; i++)
        {
            //creature_colliding_geoms.push_back(crtr.legs[i].first.geom);
            creature_colliding_geoms.push_back(crtr.legs[i].second.geom);
            creature_colliding_geoms.push_back(crtr.legs[i].third.geom);
            //creature_colliding_geoms.push_back(crtr.legs[i].fourth.geom);

            //bounding_nodes.push_back(crtr.legs[i].first.node);
            bounding_nodes.push_back(crtr.legs[i].second.node);
            bounding_nodes.push_back(crtr.legs[i].third.node);
            //bounding_nodes.push_back(crtr.legs[i].fourth.node);
        }

        //        dJointGroupID gc_body = dJointGroupCreate (0);
        //        dJointID j_body = dJointCreateFixed (world, gc_body);
        //        dJointAttach (j_body, 0, dGeomGetBody(crtr.body.geom));
        //        dJointSetFixed(j_body);

        //bool ff = crtr.body.node->getShowBoundingBox();
    }

    trpd.reset(new tripod(world, camNode, crtr.body.body));
}

void world_3d::cycle()
{
    new std::thread([&]()
    {
        float coef = 0.01f;

        while(true)
        {
            while(!crtr.cycle);

            //dBodyAddForce(dGeomGetBody(movable_colliding_geoms.back()), 0, 0, -100);

            dWorldStep(world, 0.05f);

            for_each(stepping_figures.begin(), stepping_figures.end(), [&](std::unique_ptr<figure>& fig){ fig->step(); });

            crtr.step();
            trpd->step();

            float f;
            float coef1 = 1.0f;

            for_each(bounding_nodes.begin(), bounding_nodes.end(), [&](Ogre::SceneNode* node){ node->_updateBounds(); });

            {
                size_t i = 0;
                for_each(movable_colliding_geoms.begin(), movable_colliding_geoms.end(), [&](dGeomID& g)
                {
                    auto pos_fig = dGeomGetPosition(g);
                    auto pos_fl = dGeomGetPosition(crtr.legs[leg_fl].first.geom);
                    auto pos_fr = dGeomGetPosition(crtr.legs[leg_fr].first.geom);

                    f = pow(pow(pos_fig[0] - pos_fl[0], 2) + pow(pos_fig[1] - pos_fl[1], 2) + pow(pos_fig[2] - pos_fl[2], 2), 0.5);
                    (*input_from_world)[i++] = (uint32)(f * coef1);
                    f = pow(pow(pos_fig[0] - pos_fr[0], 2) + pow(pos_fig[1] - pos_fr[1], 2) + pow(pos_fig[2] - pos_fr[2], 2), 0.5);
                    (*input_from_world)[i++] = (uint32)(f * coef1);
                });
            }
            collide_action();

            if(true)
            {
                auto q = camNode->getOrientation();

                Ogre::Quaternion q1;
                Vector3 x, y, z;

                x = q.xAxis();
                y = q.yAxis();
                z = q.zAxis();

                velocity +=
                        x * (keyboard_key_flag_d - keyboard_key_flag_a) * coef
                        + y * (keyboard_key_flag_space - keyboard_key_flag_lshift) * coef
                        + z * (keyboard_key_flag_s - keyboard_key_flag_w) * coef;

                camNode->setPosition(camNode->getPosition().x + velocity.x,
                                     camNode->getPosition().y + velocity.y,
                                     camNode->getPosition().z + velocity.z);
            }

            if(false)
            {
                float coef = 0.001f;

                velocity += Vector3((keyboard_key_flag_d - keyboard_key_flag_a) * coef,
                                    (keyboard_key_flag_lshift - keyboard_key_flag_space) * coef,
                                    (keyboard_key_flag_s - keyboard_key_flag_w) * coef);

                camNode->setPosition(camNode->getPosition().x + velocity.x,
                                     camNode->getPosition().y + velocity.y,
                                     camNode->getPosition().z + velocity.z);
            }

            if(false)
            {
                if(keyboard_key_flag_a) // left
                    camNode->setPosition(camNode->getPosition().x - 1, camNode->getPosition().y, camNode->getPosition().z);

                if(keyboard_key_flag_d) // right
                    camNode->setPosition(camNode->getPosition().x + 1, camNode->getPosition().y, camNode->getPosition().z);

                if(keyboard_key_flag_w) // forward
                    camNode->setPosition(camNode->getPosition().x, camNode->getPosition().y, camNode->getPosition().z - 1);

                if(keyboard_key_flag_s) // backward
                    camNode->setPosition(camNode->getPosition().x, camNode->getPosition().y, camNode->getPosition().z + 1);

                if(keyboard_key_flag_space) // up
                    camNode->setPosition(camNode->getPosition().x, camNode->getPosition().y + 1, camNode->getPosition().z);

                if(keyboard_key_flag_lshift) // down
                    camNode->setPosition(camNode->getPosition().x, camNode->getPosition().y - 1, camNode->getPosition().z);
            }

            crtr.cycle = false;
        }

        dWorldDestroy (world);
        dCloseODE();
    });
}

void nearCallback (void *data, dGeomID o1, dGeomID o2)
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

    if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) {
        cout << "bbbbbbbbbbb" << endl;
        // colliding a space with something :
        dSpaceCollide2 (o1, o2, data,&nearCallback);

        // collide all geoms internal to the space(s)
        if (dGeomIsSpace (o1))
            dSpaceCollide ((dSpaceID)o1, data, &nearCallback);
        if (dGeomIsSpace (o2))
            dSpaceCollide ((dSpaceID)o2, data, &nearCallback);

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
        cout << "ccccccccccccc" << endl;
        // colliding two non-space geoms, so generate contact
        // points between o1 and o2
        //return;
        try {
            //int num_contact = dCollide (o1, o2, 1, cg, 1);
            const int N = 32;
            dContact contact[N];



            int n = dCollide (o1, o2, N, &(contact[0].geom), sizeof(dContact));
            cout << "fffffffffffff" << endl;
            cout << to_string(n) << endl;
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
                    dJointID c = dJointCreateContact (world_st,contactgroup_st,&contact[i]);
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
            cout << "try" << endl;
        }
        // add these contact points to the simulation ...
        cout << "ccccccccccccc12" << endl;
        //cout << to_string(num_contact) << endl;

    }
}

void world_3d::setup(void)
{
    setup_ogre();
    setup_ode();
    fill_it_up();


    //    dMass m;
    //        dMassSetSphere(&m, 0.1, ball_radius);

    //        ball1_geom = dCreateSphere(space, ball_radius);
    //        ball1_body = dBodyCreate(world);
    //        dGeomSetBody(ball1_geom, ball1_body);
    //        dBodySetMass(ball1_body, &m);

    //        ball2_geom = dCreateSphere(space, ball_radius);
    //        ball2_body = dBodyCreate(world);
    //        dGeomSetBody(ball2_geom, ball2_body);
    //        dBodySetMass(ball2_body, &m);




    //    //dMass m;
    //    dMassSetBox (&m,1,SIDE,SIDE,SIDE);
    //    dMassAdjust (&m,MASS);

    //    dQuaternion q;
    //    dQFromAxisAndAngle (q,1,1,0,0.25*M_PI);

    //    body[0] = dBodyCreate (world);
    //    dBodySetMass (body[0],&m);
    //    dBodySetPosition (body[0], -100, 100, 150);
    //    dBodySetQuaternion (body[0],q);

    //    body[1] = dBodyCreate (world);
    //    dBodySetMass (body[1],&m);
    //    dBodySetPosition (body[1], -100, 100, -150);
    //    dBodySetQuaternion (body[1],q);

    //    hinge = dJointCreateHinge (world,0);
    //    dJointAttach (hinge,body[0],body[1]);
    //    dJointSetHingeAnchor (hinge,0,0,1);
    //    dJointSetHingeAxis (hinge,1,-1,1.41421356);

    // run simulation
    //dsSimulationLoop (argc, argv, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, &fn);

    //dWorldSetERP (world, 0.5);
    //dWorldSetCFM (world, 0.001);
    //dBodyAddForce            (body[0], 0, 0, +gravity);



}
