/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@yandex.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_APPLICATION_APPLICATION_H
#define BNN_DEVICE_3D_APPLICATION_APPLICATION_H

#include <Ogre.h>

#include "config.h"
#include "scenes/scene.h"

namespace bnn_device_3d::application
{

class application : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
public:
    virtual ~application();
    application();
    void run();

private:
    void setup() override;
    bool keyPressed(const OgreBites::KeyboardEvent&) override;
    bool keyReleased(const OgreBites::KeyboardEvent&) override;
    void load();
    void save();
    void save_random();
    void start();
    void stop();

    std::unique_ptr<bnn_device_3d::scenes::scene> scene;
    bool verbose = true;
    config config_;
    keys_states keys_states_;
};

} // namespace bnn_device_3d::application

#endif // BNN_DEVICE_3D_APPLICATION_APPLICATION_H
