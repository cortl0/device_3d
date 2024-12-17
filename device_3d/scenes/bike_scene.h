/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SCENES_BIKE_H
#define BNN_DEVICE_3D_SCENES_BIKE_H

#include <OgreTextAreaOverlayElement.h>

#include "scene.h"

namespace bnn_device_3d::scenes::bike
{

class bike final : public scene
{
public:
    ~bike();
    bike() = delete;
    bike(Ogre::RenderWindow*, Ogre::SceneManager*);
    void setup(const bnn_device_3d::application::config::device_3d::bnn&) override;
    void step(keys_states&, dReal frame_length_dReal) override;

private:
    Ogre::TextAreaOverlayElement* create_text_panel();
    void creating_creature(const bnn_device_3d::application::config::device_3d::bnn&);
    void creating_movable_objects();
    void creating_stationary_objects();
    bool is_fail();
    void keyboard_polling(keys_states&);
    void ogre_setup();
    void panel_to_place();
    void print_distance();
    void set_text(std::string&);

    Ogre::TextAreaOverlayElement* text_area = nullptr;
    Ogre::OverlayContainer* panel = nullptr;
};

} // namespace bnn_device_3d::scenes::bike

#endif // BNN_DEVICE_3D_SCENES_BIKE_H
