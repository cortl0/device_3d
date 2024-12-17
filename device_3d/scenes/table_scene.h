/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_SCENES_TABLE_H
#define BNN_DEVICE_3D_SCENES_TABLE_H

#include "scene.h"

namespace bnn_device_3d::scenes::table
{

class table final : public scene
{
public:
    table() = delete;
    table(Ogre::RenderWindow*, Ogre::SceneManager*);
    void setup(const bnn_device_3d::application::config::device_3d::bnn&) override;
    void step(keys_states&, dReal frame_length_dReal) override;

private:
    void creating_creature(const bnn_device_3d::application::config::device_3d::bnn&);
    void creating_movable_objects();
    void creating_stationary_objects();

    void circle_step(
            size_t figure_number,
            size_t conductor_number,
            std::list<bnn_device_3d::physical_objects::figure>& stepping_figures
            );

    void dream_step(std::list<bnn_device_3d::physical_objects::figure>& stepping_figures);
    void kick_step(std::list<bnn_device_3d::physical_objects::figure>& stepping_figures);
    void tail_step(std::list<bnn_device_3d::physical_objects::figure>& stepping_figures);
    void keyboard_polling(keys_states&);
    void ogre_setup();
};

} // namespace bnn_device_3d::scenes

#endif // BNN_DEVICE_3D_SCENES_TABLE_H
