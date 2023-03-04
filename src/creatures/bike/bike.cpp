/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "bike.h"

namespace dpm = bnn_device_3d::data_processing_methods;
namespace pho = bnn_device_3d::physical_objects;
namespace sens = bnn_device_3d::sensors;

namespace bnn_device_3d::creatures::bike
{

bike::~bike()
{

}

const dQuaternion wheel_default_quaternion{pow(0.5, 0.5), 0, 0, pow(0.5, 0.5)};

bike::bike(
        Ogre::RenderWindow* render_window,
        Ogre::SceneManager* scnMgr,
        dWorldID world,
        const bnn_device_3d::application::config::device_3d::bnn& config_bnn
        )
{
    data_processing_method_.reset(new dpm::data_processing_method_linearly());

    auto make_fixed_joint = [&](dGeomID g)
    {
        dJointGroupID jg = dJointGroupCreate (0);
        dJointID j = dJointCreateFixed (world, jg);
        dJointAttach(j, dGeomGetBody(body.geom), dGeomGetBody(g));
        dJointSetFixed(j);
    };

    space = dSimpleSpaceCreate(nullptr);

    {
        body = pho::cube("body", scnMgr, world, space, settings::body_mass,
                         settings::body_width, settings::body_height, settings::body_length);

        body.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));
        colliding_geoms.push_back(body.geom);
    }

    auto p = dBodyGetPosition(body.body);

    {
        body_sign = pho::cube("body_sign", scnMgr, world, space, settings::body_mass * 0.001,
                              settings::body_width * 1.05, settings::body_length * 0.95, settings::body_length * 0.95);

        body_sign.set_material(body_sign.create_material_body_sign(512));
        make_fixed_joint(body_sign.geom);
    }

    {
        front_wheel = pho::sphere("bike_front_wheel", scnMgr, world, space, settings::wheel_mass, settings::whell_radius);
        front_wheel.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));
        colliding_geoms.push_back(front_wheel.geom);

        dBodySetQuaternion(front_wheel.body, wheel_default_quaternion);

        dBodySetPosition(front_wheel.body, p[0], p[1] - settings::clearance, p[2] - settings::body_height);

        dJointGroupID Joint_group_id = dJointGroupCreate(0);
        front_hinge2_joint_id = dJointCreateHinge2(world, Joint_group_id);
        dJointAttach(front_hinge2_joint_id, body.body, front_wheel.body);

        dJointSetHinge2Anchor(front_hinge2_joint_id, p[0], p[1] - settings::clearance, p[2] - settings::body_height);
        const dReal axis1[dSA__MAX]{0, 1, 0};
        const dReal axis2[dSA__MAX]{0, 0, 1};
        dJointSetHinge2Axes(front_hinge2_joint_id, axis1, axis2);

        dJointSetHinge2Param(front_hinge2_joint_id, dParamLoStop, -settings::front_whell_direction_angle);
        dJointSetHinge2Param(front_hinge2_joint_id, dParamHiStop, settings::front_whell_direction_angle);
    }

    {
        rear_wheel = pho::sphere("bike_rear_wheel", scnMgr, world, space, settings::wheel_mass, settings::whell_radius);
        rear_wheel.set_material(pho::figure::create_material_chess(128, 32, COLOR_MEDIUM, COLOR_LIGHT));
        colliding_geoms.push_back(rear_wheel.geom);

        dBodySetQuaternion(rear_wheel.body, wheel_default_quaternion);

        dBodySetPosition(rear_wheel.body, p[0], p[1] - settings::clearance, p[2] + settings::body_height);

        dJointGroupID Joint_group_id = dJointGroupCreate(0);
        rear_hinge_joint_id = dJointCreateHinge(world, Joint_group_id);
        dJointAttach(rear_hinge_joint_id, body.body, rear_wheel.body);
        dJointSetHingeAnchor(rear_hinge_joint_id, p[0], p[1] - settings::clearance, p[2] + settings::body_height);
        dJointSetHingeAxis(rear_hinge_joint_id, 0, 0, 1);

        //make_fixed_joint(rear_wheel.geom);
    }

    u_word input_length =
            2 * settings::front_wheel_torque_left.bits_quantity +
            settings::i_feel_my_velosity_quantity_bits +
            settings::i_feel_my_orientation_quantity_bits;

    u_word output_length =
            settings::front_wheel_torque_left.bits_quantity +
            settings::front_wheel_torque_right.bits_quantity +
            settings::rear_wheel_throttle_forward.bits_quantity +
            settings::rear_wheel_throttle_backward.bits_quantity;

    // I feel time
    input_length += sensors::time::get_data_size();

    auto step = 8;
    video_.reset(new bnn_device_3d::sensors::video(render_window->getWidth() / 4, render_window->getHeight() / 4, step));
    input_length += bnn_device_3d::sensors::video::length * video_->calc_data.size();

    const bnn_settings bs
    {
        .quantity_of_neurons_in_power_of_two = static_cast<u_word>(config_bnn.quantity_of_neurons_in_power_of_two),
        .input_length = input_length,
        .output_length = output_length,
        .motor_binaries_per_motor = static_cast<u_word>(config_bnn.motor_binaries_per_motor),
        .random_size_in_power_of_two = static_cast<u_word>(config_bnn.random_size_in_power_of_two),
        .quantity_of_threads_in_power_of_two = static_cast<u_word>(config_bnn.quantity_of_threads_in_power_of_two)
    };

    brain_.reset(new bnn::brain_tools(bs));
}

physical_objects::figure& bike::get_body()
{
    return body;
}

std::vector<physical_objects::figure*> bike::get_figures()
{
    std::vector<pho::figure*> value;
    value.push_back(&body);
    value.push_back(&body_sign);
    value.push_back(&front_wheel);
    value.push_back(&rear_wheel);
    return value;
}

Ogre::Vector3 bike::get_camera_place()
{
    const Ogre::Quaternion ort_x(0, 0, 0, -1);
    const dReal* body_q = dBodyGetQuaternion(body.body);
    Ogre::Quaternion body_quat = Ogre::Quaternion(body_q[0], body_q[1], body_q[2], body_q[3]);
    Ogre::Quaternion body_quat_inv = body_quat.Inverse();
    Ogre::Quaternion ort_x_rel = body_quat * ort_x * body_quat_inv;
    ort_x_rel.normalise();
    const dReal* body_p = dBodyGetPosition(body.body);
    Ogre::Vector3 v(ort_x_rel.x + body_p[0], ort_x_rel.y + body_p[1], ort_x_rel.z + body_p[2]);
    return v * 1.2;
}

dReal bike::get_level()
{
    return settings::level;
}

void bike::set_position(dReal x, dReal y, dReal z)
{
    auto p = dBodyGetPosition(body.body);

    dQuaternion q{1,0,0,0};

    dBodySetPosition(body.body, x, y, z);
    dBodySetQuaternion(body.body, q);

    dBodySetPosition(body_sign.body, x, y, z);
    dBodySetQuaternion(body_sign.body, q);

    dBodySetPosition(front_wheel.body, p[0], p[1] - settings::clearance, p[2] - settings::body_length);
    dBodySetQuaternion(front_wheel.body, wheel_default_quaternion);

    dBodySetPosition(rear_wheel.body, p[0], p[1] - settings::clearance, p[2] + settings::body_length);
    dBodySetQuaternion(rear_wheel.body, wheel_default_quaternion);

    dBodySetLinearVel(body.body, 0, 0, 0);
    dBodySetLinearVel(body_sign.body, 0, 0, 0);
    dBodySetLinearVel(front_wheel.body, 0, 0, 0);
    dBodySetLinearVel(rear_wheel.body, 0, 0, 0);

    dBodySetAngularVel(body.body, 0, 0, 0);
    dBodySetAngularVel(body_sign.body, 0, 0, 0);
    dBodySetAngularVel(front_wheel.body, 0, 0, 0);
    dBodySetAngularVel(rear_wheel.body, 0, 0, 0);
}

void bike::step(std::string& debug_str, bool& verbose)
{
    static constexpr double range = 1.0f;

    static u_word iteration = 0;
    static u_word old_iteration = iteration;
    iteration = brain_->get_iteration();

    if((iteration / 128) > old_iteration)
    {
        //debug_str = "";
        old_iteration = iteration / 128;
        //verbose = true;
    }
    else
        verbose = false;

    u_word i = 0;
    sense_.set_default();

    auto get_value = [&](int length, int both_side = 0) -> int
    {
        int value = 0;

        for(int j = 0; j < length; j++)
        {
            if(verbose)
                debug_str += std::to_string(brain_->get_output(i));

            value += brain_->get_output(i) * !both_side +
                    (brain_->get_output(i) * 2 - 1) * both_side;

            ++i;
        }

        return value;
    };

    force_.left = get_value(settings::front_wheel_torque_left.bits_quantity);
    force_.right = get_value(settings::front_wheel_torque_right.bits_quantity);
    force_.forward = get_value(settings::rear_wheel_throttle_forward.bits_quantity);
    force_.backward = get_value(settings::rear_wheel_throttle_backward.bits_quantity);

    auto calculate_single_effector = [](effector& effector, int value, float rate) -> float
    {
        return effector.force_coefficient * value / effector.bits_quantity - rate;
    };

    auto calculate_double_opposite_effector = [](
            const effector& first,
            const effector& second,
            float force_first,
            float force_second,
            float current_position,
            float rate,
            bool full_rate
            ) -> float
    {
        float force = 0;
        force_first = first.force_coefficient * force_first / first.bits_quantity;
        force_second = second.force_coefficient * force_second / second.bits_quantity;
        force += force_first;
        force -= force_second;
        float position_coefficient = abs(current_position) / ((first.max_position + second.max_position) / 2);
        force *= 1 - position_coefficient;
        force -= rate * (!full_rate * position_coefficient + full_rate);
        return force;
    };

#if(1)
    float front_direction_torque = 0;
#else
    float front_direction_torque = calculate_double_opposite_effector(
                settings::front_wheel_torque_right,
                settings::front_wheel_torque_left,
                force_.right,
                force_.left,
                static_cast<float>(dJointGetHinge2Angle1(front_hinge2_joint_id)),
                static_cast<float>(dJointGetHinge2Angle1Rate(front_hinge2_joint_id)),
                false
                );
#endif

    float rear_throttle_torque = calculate_double_opposite_effector(
                settings::rear_wheel_throttle_forward,
                settings::rear_wheel_throttle_backward,
                force_.forward,
                force_.backward,
                0.0f,
                static_cast<float>(dJointGetHingeAngleRate(rear_hinge_joint_id)),
                true
                );

    dJointAddHinge2Torques(front_hinge2_joint_id, front_direction_torque, rear_throttle_torque/*front_throttle*/);
    dJointAddHingeTorque(rear_hinge_joint_id, rear_throttle_torque);

    front_direction_torque = dJointGetHinge2Angle1(front_hinge2_joint_id);
    //return value_in_range(dJointGetHingeAngle(joint_id), angle_start, angle_end);
    //front_trotle = dJointGetHinge2Angle2(front_hinge2_joint_id);

    body.step();
    body_sign.step();
    front_wheel.step();
    rear_wheel.step();

//#ifdef learning_creature
//    u_word data = teacher_->get_data();
//#endif

    //#define RANDOM_STEPS
//#ifndef RANDOM_STEPS
//    for(size_t i = 0; i < legs.size(); i++)
//    {
//        fs = force[i * QUANTITY_OF_JOINTS_IN_LEG + 0];
//        st = force[i * QUANTITY_OF_JOINTS_IN_LEG + 1];

//#ifdef learning_creature
//        float c = static_cast<float>(((static_cast<int>(data) >> (i * 2)) & 1) * 2 - 1);
//        if(teacher_->get_count())
//        {
//            auto k = static_cast<float>(teacher_->get_count()) / static_cast<float>(teacher_->get_count_max());
//            fs = fs * (1.f - k) + c * k;
//            st = st * (1.f - k) + c * k;
//        }
//#endif

//        //#define REAR_LEGS_OFF
//#ifdef REAR_LEGS_OFF
//        if(NUMBER_OF_REAR_LEFT_LEG == i || NUMBER_OF_REAR_RIGHT_LEG == i)
//        {
//            fs = 0;
//            st = 0;
//        }
//#endif

//        legs[i].step(fs, st);

//        distance[i * QUANTITY_OF_JOINTS_IN_LEG + 0] = fs;
//        distance[i * QUANTITY_OF_JOINTS_IN_LEG + 1] = st;
//    }
//#else
//    // random movements
//    for(int i = 0; i < LEGS_QUANTITY; i++)
//    {
//        fs = ((float)rand() / RAND_MAX) * 2 - 1;
//        st = ((float)rand() / RAND_MAX) * 2 - 1;

//        legs[i].step(fs, st);
//    }
//#endif

    u_word count_input = 0;

    // steering wheel
    data_processing_method_->set_inputs(*brain_.get(), count_input,
                                        settings::front_wheel_torque_left.bits_quantity +
                                        settings::front_wheel_torque_right.bits_quantity,
                                        front_direction_torque, -settings::front_whell_direction_angle,
                                        settings::front_whell_direction_angle, debug_str, verbose);

#if(1)
    debug_str += " ";
    debug_str += "\nvideo [\n";
    video_->set_inputs(*brain_.get(), count_input, bnn_device_3d::sensors::video::length, range, debug_str, verbose);
#endif

//    brain_->set_input(count_input++, front_direction);
//    brain_->set_input(count_input++, front_trotle);

//    debug_str += "] legs [ ";
//    for(size_t i = 0; i < distance.size(); i++)
//    {
//        data_processing_method_->set_inputs(*brain_, count_input, QUANTITY_BITS_PER_JOINT, distance[i], -range, range, debug_str, verbose);
//        if(i % 2)
//            debug_str += " ";
//    }

#if(1)
    debug_str += "]\nvel [ ";
    speedometer_.set_inputs(body.body, *brain_.get(), count_input, settings::i_feel_my_velosity_quantity_bits, -range, range, debug_str, verbose);

    debug_str += " ]\ndir [ ";
    gyroscope_.set_inputs(body.body, *brain_.get(), count_input, settings::i_feel_my_orientation_quantity_bits, -range, range, debug_str, verbose);

    debug_str += " ]\ntime [ ";
    time_.set_inputs(*brain_.get(), count_input, debug_str, verbose);
#endif

//    if(verbose)
//        debug_str += "] out [ ";

//    // I can move legs
//    for(u_word i = 0; i < force.size(); i++)
//    {
//        force[i] = 0;

//        for(size_t j = 0; j < QUANTITY_BITS_PER_JOINT; j++)
//        {
//            force[i] += static_cast<float>(brain_->get_output(i * QUANTITY_BITS_PER_JOINT + j) * 2 - 1);

//            if(verbose)
//                debug_str += std::to_string(brain_->get_output(i * QUANTITY_BITS_PER_JOINT + j));
//        }

//        if(verbose)
//            debug_str += " ";

//        force[i] /= QUANTITY_BITS_PER_JOINT;
//    }

    if(verbose)
    {
        debug_str += "]\n";
        brain_->get_debug_string(debug_str);
    }
}

void sense::set_default()
{
    left = 0;
    right = 0;
    front = 0;
    rear = 0;
}

} // namespace bnn_device_3d::creatures::bike
