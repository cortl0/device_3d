#ifndef BNN_DEVICE_3D_APPLICATION_CONFIG_H
#define BNN_DEVICE_3D_APPLICATION_CONFIG_H

namespace bnn_device_3d::application
{

class config
{
public:
    struct device_3d
    {
        enum scene : int
        {
            bike_scene,
            table_scene,
        };
        scene scene_;

        enum creature : int
        {
            bike,
            table,
        };
        creature creature_;

        struct bnn
        {
            int quantity_of_neurons_in_power_of_two;
            int motor_binaries_per_motor;
            int random_size_in_power_of_two;
            int quantity_of_threads_in_power_of_two;
        };
        bnn bnn_;
    };
    device_3d device_3d_;

    config();
    bool parse();
    void print();

private:
    bool parse_body();
};

} // namespace bnn_device_3d::application

#endif // BNN_DEVICE_3D_APPLICATION_CONFIG_H
