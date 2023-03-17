#include "config.h"

#include <iostream>

#include <libconfig.h++>

namespace bnn_device_3d::application
{

config::config()
{

}

bool config::parse()
{
    using namespace libconfig;

    try
    {
        return parse_body();
    }
    catch(const FileIOException& e)
    {
      std::cerr << "I/O error while reading file" << std::endl;
    }
    catch(const ParseException& e)
    {
      std::cerr << "Parse error at " << e.getFile() << ":" << e.getLine() << " - " << e.getError() << std::endl;
    }
    catch(const SettingNotFoundException& e)
    {
      std::cerr << "No [" << e.getPath() << "] setting in configuration file" << std::endl;
    }

    return false;
}

bool config::parse_body()
{
    using namespace libconfig;
    Config cfg;
    cfg.readFile("config.cfg");
    const Setting& root = cfg.getRoot();

    const Setting& device_3d_setting = root["device_3d"];
    *(int*)&device_3d_.scene_ = device_3d_setting["scene"];
    *(double*)&device_3d_.time_coefficient = device_3d_setting["time_coefficient"];

    const Setting& bnn_setting = device_3d_setting["bnn"];
    *(int*)&device_3d_.bnn_.quantity_of_neurons_in_power_of_two = bnn_setting["quantity_of_neurons_in_power_of_two"];
    *(int*)&device_3d_.bnn_.motor_binaries_per_motor = bnn_setting["motor_binaries_per_motor"];
    *(int*)&device_3d_.bnn_.random_size_in_power_of_two = bnn_setting["random_size_in_power_of_two"];
    *(int*)&device_3d_.bnn_.quantity_of_threads_in_power_of_two = bnn_setting["quantity_of_threads_in_power_of_two"];

    return true;
}

void config::print()
{
    printf("config_.device_3d_.time_coefficient [%lf]\n", device_3d_.time_coefficient);
    printf("config_.device_3d_.scene_ [%d]\n", device_3d_.scene_);
    printf("config_.device_3d_.bnn_.quantity_of_neurons_in_power_of_two [%d]\n", device_3d_.bnn_.quantity_of_neurons_in_power_of_two);
    printf("config_.device_3d_.bnn_.motor_binaries_per_motor [%d]\n", device_3d_.bnn_.motor_binaries_per_motor);
    printf("config_.device_3d_.bnn_.random_size_in_power_of_two [%d]\n", device_3d_.bnn_.random_size_in_power_of_two);
    printf("config_.device_3d_.bnn_.quantity_of_threads_in_power_of_two [%d]\n", device_3d_.bnn_.quantity_of_threads_in_power_of_two);
}

} // namespace bnn_device_3d::application
