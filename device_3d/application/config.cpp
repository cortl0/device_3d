#include "config.h"

#include <libconfig.h++>

#include <lib/logger/src/helpers/log.h>

namespace bnn_device_3d::application
{

bool config::parse()
{
    using namespace libconfig;

    try
    {
        parse_body();
        return true;
    }
    catch(const FileIOException& e)
    {
        log_error("I/O error while reading file");
    }
    catch(const ParseException& e)
    {
        log_error("Parse error [%s] at %s:%d", e.getError(), e.getFile(), e.getLine());
    }
    catch(const SettingNotFoundException& e)
    {
        log_error("No [%s] setting in configuration file", e.getPath());
    }

    return false;
}

void config::parse_body()
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
}

void config::print()
{
    log_info("config_.device_3d_.time_coefficient [%lf]", device_3d_.time_coefficient);
    log_info("config_.device_3d_.scene_ [%d]", device_3d_.scene_);
    log_info("config_.device_3d_.bnn_.quantity_of_neurons_in_power_of_two [%d]", device_3d_.bnn_.quantity_of_neurons_in_power_of_two);
    log_info("config_.device_3d_.bnn_.motor_binaries_per_motor [%d]", device_3d_.bnn_.motor_binaries_per_motor);
    log_info("config_.device_3d_.bnn_.random_size_in_power_of_two [%d]", device_3d_.bnn_.random_size_in_power_of_two);
    log_info("config_.device_3d_.bnn_.quantity_of_threads_in_power_of_two [%d]", device_3d_.bnn_.quantity_of_threads_in_power_of_two);
}

} // namespace bnn_device_3d::application
