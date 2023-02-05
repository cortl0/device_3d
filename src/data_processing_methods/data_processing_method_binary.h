/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   http://8iter.ru/ai.html
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#ifndef BNN_DEVICE_3D_DATA_PROCESSING_METHODS_DATA_PROCESSING_METHOD_BINARY_H
#define BNN_DEVICE_3D_DATA_PROCESSING_METHODS_DATA_PROCESSING_METHOD_BINARY_H

#include "data_processing_method.h"

namespace bnn_device_3d::data_processing_methods
{

class data_processing_method_binary : public data_processing_method
{
    u_word get_bools(float from, float to, float value, int levels_number);
public:
    virtual ~data_processing_method_binary();
    data_processing_method_binary();
    void set_inputs(bnn::architecture&, u_word& count, u_word length, float value, float range_from, float range_to,
                    std::string& str, bool verbose) override;
};

} // namespace bnn_device_3d::data_processing_methods

#endif // BNN_DEVICE_3D_DATA_PROCESSING_METHODS_DATA_PROCESSING_METHOD_BINARY_H
