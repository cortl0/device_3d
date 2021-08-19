#ifndef DATA_PROCESSING_METHOD_BINARY_H
#define DATA_PROCESSING_METHOD_BINARY_H

#include "data_processing_method_base.h"

class data_processing_method_binary : public data_processing_method_base
{
    _word get_bools(float from, float to, float value, int levels_number);
public:
    virtual ~data_processing_method_binary();
    data_processing_method_binary();
    void set_inputs(bnn::brain& brn, _word& count, float value, float range, std::string& str) override;
};

#endif // DATA_PROCESSING_METHOD_BINARY_H
