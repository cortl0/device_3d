#ifndef DATA_PROCESSING_METHOD_BINARY_H
#define DATA_PROCESSING_METHOD_BINARY_H

#include "data_processing_method.h"

class data_processing_method_binary : public data_processing_method
{
    _word get_bools(float from, float to, float value, int levels_number);
public:
    virtual ~data_processing_method_binary();
    data_processing_method_binary();
    void set_inputs(bnn::brain& brn, _word& count, _word length, float value, float range_from, float range_to, std::string& str) override;
};

#endif // DATA_PROCESSING_METHOD_BINARY_H
