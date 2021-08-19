#ifndef DATA_PROCESSING_METHOD_LINEARLY_SINGLE_H
#define DATA_PROCESSING_METHOD_LINEARLY_SINGLE_H

#include "data_processing_method_base.h"

class data_processing_method_linearly_single : public data_processing_method_base
{
    bool get_bool(float from, float to, float value, int levels_number, int level);
public:
    virtual ~data_processing_method_linearly_single();
    data_processing_method_linearly_single();
    void set_inputs(bnn::brain& brn, _word& count, float value, float range, std::string& str) override;
};

#endif // DATA_PROCESSING_METHOD_LINEARLY_SINGLE_H
