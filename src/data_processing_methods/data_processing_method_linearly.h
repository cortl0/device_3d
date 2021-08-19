#ifndef DATA_PROCESSING_METHOD_LINEARLY_H
#define DATA_PROCESSING_METHOD_LINEARLY_H

#include "data_processing_method_base.h"

class data_processing_method_linearly : public data_processing_method_base
{
    bool get_bool(float from, float to, float value, int levels_number, int level);
public:
    virtual ~data_processing_method_linearly();
    data_processing_method_linearly();
    void set_inputs(bnn::brain& brn, _word& count, float value, float range, std::string& str) override;
};

#endif // DATA_PROCESSING_METHOD_LINEARLY_H
