#ifndef DATA_PROCESSING_METHOD_LINEARLY_H
#define DATA_PROCESSING_METHOD_LINEARLY_H

#include "data_processing_method.h"

class data_processing_method_linearly : public data_processing_method
{
    bool get_bool(float from, float to, float value, int levels_number, int level);
public:
    virtual ~data_processing_method_linearly();
    data_processing_method_linearly();
    void set_inputs(bnn::brain& brn, _word& count, _word length, float value, float range_from, float range_to, std::string& str) override;
};

#endif // DATA_PROCESSING_METHOD_LINEARLY_H
