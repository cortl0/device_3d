#ifndef DATA_PROCESSING_METHOD_LINEARLY_SINGLE_H
#define DATA_PROCESSING_METHOD_LINEARLY_SINGLE_H

#include "data_processing_method.h"

class data_processing_method_linearly_single : public data_processing_method
{
    bool get_bool(float from, float to, float value, int levels_number, int level);
public:
    virtual ~data_processing_method_linearly_single();
    data_processing_method_linearly_single();
    void set_inputs(bnn::brain& brn, _word& count, _word length, float value, float range_from, float range_to, std::string& str) override;
};

#endif // DATA_PROCESSING_METHOD_LINEARLY_SINGLE_H
