#ifndef DATA_PROCESSING_METHOD_LOGARITHMIC_H
#define DATA_PROCESSING_METHOD_LOGARITHMIC_H

#include "../bnn/src/brain/simple_math.h"
#include "data_processing_method.h"

class data_processing_method_logarithmic : public data_processing_method
{
    _word get_bools(float from, float to, float value, int levels_number);
public:
    ~data_processing_method_logarithmic();
    data_processing_method_logarithmic();
    void set_inputs(bnn::brain& brn, _word& count, _word length, float value, float range_from, float range_to, std::string& str) override;
};

#endif // DATA_PROCESSING_METHOD_LOGARITHMIC_H
