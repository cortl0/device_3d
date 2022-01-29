#ifndef DATA_PROCESSING_METHOD_H
#define DATA_PROCESSING_METHOD_H

#include "config.h"
#include "bnn/src/brain/brain.h"

class data_processing_method
{
public:
    virtual ~data_processing_method();
    data_processing_method();
    virtual void set_inputs(bnn::brain& brn, _word& count, _word length, float value, float range_from, float range_to, std::string& str) = 0;
};

#endif // DATA_PROCESSING_METHOD_H
