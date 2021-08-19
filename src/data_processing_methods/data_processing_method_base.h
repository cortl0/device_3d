#ifndef DATA_PROCESSING_METHOD_BASE_H
#define DATA_PROCESSING_METHOD_BASE_H

#include "config.h"
#include "Ogre.h"
#include "bnn/src/brain/brain.h"

class data_processing_method_base
{
public:
    virtual ~data_processing_method_base();
    data_processing_method_base();
    virtual void set_inputs(bnn::brain& brn, _word& count, float value, float range, std::string& str) = 0;
};

#endif // DATA_PROCESSING_METHOD_BASE_H
