/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender & Máté Cserép
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_PROCESSORPIPE_HPP
#define OLP_PROCESSORPIPE_HPP

#include "../compute/Processor.h"
#include "../helpers/Utility.h"

namespace olp
{
namespace pipe
{
template<typename PointType>
class ProcessorPipe
{
public:
    ProcessorPipe();

    ProcessorPipe* add(compute::Processor<PointType>* step);

    virtual ~ProcessorPipe();

    typename pcl::PointCloud<PointType>::Ptr execute(typename pcl::PointCloud<PointType>::ConstPtr input);

protected:
    std::vector<compute::Processor<PointType>*> processors;
    uint32_t previousTimestamp = 0;
    typename pcl::PointCloud<PointType>::Ptr result;
};


template<typename PointType>
ProcessorPipe<PointType>::ProcessorPipe() {}

template<typename PointType>
ProcessorPipe<PointType>::~ProcessorPipe()
{
    for (auto processor : processors)
    {
        if (processor != nullptr)
        {
            delete processor;
        }
    }
}

template<typename PointType>
ProcessorPipe<PointType>* ProcessorPipe<PointType>::add(compute::Processor<PointType>* step)
{
    processors.push_back(step);
    return this;
}


template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr
ProcessorPipe<PointType>::execute(typename pcl::PointCloud<PointType>::ConstPtr input)
{
    uint32_t cloudTimestamp = helper::calculatePointCloudTimeStamp(input->header.stamp);

    typename pcl::PointCloud<PointType>::Ptr tmpResult;
    if (cloudTimestamp != previousTimestamp)
    {
        previousTimestamp = cloudTimestamp;

        for (auto processor : processors)
        {
            tmpResult = processor->process(tmpResult == nullptr ? input : tmpResult);
        }
        result = tmpResult;
    }
    return result;
}

} // pipe
} // olp

#endif //OLP_PROCESSORPIPE_HPP
