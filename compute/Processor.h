/*
 * BSD 3-Clause License
 * Copyright (c) 2019-2020, Máté Cserép
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_PROCESSOR_H
#define OLP_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace olp
{
namespace compute
{
template<typename PointType>
class Processor
{
public:
    virtual ~Processor() {}

/**
  * Point cloud process function to override
  * @param input Input point cloud
  * @return Processed output point cloud
  */
    virtual typename pcl::PointCloud<PointType>::ConstPtr process(
        typename pcl::PointCloud<PointType>::ConstPtr input) = 0;
};

} // compute
} // olp

#endif //OLP_PROCESSOR_H
