/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_CALCULATOR_H
#define OLP_CALCULATOR_H

#include <pcl/point_cloud.h>

#include "../../helpers/Utility.h"
#include "../../helpers/ViewerHelper.h"

namespace olp
{
namespace compute
{
template<typename PointType>
class Calculator
{
public:
    helper::TransformData startData;

    inline Calculator(helper::TransformData start) : startData(start) {};

    virtual ~Calculator() {}

    virtual helper::TransformData calculate(
        typename pcl::PointCloud<PointType>::ConstPtr input) = 0;

    virtual const std::string stringId() const = 0;
};

} // compute
} // olp

#endif //OLP_CALCULATOR_H
