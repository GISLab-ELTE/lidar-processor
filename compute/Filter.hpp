/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender & Máté Cserép
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_FILTER_HPP
#define OLP_FILTER_HPP

#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "Processor.h"
#include <algorithm>


namespace olp
{
namespace compute
{
template<typename PointType>
class OrigoFilter : public Processor<PointType>
{
public:
    OrigoFilter() : Processor<PointType>() {}

protected:
    typename pcl::PointCloud<PointType>::ConstPtr process(
        typename pcl::PointCloud<PointType>::ConstPtr input) override
    {
        return filterPoints(input);
    }

    typename pcl::PointCloud<PointType>::ConstPtr filterPoints
        (typename pcl::PointCloud<PointType>::ConstPtr input,
         float minX = -1.0, float maxX = 1.0, float minY = -1.0, float maxY = 1.0, float minZ = -16.0,
         float maxZ = 16.0)
    {

        pcl::CropBox<PointType> boxFilter;
        boxFilter.setNegative(true);
        boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
        boxFilter.setInputCloud(input);

        // Filtered cloud
        typename pcl::PointCloud<PointType>::Ptr cloudFiltered(new pcl::PointCloud<PointType>);
        boxFilter.filter(*cloudFiltered);

        return cloudFiltered;
    }
};

template<typename PointType>
class AxisFilter : public Processor<PointType>
{
public:
    AxisFilter() : Processor<PointType>() {}

protected:
    typename pcl::PointCloud<PointType>::Ptr process(
        typename pcl::PointCloud<PointType>::ConstPtr input) override
    {
        return filterPoints(input);
    }

    typename pcl::PointCloud<PointType>::Ptr filterPoints
        (typename pcl::PointCloud<PointType>::ConstPtr input,
            const std::string& axis = "y")
    {
        pcl::PassThrough<PointType> pass;
        pass.setInputCloud(input);
        pass.setFilterFieldName(axis);
        pass.setFilterLimits(0.0, 1000.0);

        // Filtered cloud
        typename pcl::PointCloud<PointType>::Ptr cloudFiltered(new pcl::PointCloud<PointType>);
        pass.filter(*cloudFiltered);

        return cloudFiltered;
    }
};

} // compute
} // olp

#endif //OLP_FILTER_HPP
