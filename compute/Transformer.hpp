/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_TRANSFORMER_HPP
#define OLP_TRANSFORMER_HPP

#include <cmath>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>

#include "../helpers/GPSHelper.h"
#include "../helpers/ViewerHelper.h"

#include "Processor.h"
#include "calculators/Calculator.h"
#include "calculators/GPSCalculator.hpp"

namespace olp
{
namespace compute
{
template<typename PointType>
class CloudTransformer : public Processor<PointType>
{
public:

    CloudTransformer(std::vector<Calculator<PointType>*>& calculators, TransformData startData, std::shared_ptr<ViewerShareData<PointType>> shareData)
        : calculators(calculators), shareData(shareData), Processor<PointType>(), data(startData) {}

    virtual ~CloudTransformer();

protected:
    std::vector<Calculator<PointType>*> calculators;

    std::shared_ptr<ViewerShareData<PointType>> shareData;

    TransformData data;

    typename pcl::PointCloud<PointType>::ConstPtr process(
        typename pcl::PointCloud<PointType>::ConstPtr input) override;

    typename pcl::PointCloud<PointType>::ConstPtr transformCloud
        (typename pcl::PointCloud<PointType>::ConstPtr input, TransformData& data);
};

template<typename PointType>
CloudTransformer<PointType>::~CloudTransformer()
{
    for (Calculator<PointType>* calculator : calculators)
        delete calculator;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::ConstPtr CloudTransformer<PointType>::process(
    typename pcl::PointCloud<PointType>::ConstPtr input)
{
    TransformData bestData;
    TransformData actData;
    int bestId = 0;
    for (auto i = 0; i < calculators.size(); ++i)
    {
        // sync back current transformation to calculators - required by the gps calculators
        calculators.at(i)->startData = data;

        actData = calculators.at(i)->calculate(input);

        shareData->precisionMap.at(calculators.at(i)->stringId()) = actData.percentage;

        if (actData.percentage > bestData.percentage)
        {
            bestData = actData;
            bestId = i;
            shareData->tempId = calculators.at(i)->stringId();
        }
    }

    data.transform = data.transform * bestData.transform;
    
    /*std::cerr << "distance x:" << data.transform.translation()(0) << std::endl;
    std::cerr << "distance y:" << data.transform.translation()(1) << std::endl;
    std::cerr << "distance z:" << data.transform.translation()(2) << std::endl;
    std::cerr << "accuracy:" << bestData.percentage << std::endl;
    std::cerr << "angle:" << getRotZ(data.transform) * 180 / M_PI << std::endl;
    std::cerr << "delta angle:" << getRotZ(bestData.transform) * 180 / M_PI << std::endl;
    std::cerr << "angle of elevation:" << getRotX(data.transform) * 180 / M_PI << std::endl;
    std::cerr << "delta angle of elevation:" << getRotX(bestData.transform) * 180 / M_PI << std::endl;
    std::cerr << "best calculator id:" << bestId << std::endl;
    std::cerr << "---------------" << std::endl;*/

    return transformCloud(input, data);
}

template<typename PointType>
typename pcl::PointCloud<PointType>::ConstPtr CloudTransformer<PointType>::transformCloud
    (typename pcl::PointCloud<PointType>::ConstPtr input, TransformData& data)
{
    typename pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>());

    pcl::transformPointCloud(*input, *transformedCloud, data.transform);

    //create next trajectory point
    shareData->tempPoint.x = data.transform(0, 3);
    shareData->tempPoint.y = data.transform(1, 3);
    shareData->tempPoint.z = data.transform(2, 3);

    std::cerr << "data :" << std::endl;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            std::cerr << data.transform(i,j) << " ";
        }
        std::cerr << std::endl;
    }

    shareData->finalizeTemp();
    return transformedCloud;
}


template<typename PointType>
class MergeTransformer : public Processor<PointType>
{
public:
    typename pcl::PointCloud<PointType>::Ptr mergeCloud;

    MergeTransformer() : Processor<PointType>(), mergeCloud(new pcl::PointCloud<PointType>), octree(1) {}

protected:
    uint32_t previousTimestamp = 0;
    pcl::octree::OctreePointCloudSearch<PointType> octree;
    bool first = true;


    typename pcl::PointCloud<PointType>::ConstPtr process(
        typename pcl::PointCloud<PointType>::ConstPtr input) override;

    typename pcl::PointCloud<PointType>::Ptr filterPointsWithOctree
        (typename pcl::PointCloud<PointType>::ConstPtr input, float radius = 3);
};

template<typename PointType>
typename pcl::PointCloud<PointType>::ConstPtr MergeTransformer<PointType>::process(
    typename pcl::PointCloud<PointType>::ConstPtr input)
{
    uint32_t cloudTimestamp = helper::calculatePointCloudTimeStamp(input->header.stamp);

    if (cloudTimestamp != previousTimestamp)
    {
        previousTimestamp = cloudTimestamp;
        //*mergeCloud += *input;
        return filterPointsWithOctree(input);

    }

    return mergeCloud;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr MergeTransformer<PointType>::filterPointsWithOctree
    (typename pcl::PointCloud<PointType>::ConstPtr input, float radius)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<PointType> extract;

    if (first)
    {
        octree.setInputCloud(mergeCloud);
        octree.addPointsFromInputCloud();

        first = false;
    }
    else
    {
        for (std::size_t i = 0; i < input->points.size(); ++i)
        {
            pointIdxRadiusSearch.clear();
            pointRadiusSquaredDistance.clear();
            if (octree.radiusSearch(input->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <
                500)
            {
                octree.addPointToCloud(input->points[i], mergeCloud);
            }
        }

    }
    return mergeCloud;
}

} // compute
} // olp

#endif //OLP_TRANSFORMER_HPP
