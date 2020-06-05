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

    CloudTransformer(std::vector<Calculator<PointType>*>& calculators, TransformData startData)
        : calculators(calculators), Processor<PointType>()
    {
        data.angle = startData.angle;
        data.angleOfElevation = startData.angleOfElevation;
    }

protected:
    std::vector<Calculator<PointType>*> calculators;

    TransformData data = TransformData();

    typename pcl::PointCloud<PointType>::Ptr process(
        typename pcl::PointCloud<PointType>::ConstPtr input) override;

    typename pcl::PointCloud<PointType>::Ptr transformCloud
        (typename pcl::PointCloud<PointType>::ConstPtr input, TransformData& data);
};

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr CloudTransformer<PointType>::process(
    typename pcl::PointCloud<PointType>::ConstPtr input)
{
    TransformData* bestData = new TransformData();
    TransformData* actData = new TransformData();
    int bestId = 0;
    for (auto i = 0; i < calculators.size(); ++i)
    {
        *actData = calculators.at(i)->calculate(input);

        if (actData->percentage > bestData->percentage)
        {
            *bestData = *actData;
            bestId = i;
        }
    }

    data.x += bestData->x;
    data.y += bestData->y;
    data.z += bestData->z;
    data.angle += bestData->angle;
    data.angleOfElevation += bestData->angleOfElevation;


    std::cerr << "distance x:" << data.x << std::endl;
    std::cerr << "distance y:" << data.y << std::endl;
    std::cerr << "distance z:" << data.z << std::endl;
    std::cerr << "accuracy:" << bestData->percentage << std::endl;
    std::cerr << "angle:" << data.angle * 180 / M_PI << std::endl;
    std::cerr << "delta angle:" << bestData->angle * 180 / M_PI << std::endl;
    std::cerr << "angle of elevation:" << data.angleOfElevation * 180 / M_PI << std::endl;
    std::cerr << "delta angle of elevation:" << bestData->angleOfElevation * 180 / M_PI << std::endl;
    std::cerr << "best calculator id:" << bestId << std::endl;
    std::cerr << "---------------" << std::endl;

    return transformCloud(input, data);
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr CloudTransformer<PointType>::transformCloud
    (typename pcl::PointCloud<PointType>::ConstPtr input, TransformData& data)
{

    Eigen::Matrix4f rotationZ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotationX = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();


    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();


    typename pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>());

    transform.rotate(Eigen::AngleAxisf(data.angle, Eigen::Vector3f::UnitZ()));
    transform2.rotate(Eigen::AngleAxisf(data.angleOfElevation, Eigen::Vector3f::UnitX()));
    transform.translation() << data.x, data.y, data.z;

    rotationZ.block<3,3>(0,0) = transform.rotation();
    rotationX.block<3,3>(0,0) = transform2.rotation();
    translation.block<3,1>(0,3) = transform.translation();

    matrix =  translation  * rotationX  * rotationZ ;

    pcl::transformPointCloud(*input, *transformedCloud, matrix);


    std::cerr << "data :" << std::endl;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            std::cerr << matrix(i,j) << " ";
        }
        std::cerr << std::endl;
    }


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


    typename pcl::PointCloud<PointType>::Ptr process(
        typename pcl::PointCloud<PointType>::ConstPtr input) override;

    typename pcl::PointCloud<PointType>::Ptr filterPointsWithOctree
        (typename pcl::PointCloud<PointType>::ConstPtr input, float radius = 3);
};

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr MergeTransformer<PointType>::process(
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
