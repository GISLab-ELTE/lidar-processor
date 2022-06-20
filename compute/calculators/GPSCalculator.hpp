/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_GPSCALCULATOR_HPP
#define OLP_GPSCALCULATOR_HPP

#include <cmath>
#include <map>

#include <pcl/common/transforms.h>

#include "../../helpers/GPSHelper.h"
#include "../../helpers/Utility.h"

#include "../Processor.h"
#include "Calculator.h"

namespace olp
{
namespace compute
{

using namespace helper;

template<typename PointType>
class GPSCalculator : public Calculator<PointType>
{
public:
    GPSCalculator(std::vector<gps::GPS>& gps, uint64_t& time, const TransformData& data,
                  gps::GPSSource source = gps::GPSSource::stonex,
                  gps::GPSCoordType coordType = gps::GPSCoordType::latlong)
        : Calculator<PointType>(data)
    {
        gpsCloudMap.clear();
        startTime = time;
        gpsData = gps;
        type = coordType;
        gpsSource = source;
    }

    TransformData calculate(typename pcl::PointCloud<PointType>::ConstPtr input) override
    {
        TransformData data = matchWithGPS(input, type);
        this->startData.transform = this->startData.transform * data.transform;
        return data;
    }

    const std::string stringId() const override
    {
        return gps::GpsStringIds.at(gpsSource);
    }

protected:
    int startShift = 0;

    std::vector<gps::GPS> gpsData;
    uint64_t startTime;
    std::vector<gps::GPS> gpsCloudMap;
    gps::GPSCoordType type;
    gps::GPSSource gpsSource;
    TransformData actData;

    TransformData matchWithGPS(typename pcl::PointCloud<PointType>::ConstPtr input,
                               gps::GPSCoordType coordType = gps::GPSCoordType::latlong);
};

template<typename PointType>
TransformData GPSCalculator<PointType>::matchWithGPS(typename pcl::PointCloud<PointType>::ConstPtr input,
                                                     gps::GPSCoordType coordType)
{
    TransformData data = TransformData();
    int seq = input.get()->header.seq;
    uint32_t cloudTimestamp = helper::calculatePointCloudTimeStamp(input->header.stamp);
    if (seq == 0)
        startShift = cloudTimestamp;

    uint64_t timestamp = startTime + (cloudTimestamp - startShift);

    int i = 0;
    while (i < gpsData.size() && gpsData[i].secondsSinceReference <= timestamp)
    {
        i++;
    }

    if (i < gpsData.size())
    {
        int diff1 = gpsData[i].secondsSinceReference - timestamp;
        int diff2 = diff1;
        int index = i;

        if (i > 0)
        {
            diff2 = gpsData[i - 1].secondsSinceReference - timestamp;
            index = diff1 <= diff2 ? i : i - 1;
        }

        if (gpsCloudMap.size() > 0)
        {
            data = gps::calculateTransformData(gpsCloudMap[gpsCloudMap.size() - 1], gpsData[index], this->startData,
                                               coordType);
            data.percentage = gpsData[index].accuracy;
            int timeDiff = (gpsData[index].secondsSinceReference - timestamp);
            data.percentage -= 10 * (timeDiff < 0 ? -timeDiff : timeDiff);
        }

        gpsCloudMap.push_back(gpsData[index]);


        std::cerr << "source:" << (gpsSource == gps::GPSSource::stonex ? "Stonex" : "Mobile") << std::endl;
        std::cerr << "seq:" << seq << std::endl;
        std::cerr << "index:" << index << std::endl;
        std::cerr << "gps_timestamp:" << gpsData[index].secondsSinceReference << std::endl;
        std::cerr << "cloud_timestamp:" << timestamp << std::endl;
        std::cerr << "accuracy:" << data.percentage << std::endl;
        std::cerr << "---------------" << std::endl;
    }

    return data;
}

} // compute
} // olp

#endif //OLP_GPSCALCULATOR_HPP
