/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_GPSPACKETCALCULATOR_HPP
#define OLP_GPSPACKETCALCULATOR_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "../../helpers/Utility.h"

#include "Calculator.h"
#include "GPSExtrapolationStrategy/EKFExtrapolationStrategy.hpp"
#include "GPSExtrapolationStrategy/PoliExtrapolationStrategy.hpp"


namespace olp
{
namespace compute
{


/**
 * Calculator for real time supplied gps data based cloud positioning.
 * Gps data supplied via the addPacket method.
 * @tparam PointType
 */
template<typename PointType>
class GPSPacketCalculator : public Calculator<PointType>
{
public:
    GPSPacketCalculator (const helper::TransformData& data, std::uint64_t startTime, std::uint32_t timeScale = 500000, bool useEKF = false)
        : Calculator<PointType>     (data)
        , timeScale                 (timeScale)
        , hasDoneFirstCalculation   (false)
        , hasReceivedFirstPacket    (false)
        , prevGPSPoint              ()
        , startTime                 (startTime)
    {
        if (useEKF)
            extrapolationStrategy = std::make_unique<EKFExtrapolationStrategy> (timeScale);
        else
            extrapolationStrategy = std::make_unique<PoliExtrapolationStrategy<2>> (timeScale);
    }


    virtual ~GPSPacketCalculator ()
    {
        gpsPacketQueue.stopQueue ();
    }


    virtual helper::TransformData calculate (typename pcl::PointCloud<PointType>::ConstPtr input) override
    {
        helper::TransformData data = matchWithGPS (input);
        this->startData.transform = this->startData.transform * data.transform;
        return data;
    }


    void addPacket (const grabber::GPSVLPGrabber::GPSPacketConstPtr& packet)
    {
        hasReceivedFirstPacket = true;
        gpsPacketQueue.enqueue (packet);
    }


    virtual const std::string stringId () const override
    {
        return "packetGPS";
    }


private:
    GPSExtrapolationStrategy::PacketQueue       gpsPacketQueue;
    std::unique_ptr<GPSExtrapolationStrategy>   extrapolationStrategy;

    const std::uint32_t timeScale;
    const std::uint64_t startTime;

    helper::gps::GPS    prevGPSPoint;

    bool                hasDoneFirstCalculation;
    bool                hasReceivedFirstPacket;

    helper::TransformData   matchWithGPS    (typename pcl::PointCloud<PointType>::ConstPtr input);
    helper::gps::GPS        getMatchingGPS  (const std::uint32_t cloudStamp);
};


template<typename PointType>
helper::TransformData GPSPacketCalculator<PointType>::matchWithGPS (typename pcl::PointCloud<PointType>::ConstPtr input)
{
    if (!hasReceivedFirstPacket)
        return helper::TransformData {};

    const std::uint32_t cloudStamp = helper::calculatePointCloudTimeStamp (input->header.stamp, timeScale);
    if (cloudStamp < startTime)
        return helper::TransformData {};
    
    helper::gps::GPS currGPSPoint = getMatchingGPS (cloudStamp);

    if (!hasDoneFirstCalculation) {
        prevGPSPoint = currGPSPoint;
        hasDoneFirstCalculation = true;
        return helper::TransformData {};
    }

    helper::TransformData data = helper::gps::calculateTransformData (currGPSPoint, prevGPSPoint, this->startData);
    data.percentage = currGPSPoint.accuracy;
    prevGPSPoint = currGPSPoint;
    return data;
}


template<typename PointType>
helper::gps::GPS GPSPacketCalculator<PointType>::getMatchingGPS (const std::uint32_t cloudStamp)
{
    // previous data is still valid
    if (prevGPSPoint.secondsSinceReference == cloudStamp)
        return prevGPSPoint;

    extrapolationStrategy->ParsePackets (gpsPacketQueue);
    return extrapolationStrategy->GetNextPoint (cloudStamp, prevGPSPoint);
}


} // compute
} // olp

#endif //OLP_GPSPACKETCALCULATOR_HPP
