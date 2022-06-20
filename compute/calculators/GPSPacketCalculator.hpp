/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_GPSPACKETCALCULATOR_HPP
#define OLP_GPSPACKETCALCULATOR_HPP

#include <array>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>

#include <pcl/io/impl/synchronized_queue.hpp>

#include "../../grabber/GPSVLPGrabber.h"
#include "../../grabber/GPSPacket.h"
#include "../../helpers/Utility.h"
#include "../../helpers/GPSHelper.h"
#include "../../helpers/TinyEKFHelper.h"

#include "Calculator.h"


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
    GPSPacketCalculator (const helper::TransformData& data, std::uint64_t startTime, std::uint32_t timeScale = 500000)
        : Calculator<PointType>     (data)
        , timeScale                 (timeScale)
        , hasDoneFirstCalculation   (false)
        , hasReceivedFirstPacket    (false)
        , hasUsedFirstPacket        (false)
        , lastParsedTimestamp       (0)
        , prevGPSPoint              ()
        , startTime                 (startTime)
    {}


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
    pcl::SynchronizedQueue<grabber::GPSVLPGrabber::GPSPacketConstPtr>   gpsPacketQueue;
    std::queue<helper::gps::GPS>                                        gpsDataQueue;

    const std::uint32_t timeScale;
    const std::uint64_t startTime;

    std::uint32_t       lastParsedTimestamp;
    // the GPS point used for the previous positioning
    helper::gps::GPS    prevGPSPoint;

    bool            hasDoneFirstCalculation;
    volatile bool   hasReceivedFirstPacket;
    bool            hasUsedFirstPacket;

    helper::gps::TinyEKFHelper ekfHelper;

    helper::TransformData   matchWithGPS    (typename pcl::PointCloud<PointType>::ConstPtr input);
    helper::gps::GPS        getMatchingGPS  (const std::uint32_t cloudStamp);
    void                    parsePackets    ();
    inline helper::gps::GPS packetToGPS     (const grabber::GPSVLPGrabber::GPSPacketConstPtr& packet) const;
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

    helper::TransformData data = helper::gps::calculateTransformData (prevGPSPoint, currGPSPoint, this->startData);
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

    parsePackets ();
    helper::gps::GPS match = prevGPSPoint;
    bool found = false;
    while (!found && !gpsDataQueue.empty () && gpsDataQueue.front ().secondsSinceReference <= cloudStamp) {
        helper::gps::GPS front = gpsDataQueue.front ();

        ekfHelper.setRMatrix ((1000 - front.accuracy) * 10);
        if (hasUsedFirstPacket) {
            ekfHelper.update (front, front);
            hasUsedFirstPacket = true;
        }
        else {
            ekfHelper.update (front, match);
        }

        helper::gps::Point newCoord = ekfHelper.getNewCoord ();

        match.latitude = newCoord.x;
        match.longitude = newCoord.y;
        match.elevation = newCoord.z;
        match.accuracy = front.accuracy;
        match.secondsSinceReference = front.secondsSinceReference;

        gpsDataQueue.pop ();

        if (match.secondsSinceReference == cloudStamp)
            found = true;
    }

    if (!found) {
        match.secondsSinceReference = cloudStamp;
        match.accuracy -= 5;
    }

    return match;
}


template<typename PointType>
void GPSPacketCalculator<PointType>::parsePackets ()
{
    grabber::GPSVLPGrabber::GPSPacketConstPtr packet;
    std::vector<helper::gps::GPS> tmp;
    while (!gpsPacketQueue.isEmpty ()) {
        gpsPacketQueue.dequeue (packet);
        tmp.push_back (packetToGPS (packet));
    }

    if (tmp.empty ())
        return;

    helper::gps::GPS& best = tmp[0];
    for (helper::gps::GPS& current : tmp) {
        if (current.secondsSinceReference == best.secondsSinceReference) {
            if (current.accuracy > best.accuracy)
                best = current;
        }
        else {
            gpsDataQueue.push (best);
            best = current;
        }
            
    }
    gpsDataQueue.push (best);
}


template<typename PointType>
helper::gps::GPS GPSPacketCalculator<PointType>::packetToGPS (const grabber::GPSVLPGrabber::GPSPacketConstPtr& packet) const
{
    helper::gps::GPS result;
    result.accuracy = packet->accuracy;
    result.longitude = packet->longitude;
    result.latitude = packet->latitude;
    result.elevation = packet->elevation;
    result.secondsSinceReference = helper::calculatePointCloudTimeStamp (packet->timestamp, timeScale);
    return result;
}


} // compute
} // olp

#endif //OLP_GPSPACKETCALCULATOR_HPP
