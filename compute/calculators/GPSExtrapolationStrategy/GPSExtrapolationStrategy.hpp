/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_GPSEXTRAPOLATIONSTRATEGY_HPP
#define OLP_GPSEXTRAPOLATIONSTRATEGY_HPP

#include <queue>

#include <pcl/io/impl/synchronized_queue.hpp>

#include "../../../grabber/GPSPacket.h"
#include "../../../grabber/GPSVLPGrabber.h"
#include "../../../helpers/GPSHelper.h"
#include "../../../helpers/Utility.h"


namespace olp
{
namespace compute
{


/*
 * Interface for the GPSPacketCalculator class used to extrapolate gps measurements
 * when none was found for the current timestamp
 */
class GPSExtrapolationStrategy {
public:
    using PacketQueue = pcl::SynchronizedQueue<grabber::GPSVLPGrabber::GPSPacketConstPtr>;


    GPSExtrapolationStrategy (const std::uint32_t timeScale)
        : timeScale (timeScale)
    {
    }

    virtual ~GPSExtrapolationStrategy () = default;

    virtual helper::gps::GPS GetNextPoint (const std::uint32_t              targetStamp,
                                           const helper::gps::GPS&          prevGPSPoint) = 0;

    virtual void ParsePackets (PacketQueue& packetQueue) = 0;

    helper::gps::GPS PacketToGPS (const grabber::GPSVLPGrabber::GPSPacketConstPtr& packet) const
    {
        helper::gps::GPS result;
        result.accuracy = packet->accuracy;
        result.longitude = packet->longitude;
        result.latitude = packet->latitude;
        result.elevation = packet->elevation;
        result.secondsSinceReference = helper::calculatePointCloudTimeStamp (packet->timestamp, timeScale);
        return result;
    }

protected:
    const std::uint32_t          timeScale;
    std::queue<helper::gps::GPS> gpsDataQueue;
};


} // namespace compute
} // namespace olp

#endif // OLP_GPSEXTRAPOLATIONHELPER_HPP
