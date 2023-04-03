/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_EKFEXTRAPOLATIONSTRATEGY_HPP
#define OLP_EKFEXTRAPOLATIONSTRATEGY_HPP

#include "GPSExtrapolationStrategy.hpp"
#include "../../../helpers/TinyEKFHelper.h"

namespace olp
{
namespace compute
{


/*
 * Extrapolation implementation based on an extended kalman filter
 */
class EKFExtrapolationStrategy : public GPSExtrapolationStrategy {
public:
    EKFExtrapolationStrategy (const std::uint32_t timeScale)
        : GPSExtrapolationStrategy (timeScale)
    {
    }


    virtual ~EKFExtrapolationStrategy () override = default;


    virtual helper::gps::GPS GetNextPoint (const std::uint32_t      targetStamp,
                                           const helper::gps::GPS&  prevGPSPoint) override
    {
        helper::gps::GPS match = prevGPSPoint;
        bool found = false;
        while (!found && !gpsDataQueue.empty () && gpsDataQueue.front ().secondsSinceReference <= targetStamp) {
            helper::gps::GPS front = gpsDataQueue.front ();
            gpsDataQueue.pop ();

            match.latitude = front.latitude;
            match.longitude = front.longitude;
            match.elevation = front.elevation;
            match.accuracy = front.accuracy;
            match.secondsSinceReference = front.secondsSinceReference;

            ekfHelper.setRMatrix ((1000 - front.accuracy) * 10);
            if (first) {
                ekfHelper.update (front, front);
                first = true;
            } else {
                ekfHelper.update (front, match);
            }

            helper::gps::Point newCoord = ekfHelper.getNewCoord ();

            match.latitude = newCoord.x;
            match.longitude = newCoord.y;
            match.elevation = newCoord.z;

            if (match.secondsSinceReference == targetStamp)
                found = true;
        }

        if (!found) {
            match.secondsSinceReference = targetStamp;
            match.accuracy -= 5;
        }

        return match;
    }


    virtual void ParsePackets (PacketQueue& packetQueue) override
    {
        grabber::GPSVLPGrabber::GPSPacketConstPtr packet;
        std::vector<helper::gps::GPS> tmp;
        while (!packetQueue.isEmpty ()) {
            packetQueue.dequeue (packet);
            tmp.push_back (PacketToGPS (packet));
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

private:
    helper::gps::TinyEKFHelper  ekfHelper;
    bool                        first = true;
};


} // namespace compute
} // namespace olp

#endif // OLP_EKFEXTRAPOLATIONSTRATEGY_HPP
