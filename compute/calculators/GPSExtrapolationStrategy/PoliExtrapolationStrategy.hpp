/*
 * BSD 3-Clause License
 * Copyright (c) 2023, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_POLIEXTRAPOLATIONSTRATEGY_HPP
#define OLP_POLIEXTRAPOLATIONSTRATEGY_HPP

#include <array>
#include <cstdint>
#include <cmath>
#include <stdexcept>

#include "GPSExtrapolationStrategy.hpp"


namespace olp
{
namespace compute
{

/**
 * Extrapolation implementation based on up "Degree"-th degree polynomials
 * @tparam Degree Degree of the polynomials
 */
template<std::uint32_t Degree>
class PoliExtrapolationStrategy : public GPSExtrapolationStrategy {
    static_assert (Degree != 0);

public:
    PoliExtrapolationStrategy (const std::uint32_t timeScale)
        : GPSExtrapolationStrategy (timeScale)
        , count (0)
    {
    }


    virtual ~PoliExtrapolationStrategy () override = default;


    virtual helper::gps::GPS GetNextPoint (const std::uint32_t      targetStamp,
                                           const helper::gps::GPS&  prevGPSPoint) override
    {
        while (!gpsDataQueue.empty () && gpsDataQueue.front ().secondsSinceReference <= targetStamp) {
            helper::gps::GPS gpsData = std::move (gpsDataQueue.front ());
            gpsDataQueue.pop ();

            if (gpsData.secondsSinceReference < targetStamp) {
                UpdateHistory (gpsData);
            } else {
                AddToHistory (gpsData);
                return gpsData;
            }
        }

        helper::gps::GPS result = count == 0 ? prevGPSPoint : Predict (targetStamp);
        AddToHistory (result);
        return result;
    }


    virtual void ParsePackets (PacketQueue& packetQueue) override
    {
        grabber::GPSVLPGrabber::GPSPacketConstPtr packet;
        while (!packetQueue.isEmpty ()) {
            packetQueue.dequeue (packet);
            gpsDataQueue.push (PacketToGPS (packet));
        }
    }

private:
    static constexpr std::size_t arraySize = std::pow (2, Degree);
    std::array<helper::gps::GPS, arraySize> historyBuffer;
    std::size_t count;


    void AddToHistory (const helper::gps::GPS& value)
    {
        if (historyBuffer[(count - 1) % arraySize].secondsSinceReference == value.secondsSinceReference &&
            historyBuffer[(count - 1) % arraySize].accuracy < value.accuracy)
        {
            historyBuffer[(count - 1) % arraySize] = value;
        }
        
        historyBuffer[count % arraySize] = value;
        ++count;
    }


    // replace previously added potentially predicted entry
    void UpdateHistory (const helper::gps::GPS& value)
    {
        for (helper::gps::GPS& gps : historyBuffer)
            if (gps.secondsSinceReference == value.secondsSinceReference)
                gps = value;
    }


    helper::gps::GPS Predict (const std::uint32_t timestamp) const
    {
        std::vector<helper::gps::GPS> history = GetTimestampOrderedHistory ();
        helper::gps::GPS prediction = history.back ();
        int actualDegree = 0;
        while (history.size () > 1) {
            ++actualDegree;
            const int elemCount = history.size () / 2;
            std::vector<helper::gps::GPS> derivatives (elemCount);

            for (int i = 0; i < elemCount; ++i)
                derivatives[i] = SubtractGPS (history[i * 2 + 1], history[i * 2]);
            history = std::move (derivatives);

            const gps::GPS& increment = history.back ();
            prediction.latitude += increment.latitude / actualDegree;
            prediction.longitude += increment.longitude / actualDegree;
            prediction.elevation += increment.elevation / actualDegree;
        }
        prediction.accuracy -= 10 * (1. / (actualDegree + 1));
        prediction.secondsSinceReference = timestamp;
        return prediction;
    }


    // not public friend operators because we don't handle all members
    static inline helper::gps::GPS SubtractGPS (const helper::gps::GPS& lhs, const helper::gps::GPS& rhs)
    {
        helper::gps::GPS result = lhs;
        result.latitude -= rhs.latitude;
        result.longitude -= rhs.longitude;
        result.elevation -= rhs.elevation;
        return result;
    }


    inline std::vector<helper::gps::GPS> GetTimestampOrderedHistory () const
    {
        const std::uint32_t historySize = GetBitFloor (count > arraySize ? arraySize : count);
        std::vector<helper::gps::GPS> result (historySize);
        for (int i = 0; i < historySize; ++i)
            result[historySize - 1 - i] = historyBuffer[(count - 1 - i) % arraySize];
        
        return result;
    }


    // bitwise tomfoolery to find previous power of 2 via counting leading zeros
    // see https://stackoverflow.com/questions/2679815/previous-power-of-2
    static inline std::uint32_t GetBitFloor (std::uint32_t x)
    {
        x = x | (x >> 1);
        x = x | (x >> 2);
        x = x | (x >> 4);
        x = x | (x >> 8);
        x = x | (x >> 16);
        return x - (x >> 1);
    }
};


} // namespace compute
} // namespace olp

#endif // OLP_POLIEXTRAPOLATIONSTRATEGY_HPP
