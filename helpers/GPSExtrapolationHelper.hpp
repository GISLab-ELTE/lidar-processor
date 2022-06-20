/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_GPSEXTRAPOLATIONHELPER_HPP
#define OLP_GPSEXTRAPOLATIONHELPER_HPP

#include <array>
#include <cstdint>
#include <cmath>
#include <stdexcept>

#include "GPSHelper.h"


namespace olp
{
namespace helper
{

/**
 * Helper class to extrapolate new GPS datapoints based on up "Degree"-th degree polynomials
 * @tparam Degree Degree of the polynomials
 */
template<std::uint32_t Degree>
class GPSExtrapolationHelper {
    static_assert (Degree != 0);

public:
    GPSExtrapolationHelper () : count (0) {}


    void AddToHistory (const gps::GPS& value)
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
    void UpdateHistory (const gps::GPS& value)
    {
        for (gps::GPS& gps : historyBuffer)
            if (gps.secondsSinceReference == value.secondsSinceReference)
                gps = value;
    }


    gps::GPS Predict (const std::uint32_t timestamp) const
    {
        // this case should be handled in the caller environment
        if (count == 0)
            throw std::logic_error ("Predict called with empty history");
        
        std::vector<gps::GPS> history = GetPreparedHistory ();
        gps::GPS prediction = history.back ();
        int actualDegree = 0;
        while (history.size () > 1) {
            ++actualDegree;
            const int elemCount = history.size () / 2;
            std::vector<gps::GPS> derivatives (elemCount);

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

private:
    static constexpr std::size_t arraySize = std::pow (2, Degree);
    std::array<gps::GPS, arraySize> historyBuffer;
    std::size_t count;


    // not public friend operators because we don't handle all members
    static inline gps::GPS SubtractGPS (const gps::GPS& lhs, const gps::GPS& rhs)
    {
        gps::GPS result = lhs;
        result.latitude -= rhs.latitude;
        result.longitude -= rhs.longitude;
        result.elevation -= rhs.elevation;
        return result;
    }


    inline std::vector<gps::GPS> GetPreparedHistory () const
    {
        const std::uint32_t historySize = GetBitFloor (count > arraySize ? arraySize : count);
        std::vector<gps::GPS> result (historySize);
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


} // helper
} // olp

#endif // OLP_GPSEXTRAPOLATIONHELPER_HPP
