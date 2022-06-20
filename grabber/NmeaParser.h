/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_NMEAPARSER_H
#define OLP_NMEAPARSER_H

#include <string>
#include <vector>
#include <unordered_map>

#include "GPSPacket.h"

namespace olp
{
namespace grabber
{


/**
 * Class to parse the given NMEA sentence into a uniform GPSPacket form
 */
class NmeaParser {
public:
    static bool TryParseSentence (const std::string& sentence, GPSPacket& packet);

private:
    static bool TryParseGGA (GPSPacket& packet, const std::vector<std::string>& messageParts);
    static bool TryParseRMC (GPSPacket& packet, const std::vector<std::string>& messageParts);

    inline static double GetLatFromString (const std::string& value, const std::string& direction);
    inline static double GetLonFromString (const std::string& value, const std::string& direction);
};


} // grabber
} // olp

#endif // OLP_NMEAPARSER_H
