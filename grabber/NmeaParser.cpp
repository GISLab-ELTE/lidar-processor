/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include "NmeaParser.h"

#include <iostream>

#include <boost/algorithm/string.hpp>


using namespace olp::grabber;


bool NmeaParser::TryParseSentence (const std::string& sentence, GPSPacket& packet)
{
    std::vector<std::string> messageParts;
    boost::split (messageParts, sentence, [] (char c) -> bool { return c == ','; } );
    bool isValid = false;

    if (messageParts[0] == "GGA")
        isValid = TryParseGGA (packet, messageParts);
    else if (messageParts[0] == "RMC")
        isValid = TryParseRMC (packet, messageParts);
    else {
        // std::cerr << "Unrecognized NMEA header: $GP" << messageParts[0] << " in sentence [ $GP" << sentence << " ]" << std::endl;
    }

    return isValid;
}


/**
 * Relevant fields:
 * 0:   header (GGA)
 * 1:   utc time (hhmmss.ss) -> already have from packet
 * 2:   latitude (DDmm.mm)
 * 3:   latitude direction (N/S)
 * 4:   longitute (DDDmm.mm)
 * 5:   longitude direction (E/W)
 * 6:   -
 * 7:   quality (0-7)
 * 8:   hdop
 * ...
 */
bool NmeaParser::TryParseGGA (GPSPacket& packet, const std::vector<std::string>& messageParts)
{
    // Fix not available or invalid
    if (messageParts[7] == "0")
        return false;
    
    packet.latitude = GetLatFromString (messageParts[2], messageParts[3]);
    packet.longitude = GetLonFromString (messageParts[4], messageParts[5]);

    // copied from GPSHelper, originally used with PDOP
    double hdop = std::stod (messageParts[8]);
    packet.accuracy = (100 / 11) * (11.5 - hdop);

    return true;
}


/**
 * Relevant fields:
 * 0:   header (RMC)
 * 1:   utc time (hhmmss.ss) -> already have from packet
 * 2:   position status (A -> valid, V -> invalid)
 * 3:   latitude (DDmm.mm)
 * 4:   latitude direction (N/S)
 * 5:   longitute (DDDmm.mm)
 * 6:   longitude direction (E/W)
 * 7:   -
 * 8:   quality (0-7)
 * 9:   hdop
 * ...
 */
bool NmeaParser::TryParseRMC (GPSPacket& packet, const std::vector<std::string>& messageParts)
{
    if (messageParts[2] == "V")
        return false;
    
    packet.latitude = GetLatFromString (messageParts[3], messageParts[4]);
    packet.longitude = GetLonFromString (messageParts[5], messageParts[6]);

    // TODO accuracy metric
    packet.accuracy = 60;

    return true;
}


/**
 * Method to get the latitude in angles
 * @param value DDmm.mm format input string
 * @param direction "N" for north or "S" for south
 */
double NmeaParser::GetLatFromString (const std::string& value, const std::string& direction)
{
    const std::string latDegs = std::string { value.begin (), value.begin () + 2 };
    const std::string latMins = std::string { value.begin () + 2, value.end () };

    double result = std::stod (latDegs);
    result += std::stod (latMins) / 60;
    if (direction == "S")
        result *= -1;
    
    return result;
}


/**
 * Method to get the latitude in angles
 * @param value DDDmm.mm format input string
 * @param direction "W" for west or "E" for east
 */
double NmeaParser::GetLonFromString (const std::string& value, const std::string& direction)
{
    const std::string lonDegs = std::string { value.begin (), value.begin () + 3 };
    const std::string lonMins = std::string { value.begin () + 3, value.end () };

    double result = std::stod (lonDegs);
    result += std::stod (lonMins) / 60;
    if (direction == "W")
        result *= -1;
    
    return result;
}
