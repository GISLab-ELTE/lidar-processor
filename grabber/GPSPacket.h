/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_GPSPACKET_H
#define OLP_GPSPACKET_H


namespace olp
{
namespace grabber
{

// barebone gps class used by the GPSVLPGrabber and its signal
struct GPSPacket
{
    double latitude;
    double longitude;
    double elevation;

    double accuracy;
    std::uint64_t timestamp;
};


} // grabber
} // olp

#endif // OLP_GPSPACKET_H
