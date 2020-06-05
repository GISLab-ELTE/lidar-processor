/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_UTILITY_H
#define OLP_UTILITY_H

#include <cstdint>

namespace olp
{
namespace helper
{
struct TransformData
{
    double x;
    double y;
    double z;
    double angle;
    double angleOfElevation;
    double percentage;

    TransformData(double x = 0, double y = 0, double z = 0, double angle = 0, double percentage = 0, double angleOfElevation = 0)
        : x(x), y(y), z(z), angle(angle), percentage(percentage), angleOfElevation(angleOfElevation) {}
};

inline uint32_t calculatePointCloudTimeStamp(uint64_t stamp)
{
    return ((stamp & 0x00000000ffffffffl) / 1000000);
}
} // helper
} // olp

#endif //OLP_UTILITY_H
