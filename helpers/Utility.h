/*
 * BSD 3-Clause License
 * Copyright (c) 2020-2021, Roxána Provender, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_UTILITY_H
#define OLP_UTILITY_H

#include <cstdint>

#include <Eigen/Geometry>

namespace olp
{
namespace helper
{
struct TransformData
{
    Eigen::Affine3d transform;
    double percentage;

    explicit TransformData(double percentage = 0) :
        transform(Eigen::Affine3d::Identity()), percentage(percentage) {}

    TransformData(const Eigen::Affine3d& transform, double percentage = 0)
        : transform(transform), percentage(percentage) {}

    TransformData(const Eigen::Matrix4d& transform, double percentage = 0)
        : transform(transform), percentage(percentage) {}
};

inline uint32_t calculatePointCloudTimeStamp(uint64_t stamp)
{
    return ((stamp & 0x00000000ffffffffl) / 1000000);
}

inline double getRotX(const Eigen::Affine3d& matrix)
{
    return atan2(matrix(2, 1), matrix(2, 2));
}

inline double getRotY(const Eigen::Affine3d& matrix)
{
    return atan2(-matrix(2, 0), std::pow(matrix(2, 1) * matrix(2, 1) +
                                         matrix(2, 2) * matrix(2, 2), 0.5));
}

inline double getRotZ(const Eigen::Affine3d& matrix)
{
    return atan2(matrix(1, 0), matrix(0, 0));
}

} // helper
} // olp

#endif //OLP_UTILITY_H
