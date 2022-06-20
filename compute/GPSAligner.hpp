/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_GPSALIGNER_HPP
#define OLP_GPSALIGNER_HPP

#include <chrono>
#include <thread>

#include "Processor.h"

namespace olp
{
namespace compute
{


template<typename PointType>
class GPSAligner : public Processor<PointType> {
public:
    GPSAligner () : latestGPSTimestamp (0) {}

    virtual typename pcl::PointCloud<PointType>::ConstPtr process(
        typename pcl::PointCloud<PointType>::ConstPtr input) override;
    
    void UpdateGPSTimestamp (std::uint64_t timestamp) { latestGPSTimestamp = timestamp; }

private:
    volatile std::uint64_t latestGPSTimestamp;
};


template<typename PointType>
typename pcl::PointCloud<PointType>::ConstPtr GPSAligner<PointType>::process
    (typename pcl::PointCloud<PointType>::ConstPtr input)
{
    constexpr std::uint64_t max_wait_millis = 1000;
    const auto start = std::chrono::high_resolution_clock::now();
    while (true)
    {

        if (latestGPSTimestamp > input->header.stamp)
            return input;

        const auto current = std::chrono::high_resolution_clock::now();
        if (current - start > std::chrono::milliseconds(max_wait_millis))
            return input;
        
        std::this_thread::sleep_for (std::chrono::milliseconds(max_wait_millis/10));
    }
}


} // compute
} // olp

#endif // OLP_GPSALIGNER_HPP
