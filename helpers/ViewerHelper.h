/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_VIEWERHELPER_H
#define OLP_VIEWERHELPER_H

#include <map>
#include <queue>
#include <utility>
#include "IMUHelper.h"

namespace olp
{
namespace helper
{

enum CalculatorType
{
    stonexGPS = 0,
    mobileGPS = 1,
    ICP = 2,
    LOAM = 3
};

struct Color
{
    double r;
    double g;
    double b;
};

const static std::map<std::string, Color> calculatorColors = {{"stonexGPS", {1.0, 0.0, 1.0}},
                                                              {"mobileGPS", {0.0, 0.4, 1.0}},
                                                              {"pcapGPS",   {0.6, 1.0, 0.6}},   // GPS read from csv exported from PCAP packages
                                                              {"packetGPS", {0.0, 0.6, 0.0}},   // GPS read from PCAP packages
                                                              {"ICP",       {1.0, 1.0, 1.0}},
                                                              {"LOAM",      {0.6, 0.0, 0.6}},
                                                              {"IMU",       {0.5, 1.0, 0.0}}};

template<typename PointType>
struct ViewerShareData
{
    std::map<std::string, double> precisionMap;
    std::queue<std::pair<PointType, std::string>> trajectoryQueue;
    imu::IMUEstimate currentIMUEstimate;
    PointType tempPoint;
    std::string tempId;

    // add temp data as new queue element
    void finalizeTemp(){
        // On the first few cloudTransformer runs, when there's no
        // actual data, this will be called with an empty string
        if(!tempId.empty())
            trajectoryQueue.emplace(tempPoint, tempId);
    }
};

inline Color getCalculatorColor(std::string stringId)
{
    return calculatorColors.at(stringId);
}

} // helper
} // olp

#endif //OLP_VIEWERHELPER_H
