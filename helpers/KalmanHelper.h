/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_KALMANHELPER_H
#define OLP_KALMANHELPER_H

#include <kalman.h>
#include <gps.h>

#include "GPSHelper.h"

namespace olp
{
namespace helper
{
namespace gps
{
static const double C = 2.99792458e8;

class KalmanHelper
{
public:
    KalmanHelper(bool isTest = false);

    void updateFilter(double lat, double lon, bool needEstimate = true, double secondSinceLastTimeStep = 1);
    void setObservationMatrix(double T = 1);

    points getNewCoord();

    ~KalmanHelper();

protected:
    KalmanFilter f;
    bool isGPSTest;
    bool isFirstUpdate;
    int timestamp;

    void setStartPosition(double lat, double lon);

    void setStateTransition(double T = 1, double r = 100.0);

    void setProcessNoiseCovariance(double T = 1, double r = 100.0);

    void freeFilter();
};

} // gps
} // helper
} // olp

#endif //OLP_KALMANHELPER_H
