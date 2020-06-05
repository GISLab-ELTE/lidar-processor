/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <math.h>

#include "KalmanHelper.h"

namespace olp
{
namespace helper
{
namespace gps
{
KalmanHelper::KalmanHelper(bool isTest)
{
    isGPSTest = isTest;
    isFirstUpdate = true;
    timestamp = 0;

    if (isGPSTest)
    {
        f = alloc_filter_velocity2d(1.0);
    }
    else
    {
        f = alloc_filter(14, 6);
        setStateTransition();
        setStartPosition(0.0, 0.0);
        setProcessNoiseCovariance();
        setObservationMatrix(0.5);
    }
}

KalmanHelper::~KalmanHelper()
{
    freeFilter();
}

void KalmanHelper::updateFilter(double lat, double lon, bool needEstimate, double secondSinceLastTimeStep)
{
    timestamp++;

    if (isGPSTest)
    {
        update_velocity2d(f, lat, lon, secondSinceLastTimeStep);
    }
    else
    {
        if (isFirstUpdate)
        {
            setStartPosition(lat, lon);
            isFirstUpdate = false;
        }
        if (needEstimate)
        {
            set_matrix(f.observation, lat, 0.0, lon, 0.0, 0.0, 0.0);
            update(f);
        }
        else {
            predict(f);
        }
    }
}

points KalmanHelper::getNewCoord()
{
    double lat;
    double lon;

    if (isGPSTest)
    {
        get_lat_long(f, &lat, &lon);
        std::cerr << std::setprecision(15) << "lat" << ":" << lat << std::endl;
        std::cerr << std::setprecision(15) << "lon" << ":" << lon << std::endl;
    }
    else
    {
        lat = f.state_estimate.data[0][0];
        lon = f.state_estimate.data[2][0];

        for (int i = 0; i < 14; i++)
        {
            std::cerr << i << ":" << f.state_estimate.data[i][0] << std::endl;
        }
    }

    std::cerr << std::endl << "---------------------" << std::endl;
    return points(lat, lon);
}

void KalmanHelper::setStateTransition(double T, double r)
{

    double d = (2 * r - 1) / (2 * r + 1);
    set_matrix(f.state_transition,
               1.0, T, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0, T, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 1.0, T, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, T, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, d, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, d, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, d, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, d, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, d, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, d);
}

void KalmanHelper::setProcessNoiseCovariance(double T, double r)
{
    double l = (pow(C, 2.0) * pow(T, 3.0)) / 3;
    double b = (pow(C, 2.0) * pow(T, 2.0)) / 2;
    double g = T;
    double p = 2 * pow((2 * r / (2 * r + 1)), 2.0);

    double ox, oy, oz = pow(1.6, 2.0);
    double of = pow(0.6 * pow(10, -18), 2.0);
    double og = pow(1.0 * pow(10, -16), 2.0);

    double oe = pow(0.3 * 4, 2);

    set_matrix(f.process_noise_covariance,
               l * ox * b * oy, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               b * ox * g * ox, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, l * oy * b * oy, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, b * oy * g * oy, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, l * oz * b * oz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, b * oz * g * oz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, g * of + l * og * b * og, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, b * og, g * og, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p * oe, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p * oe, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p * oe, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p * oe, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p * oe, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, p * oe);


}

void KalmanHelper::setObservationMatrix(double T)
{
    set_matrix(f.observation_noise_covariance,
            T,0.0,0.0,0.0,0.0,0.0,
            0.0,T,0.0,0.0,0.0,0.0,
            0.0,0.0,T,0.0,0.0,0.0,
            0.0,0.0,0.0,T,0.0,0.0,
            0.0,0.0,0.0,0.0,T,0.0,
            0.0,0.0,0.0,0.0,0.0,T);
}

void KalmanHelper::setStartPosition(double lat, double lon)
{
    set_matrix(f.observation_model,
               1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    set_matrix(f.state_estimate, lat, 0.0, lon, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    set_identity_matrix(f.estimate_covariance);
}

void KalmanHelper::freeFilter()
{
    free_filter(f);
}

} // gps
} // helper
} // olp
