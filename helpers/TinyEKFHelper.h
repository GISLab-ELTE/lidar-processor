/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_TINYEKFHELPER_H
#define OLP_TINYEKFHELPER_H

/* states */
#define Nsta 6

/* observables */
#define Mobs 8

#include <TinyEKF.h>
#include "GPSHelper.h"


namespace olp
{
namespace helper
{
namespace gps
{

class TinyEKFHelper: public TinyEKF
{
public:
    double T = 1;

    TinyEKFHelper();
    void update(gps::GPS currentgps, gps:: GPS previousGPS);
    points getNewCoord();
    void setRMatrix(double R0 = 29);
protected:
    std::vector<double> pseudorange;

    bool isFirst = true;

    void setQMatrix(double r = 100.0);
    void setPMatrix(double P0 = 10);
    void setXVektor(gps::GPS gps);
    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) override;
    void blkfill(const double * a, int off);
};
}
}
}


#endif //OLP_TINYEKFHELPER_H
