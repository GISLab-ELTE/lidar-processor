/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include "TinyEKFHelper.h"
#include "KalmanHelper.h"


namespace olp
{
namespace helper
{
namespace gps
{
TinyEKFHelper::TinyEKFHelper()
{
    setQMatrix();
    setRMatrix();
    setPMatrix();
}

void TinyEKFHelper::blkfill(const double * a, int off)
{
   // off *= 2;

    setQ(off, off ,a[0]);
    setQ(off, off+1,a[1]);
    setQ(off+1, off, a[2]);
    setQ(off+1,off+1, a[3]);
}


void TinyEKFHelper::setQMatrix(double r) {

    double sigma  = 0.0625;


    double part1 = sigma*(pow(T, 4)/4.0);
    double part2 = sigma*(pow(T, 3)/2.0);
    double part3 = sigma*(pow(T, 2));

    double xyz[4] = {part1,part2,part2,part3};
    blkfill(xyz, 0);
    blkfill(xyz, 2);
    blkfill(xyz, 5);

}
void TinyEKFHelper::setRMatrix(double R0) {
    for (int i=0; i<Mobs; ++i)
        setR(i,i,R0);
}

void TinyEKFHelper::setPMatrix(double P0) {
    for (int i=0; i<Nsta; ++i)
        setP(i,i,P0);
}

void TinyEKFHelper::setXVektor(gps::GPS gps) {
    setX(0,gps.latitude);
    setX(2,gps.longitude);
    setX(4,gps.elevation);


    // velocity
    setX(1,0);
    setX(3,0);
    setX(5,0);

    // clock bias
   // setX(6,3.575261153706439e+006);

    // clock drift
     //setX(7,4.549246345845814e+001);
}

void TinyEKFHelper::update(gps::GPS currentgps, gps:: GPS previousGPS) {
    if(isFirst) {
        setXVektor(currentgps);
        isFirst = false;
    } else {
        double velocityLat = currentgps.latitude - previousGPS.latitude;
        double velocityLon = currentgps.longitude - previousGPS.longitude;
        double velocityAlt = currentgps.elevation - previousGPS.elevation;

        double z[6] = {currentgps.latitude, velocityLat, currentgps.longitude, velocityLon, currentgps.elevation, velocityAlt};
        step(z);
    }


}

points TinyEKFHelper::getNewCoord() {
    std::cerr << "--------------" << std::endl;
    for(int i = 0; i < 6; i++) {
        std::cerr << i << ": " << getX(i) << std::endl;
    }

    return points(getX(0), getX(2), getX(4));


}

void TinyEKFHelper::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
{
    int i, j;

    for (j=0; j<Nsta; j+=2) {
        fx[j] = this->x[j] + T * this->x[j+1];
        fx[j+1] = this->x[j+1];
    }

    for (j=0; j<Nsta; ++j)
        F[j][j] = 1;

    for (j=0; j<Nsta; ++j)
        F[2*j][2*j+1] = T;

    for (j=0; j<Nsta; ++j) {
        hx[j] = x[j];
        H[j][j] = 1;
    }


}
}
}
}
