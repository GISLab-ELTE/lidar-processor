/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_GPSHELPER_H
#define OLP_GPSHELPER_H

#include <map>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <algorithm>
#include <iomanip>

#include <boost/algorithm/string.hpp>

#include "Utility.h"

namespace olp
{
namespace helper
{
namespace gps
{
enum GPSSource
{
    stonex, mobile
};
enum GPSCoordType
{
    EOV, latlong
};

const std::map<GPSSource, std::string> GpsStringIds = {
    {stonex, "stonexGPS"},
    {mobile, "mobileGPS"}
};

struct GPS
{
    double latitude;
    double longitude;
    int secondsSinceReference;
    double accuracy;
    std::tm datetime;
    double localN;
    double localE;
    double localZ;
    double elevation;
    double cartesianX;
    double cartesianY;
    double cartesianZ;
};

struct GPSStonex : GPS
{
    int id;
    std::string antennaHeight;
    double hdop;
    double vdop;
    double pdop;
};


struct Point
{
    double x, y, z;

    Point(double px = 0, double py = 0, double pz = 0) : x(px), y(py), z(pz) {}
};


std::vector<GPS> read(const std::string& fileName, GPSSource source = stonex);

void write(const std::string& fileName, const std::vector<GPS>& gps_data);

typedef std::vector<Point> PointVector;
typedef std::map<int, PointVector> MapPointVector;

std::vector<GPS> kalmanFilter(std::vector<GPS>& gpsData, int startIndex, GPSSource source);

int getStartIndex(std::vector<GPS> gpsData, uint64_t time);

TransformData calculateTransformData(GPS& data1, GPS& data2, TransformData& actTransformation, GPSCoordType coordType = GPSCoordType::latlong);

double calculateDistance(const Point& from, const Point& to);

Point calculateDistanceFromEOV(const Point& from, const Point& to);

double calculateAngle(double dx, double dy);

double calculateAngle(const Point& from, const Point& to);

double calculateAzimuth(const Point& from, const Point& to);

void displayDistances(const MapPointVector& mp, int v1, int v2);
} // gps
} // helper
} // olp

#endif //OLP_GPSHELPER_H
