/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#define _USE_MATH_DEFINES // for Windows to have M_PI
#include <cmath>

#include <boost/lexical_cast.hpp>
#include "Eigen/Geometry"

#include "GPSHelper.h"
#include "KalmanHelper.h"
#include "CsvFileWriter.h"
#include "TinyEKFHelper.h"

// No strptime for Windows :(
// source: https://stackoverflow.com/questions/321849/strptime-equivalent-on-windows/321940
#ifdef _WIN32
#include <time.h>
#include <iomanip>
#include <sstream>

extern "C" char* strptime(const char* s, const char* f, struct tm* tm)
{
  std::istringstream input(s);
  input.imbue(std::locale(setlocale(LC_ALL, nullptr)));
  input >> std::get_time(tm, f);
  if (input.fail()) {
    return nullptr;
  }
  return (char*)(s + input.tellg());
}
#endif

namespace olp
{
namespace helper
{
namespace gps
{

static const double DEG_TO_RAD = M_PI / 180;
static const double EARTH_RADIUS_IN_METERS = 6372797.560856;

double convert(const std::string& input)
{
    if (input == "")
        return 0;

    return boost::lexical_cast<double>(input);
}

double convertDegree(const std::string& input)
{
    std::vector<std::string> vec;
    boost::algorithm::split(vec, input, boost::is_any_of(" "));

    double degree = convert(vec[0]);
    double minutes = convert(vec[1]) / 60;
    double second = convert(vec[2]) / 3600;

    return (degree + minutes + second);
}

double calculateAccuracy(double value)
{
    return (100 / 11) * (11.5 - value);
}

GPSStonex createStonex(std::vector<std::string>& data)
{
    GPSStonex gpsData = GPSStonex();

    gpsData.id = ::atoi(data[0].c_str());
    gpsData.latitude = convertDegree(data[1]);
    gpsData.longitude = convertDegree(data[2]);
    gpsData.elevation = convert(data[3]);
    gpsData.cartesianX = convert(data[4]);
    gpsData.cartesianY = convert(data[5]);
    gpsData.cartesianZ = convert(data[6]);
    gpsData.localN = convert(data[7]);
    gpsData.localE = convert(data[8]);
    gpsData.localZ = convert(data[9]);
    gpsData.antennaHeight = data[10];
    gpsData.pdop = convert(data[16]);
    gpsData.hdop = convert(data[17]);
    gpsData.vdop = convert(data[18]);

    std::string date = data[22];
    strptime(date.data(), "%a. %d %b %Y %H:%M:%S", &gpsData.datetime);
    gpsData.secondsSinceReference = mktime(&gpsData.datetime) - 2 *  (60 * 60);
    gpsData.accuracy = calculateAccuracy(gpsData.pdop * 0.5);

    return gpsData;
}


GPS create(std::vector<std::string>& data)
{
    GPS gpsData = GPS();

    gpsData.latitude = convert(data[2]);
    gpsData.longitude = convert(data[3]);
    gpsData.accuracy = calculateAccuracy(convert(data[4]));
    gpsData.elevation = convert(data[5]);


    std::string date = data[1];
    strptime(date.data(), "%Y-%m-%d %H:%M:%S", &gpsData.datetime);
    gpsData.secondsSinceReference = mktime(&gpsData.datetime) + (60 * 60);


    return gpsData;
}


GPS createPcap(std::vector<std::string>& data)
{
    GPS gpsData;

    gpsData.latitude = convert(data[0]);
    gpsData.longitude = convert(data[1]);
    gpsData.elevation = convert(data[2]);
    gpsData.accuracy = convert(data[3]);
    gpsData.secondsSinceReference = std::stoi(data[4]);

    return gpsData;
}


int getStartIndex(std::vector<GPS> gpsData, uint64_t time)
{
    int j = 0;
    bool found = false;
    while (!found && j < gpsData.size() - 1)
    {
        if (gpsData[j].secondsSinceReference < time)
            j++;
        else
        {
            found = calculateDistance(Point(gpsData[j].latitude, gpsData[j].longitude),
                                      Point(gpsData[j + 1].latitude, gpsData[j + 1].longitude)) > 0.09;
            j++;
        }
    }

    return j;
}

std::vector<GPS> kalmanFilter(const std::vector<GPS>& gpsData, int startIndex)
{
    std::vector<GPS> newData;

    if (gpsData.empty ())
        return newData;

    TinyEKFHelper f = TinyEKFHelper();

    Point newCoord;
    GPS data = GPS();
    int secondsSinceRefrence = gpsData[startIndex].secondsSinceReference;
    int i = startIndex;
    double prevAzimuth = 0, currentAzimuth, deltaAzimuth;
    GPS prevData = gpsData[startIndex];
    bool hasMatch;

    while (i < gpsData.size())
    {
        if (i > startIndex)
            secondsSinceRefrence++;

        data.secondsSinceReference = secondsSinceRefrence;
        if (secondsSinceRefrence == gpsData[i].secondsSinceReference)
        {
            f.setRMatrix((1000 - gpsData[i].accuracy) * 10);

            if(i == startIndex)
                f.update(gpsData[i], gpsData[i]);
            else
                f.update(gpsData[i], newData[newData.size() - 1]);
            data.accuracy = gpsData[i].accuracy;
            i++;
            hasMatch = true;
        }
        else
        {
            hasMatch = false;
            f.update(gpsData[i], newData[newData.size() - 1]);
            //data.accuracy = newData[newData.size() - 1].accuracy - 1;
        }

        newCoord = f.getNewCoord();
        data.latitude = newCoord.x;
        data.longitude = newCoord.y;
        data.elevation = newCoord.z;

        currentAzimuth = calculateAzimuth({data.latitude, data.longitude, data.elevation},
                                          {prevData.latitude, prevData.longitude, prevData.elevation});
        deltaAzimuth = abs(prevAzimuth - currentAzimuth);
        prevAzimuth = currentAzimuth;
        // penalty for points created by the filter making sharp turns
        if(!hasMatch)
            data.accuracy -= deltaAzimuth * 180;

        prevData = data;
        newData.push_back(data);
    }

    return newData;
}

std::vector<GPS> read(const std::string& fileName, GPSSource source)
{

    std::ifstream file(fileName);
    std::vector<GPS> gpsData;

    std::string line;
    std::vector<std::string> vec;

    if (file.is_open())
    {
        getline(file, line);
        while (getline(file, line))
        {
            boost::erase_all(line, "\"");
            std::replace(line.begin(), line.end(), ',', '.');

            boost::algorithm::split(vec, line, boost::is_any_of(";"));

            switch (source)
            {
                case mobile:
                    gpsData.push_back(create(vec));
                    break;
                case stonex:
                    gpsData.push_back(createStonex(vec));
                    break;
                case pcap:
                    gpsData.push_back(createPcap(vec));
                    break;
            }
        }
        file.close();
    }
    return gpsData;
}

void write(const std::string& fileName, const std::vector<GPS>& gpsData)
{
    helper::csvfile csv(fileName);
    csv << "latitude" << "longitude" << "elevation" << helper::endrow;
    for (int i = 0; i < gpsData.size(); i++)
    {
        csv << gpsData[i].latitude << gpsData[i].longitude << gpsData[i].elevation << helper::endrow;
    }
}

Point calculateDistanceFromEOV(const Point& from, const Point& to)
{
    double diffY = to.y - from.y;
    double diffX = to.x - from.x;

    return Point(diffY, diffX);
}

double haversine(const Point& from, const Point& to)
{
    double dlong = (to.y - from.y) * DEG_TO_RAD;
    double dlat = (to.x - from.x) * DEG_TO_RAD;
    double a = pow(sin(dlat / 2.0), 2) + cos(from.x * DEG_TO_RAD) * cos(to.x * DEG_TO_RAD) * pow(sin(dlong / 2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return c;
}

double calculateDistance(const Point& from, const Point& to)
{
    return EARTH_RADIUS_IN_METERS * haversine(from, to);
}

double calculateAngleOfElevation(double deltaXy, double deltaZ) {
    return  atan2(deltaZ,deltaXy);
    double d2 = (deltaZ) / deltaXy - deltaXy / (2*EARTH_RADIUS_IN_METERS);
    double d3 = asin(deltaZ/deltaXy);
}

TransformData calculateTransformData(const GPS& data1, const GPS& data2, const TransformData& actTransformation, GPSCoordType coordType)
{
    double deltaX, deltaY, deltaZ, angle, angleOfElevation = 0;
    double actAngle = getRotZ(actTransformation.transform);
    double actAngleOfElevation = getRotX(actTransformation.transform);
    switch (coordType)
    {
        case latlong:
            deltaY = calculateDistance(Point(data1.latitude, data1.longitude),
                                       Point(data2.latitude, data1.longitude));
            deltaX = calculateDistance(Point(data1.latitude, data1.longitude),
                                       Point(data1.latitude, data2.longitude));

            if (data2.longitude < data1.longitude) deltaX *= -1;
            if (data2.latitude < data1.latitude) deltaY *= -1;
            break;

        case EOV:
            Point EOVdistance = calculateDistanceFromEOV(Point(data1.localN, data1.localE),
                                                         Point(data2.localN, data2.localE));
            deltaX = EOVdistance.x;
            deltaY = EOVdistance.y;
            break;
    }

    if (deltaY != 0 || deltaX != 0)
    {
        angle = calculateAngle(deltaX, deltaY);
        angle -= actAngle;
    }

    deltaZ = data2.elevation - data1.elevation;

    if(deltaZ != 0)
    {
        double deltaXY = calculateDistance(Point(data1.latitude, data1.longitude),
                                           Point(data2.latitude, data2.longitude));

        angleOfElevation = calculateAngleOfElevation(deltaXY, deltaZ);
        angleOfElevation -= actAngleOfElevation;
    }

    TransformData result = TransformData();
    result.transform = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(angleOfElevation, Eigen::Vector3d::UnitX());
    result.transform.translation() << deltaX, deltaY, deltaZ;
    result.transform.translation() = actTransformation.transform.rotation().inverse() * result.transform.translation();
    return result;
}

double calculateAzimuth(const Point& from, const Point& to)
{

    double fromXRad = from.x * DEG_TO_RAD;
    double toXRad = to.x * DEG_TO_RAD;
    double toYRad = to.y * DEG_TO_RAD;
    double fromYRad = from.y * DEG_TO_RAD;
    double longDiffRad = (toYRad - fromYRad);

    double theta = atan2(sin(longDiffRad) * cos(toXRad),
                         cos(fromXRad) * sin(toXRad) - sin(fromXRad) * cos(toXRad) * cos(longDiffRad));
    return theta;
}

double calculateAngle(const Point& from, const Point& to)
{

    double dot = from.x * to.x + from.y * to.y;
    double det = from.x * to.y - from.y * to.x;

    double delta = atan2(det, dot);
    return delta;
}

double calculateAngle(double dx, double dy)
{
    if (dx != 0 && dy != 0)
        return (M_PI) - atan(dx / dy);
    if (dx > 0)
        return M_PI / 2;
    if (dx < 0)
        return M_PI / 2 * -1;
    if (dy > 0)
        return 0;
    if (dy < 0)
        return M_PI;

    return -1;
}


void displayDistances(const MapPointVector& mp, int v1, int v2)
{
    MapPointVector::const_iterator it1 = mp.find(v1);
    MapPointVector::const_iterator it2 = mp.find(v2);
    if (it1 != mp.end() && it2 != mp.end())
    {
        const PointVector& vec1 = it1->second;
        const PointVector& vec2 = it2->second;
        for (size_t i = 0; i < vec1.size(); ++i)
        {
            for (size_t j = 0; j < vec2.size(); ++j)
                std::cout << calculateDistance(vec1[i], vec2[j]) << "\n";
        }
    }
}

} // gps
} // helper
} // olp
