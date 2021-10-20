/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Levente Kiss
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_LOAMSLAMCALCULATOR_HPP
#define OLP_LOAMSLAMCALCULATOR_HPP

#include <thread>

#include <boost/thread/barrier.hpp>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "../../helpers/ViewerHelper.h"
#include "Calculator.h"

namespace olp
{
namespace compute
{
template<typename PointType>
class LOAMSLAMCalculator : public Calculator<PointType>
{
public:
    LOAMSLAMCalculator(TransformData data) : Calculator<PointType>(data), BEAMCOUNT_(16), _barrier(BEAMCOUNT_ + 1)
    {
        _previousCloud = pcl::PointCloud<PointType>().makeShared();
    }

    TransformData calculate(typename pcl::PointCloud<PointType>::ConstPtr input) override;

    const std::string stringId() const override
    {
        return "LOAM";
    }

protected:
    unsigned const BEAMCOUNT_;
    const double SCAN_PERIOD_ = 0.1;
    const float THRESHOLD_ = 0.1;
    const double NEARBY_SCAN_ = 2.5;
private:
    boost::barrier _barrier;
    typename pcl::PointCloud<PointType>::Ptr _previousCloud;

    inline int _pointID(const PointType& point);

    void _calculateDistortion(typename pcl::PointCloud<PointType>::Ptr points);

    void _transformToStart(PointType const* const pi,
                           PointType* const po,
                           const Eigen::Vector3d& translation,
                           const Eigen::Vector3d& rotation);


    void _findPlanarAndEdgePoint(const typename pcl::PointCloud<PointType>::ConstPtr input,
                                 std::vector<int>& planarPointIndexes,
                                 std::vector<int>& edgePointIndexes,
                                 int maxEdgePoints, int maxPlanarPoints,
                                 float threshold);

    void _findCorrespondances(const typename pcl::PointCloud<PointType>::ConstPtr previousCloud,
                              const typename pcl::PointCloud<PointType>::ConstPtr currentCloud,
                              const std::vector<int>& planarPointIndexes,
                              const std::vector<int>& edgePointIndexes,
                              Eigen::Vector3d& rotation,
                              Eigen::Vector3d& translation,
                              float threshold);
};

template<typename PointType>
TransformData LOAMSLAMCalculator<PointType>::calculate(typename pcl::PointCloud<PointType>::ConstPtr input)
{
    typename pcl::PointCloud<PointType>::Ptr correctedIn(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*input, *correctedIn);
    std::vector<int> planarPoints;
    std::vector<int> edgePoints;
    std::shared_ptr<std::map<int, float>> cValues(new std::map<int, float>);

    _calculateDistortion(correctedIn);
    _findPlanarAndEdgePoint(correctedIn, planarPoints, edgePoints, 2, 4, THRESHOLD_);

    if (_previousCloud->size() > 0)
    {
        Eigen::Vector3d rotation, translation;
        _findCorrespondances(_previousCloud, correctedIn, planarPoints, edgePoints, rotation, translation, 25);
        TransformData result = TransformData(70);
        result.transform *= Eigen::AngleAxisd(rotation(2), Eigen::Vector3d::UnitZ());
        result.transform.translation() = translation;
        return result;
    }
    pcl::copyPointCloud(*input, *_previousCloud);
    return TransformData();
}

template<typename PointType>
inline int LOAMSLAMCalculator<PointType>::_pointID(const PointType& point)
{
    float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
    int scanID = 0;
    scanID = int((angle + 15) / 2 + 0.5);
    if (scanID > (BEAMCOUNT_ - 1) || scanID < 0)
    {
        return -1;
    }
    return scanID;
}

template<typename PointType>
void LOAMSLAMCalculator<PointType>::_calculateDistortion(typename pcl::PointCloud<PointType>::Ptr points)
{
    bool halfPassed = false;
    float startOri = -atan2(points->points[0].y, points->points[0].x);
    float endOri = -atan2(points->points[points->points.size() - 1].y,
                          points->points[points->points.size() - 1].x) +
                   2 * M_PI;
    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    int cloudSize = points->points.size();
    for (int i = 0; i < cloudSize; i++)
    {
        int scanID = _pointID(points->points[i]);


        float ori = -atan2(points->points[i].y, points->points[i].x);
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        points->points[i].intensity = scanID + SCAN_PERIOD_ * relTime;
    }
}

template<typename PointType>
void LOAMSLAMCalculator<PointType>::_transformToStart(
    PointType const* const pi,
    PointType* const po,
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& rotation)
{
    //interpolation ratio
    double s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD_;

    auto scaledT = translation * s;
    auto scaledR = rotation * s;

    float rx = s * scaledR[0];
    float ry = s * scaledR[1];
    float rz = s * scaledR[2];
    float tx = s * scaledT[0];
    float ty = s * scaledT[1];
    float tz = s * scaledT[2];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;


    po->x = cos(ry) * x2 - sin(ry) * z2;
    po->y = y2;
    po->z = sin(ry) * x2 + cos(ry) * z2;
    po->intensity = pi->intensity;
}

template<typename PointType>
void LOAMSLAMCalculator<PointType>::_findPlanarAndEdgePoint(
    const typename pcl::PointCloud<PointType>::ConstPtr input,
    std::vector<int>& planarPointIndexes,
    std::vector<int>& edgePointIndexes,
    int maxEdgePoints, int maxPlanarPoints,
    float threshold)
{
    const float MAXDISTANCE = 0.05;
    std::vector<std::shared_ptr<std::vector<std::pair<int, Eigen::Vector3f>>>> pointsPerBeam;
    for (auto i = 0; i < BEAMCOUNT_; ++i)
    {
        pointsPerBeam.push_back(std::shared_ptr<std::vector<std::pair<int, Eigen::Vector3f>>>(
            new std::vector<std::pair<int, Eigen::Vector3f>>()));
    }
    for (auto i = 0; i < input->size(); ++i)
    {
        int beamIndex = _pointID(input->points[i]);
        Eigen::Vector3f tmpPt(input->points[i].x, input->points[i].y, input->points[i].z);
        pointsPerBeam[beamIndex]->push_back(std::pair<int, Eigen::Vector3f>(i, tmpPt));

    }
    std::vector<std::shared_ptr<std::vector<int>>> beamEdgesPoints;
    std::vector<std::shared_ptr<std::vector<int>>> beamPlanarPoints;


    auto f = [this, &threshold, &maxEdgePoints, &maxPlanarPoints, &MAXDISTANCE](
        const std::vector<std::pair<int, Eigen::Vector3f>>& points,
        std::shared_ptr<std::vector<int>> selectedEdgePoints,
        std::shared_ptr<std::vector<int>> selectedPlanarPoints)
    {
        std::vector<float> cloudCurvature(points.size());
        std::vector<int> cloudSortInd(points.size());
        std::vector<int> cloudNeighborPicked(points.size());
        std::vector<int> cloudLabel(points.size());
        if (points.size() > 11)
        {
            for (int i = 5; i < points.size() - 6; i++)
            {
                float diffX =
                    points.at(i - 5).second.x() + points.at(i - 4).second.x() + points.at(i - 3).second.x() +
                    points.at(i - 2).second.x() + points.at(i - 1).second.x() - 10 * points.at(i).second.x() +
                    points.at(i + 1).second.x() + points.at(i + 2).second.x() + points.at(i + 3).second.x() +
                    points.at(i + 4).second.x() + points.at(i + 5).second.x();
                float diffY =
                    points.at(i - 5).second.y() + points.at(i - 4).second.y() + points.at(i - 3).second.y() +
                    points.at(i - 2).second.y() + points.at(i - 1).second.y() - 10 * points.at(i).second.y() +
                    points.at(i + 1).second.y() + points.at(i + 2).second.y() + points.at(i + 3).second.y() +
                    points.at(i + 4).second.y() + points.at(i + 5).second.y();
                float diffZ =
                    points.at(i - 5).second.z() + points.at(i - 4).second.z() + points.at(i - 3).second.z() +
                    points.at(i - 2).second.z() + points.at(i - 1).second.z() - 10 * points.at(i).second.z() +
                    points.at(i + 1).second.z() + points.at(i + 2).second.z() + points.at(i + 3).second.z() +
                    points.at(i + 4).second.z() + points.at(i + 5).second.z();
                float c = diffX * diffX + diffY * diffY + diffZ * diffZ;
                cloudCurvature[i] = c;
                cloudSortInd[i] = i;
                cloudNeighborPicked[i] = 0;
                cloudLabel[i] = 0;
            }

            for (int i = 0; i < 6; ++i)
            {
                int startIndex = 5 + (points.size() - 10) * i / 6;
                int endIndex = 5 + (points.size() - 10) * (i + 1) / 6 - 1;
                std::sort(cloudSortInd.begin() + startIndex, cloudSortInd.begin() + endIndex + 1,
                          [&cloudCurvature](int i, int j)
                          {
                              return cloudCurvature[i] < cloudCurvature[j];
                          });
                int largestPickedNum = 0;
                for (int k = endIndex; k >= startIndex; k--)
                {
                    int ind = cloudSortInd[k];

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > threshold)
                    {

                        largestPickedNum++;

                        if (largestPickedNum <= maxEdgePoints)
                        {
                            cloudLabel[ind] = 1;
                            selectedEdgePoints->push_back(points[ind].first);
                        }
                        else
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        //not selecting close points
                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = points.at(ind + l).second.x() - points.at(ind + l - 1).second.x();
                            float diffY = points.at(ind + l).second.y() - points.at(ind + l - 1).second.y();
                            float diffZ = points.at(ind + l).second.z() - points.at(ind + l - 1).second.z();
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > MAXDISTANCE)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = points.at(ind + l).second.x() - points.at(ind + l + 1).second.x();
                            float diffY = points.at(ind + l).second.y() - points.at(ind + l + 1).second.y();
                            float diffZ = points.at(ind + l).second.z() - points.at(ind + l + 1).second.z();
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > MAXDISTANCE)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }

                }

                int smallestPickedNum = 0;
                for (int k = startIndex; k <= endIndex; k++)
                {
                    int ind = cloudSortInd[k];

                    if (ind < 1)
                    {
                        continue;
                    }

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < threshold)
                    {

                        cloudLabel[ind] = -1;
                        selectedPlanarPoints->push_back(points.at(ind).first);

                        smallestPickedNum++;
                        if (smallestPickedNum >= maxPlanarPoints)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = points.at(ind + l).second.x() - points.at(ind + l - 1).second.x();
                            float diffY = points.at(ind + l).second.y() - points.at(ind + l - 1).second.y();
                            float diffZ = points.at(ind + l).second.z() - points.at(ind + l - 1).second.z();
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > MAXDISTANCE)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = points.at(ind + l).second.x() - points.at(ind + l + 1).second.x();
                            float diffY = points.at(ind + l).second.y() - points.at(ind + l + 1).second.y();
                            float diffZ = points.at(ind + l).second.z() - points.at(ind + l + 1).second.z();
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > MAXDISTANCE)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
            }
        }
        _barrier.wait();
    };
    int i = 0;
    for (auto beamPoints : pointsPerBeam)
    {
        auto edgeTmp = std::shared_ptr<std::vector<int>>(new std::vector<int>);
        beamEdgesPoints.push_back(edgeTmp);
        auto planeTmp = std::shared_ptr<std::vector<int>>(new std::vector<int>);
        beamPlanarPoints.push_back(planeTmp);
        std::thread(f, *beamPoints, edgeTmp, planeTmp).detach();
        ++i;
    }
    _barrier.wait();
    for (auto edgePoints : beamEdgesPoints)
    {
        edgePointIndexes.insert(edgePointIndexes.end(), edgePoints->begin(), edgePoints->end());
    }
    for (auto planarPoints : beamPlanarPoints)
    {
        planarPointIndexes.insert(planarPointIndexes.end(), planarPoints->begin(), planarPoints->end());
    }
}

template<typename PointType>
void LOAMSLAMCalculator<PointType>::_findCorrespondances(
    const typename pcl::PointCloud<PointType>::ConstPtr previousCloud,
    const typename pcl::PointCloud<PointType>::ConstPtr currentCloud,
    const std::vector<int>& planarPointIndexes,
    const std::vector<int>& edgePointIndexes,
    Eigen::Vector3d& rotation,
    Eigen::Vector3d& translation,
    float threshold)
{
    Eigen::Vector3d tr = Eigen::Vector3d::Zero();
    Eigen::Vector3d rt = Eigen::Vector3d::Zero();

    typename pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<PointType>());
    typename pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<PointType>());
    kdtreeCornerLast->setInputCloud(previousCloud);
    kdtreeSurfLast->setInputCloud(previousCloud);

    int cornerPointsSharpNum = edgePointIndexes.size();
    int surfPointsFlatNum = planarPointIndexes.size();

    // neighbour edgePoint indices (J,K)
    std::map<int, int> edgeJPointIndices;
    std::map<int, int> edgeLPointIndices;

    // neighbour planarPoint indices (J,K,L)
    std::map<int, int> planarJPointIndices;
    std::map<int, int> planarLPointIndices;
    std::map<int, int> planarMPointIndices;


    PointType coeff;
    typename pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());
    typename pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());

    if (planarPointIndexes.size() > 100 && edgePointIndexes.size() > 10)
    {
        for (int iterationCount = 0; iterationCount < 25; ++iterationCount)
        {
            std::vector<int> pointSearchInd(1);
            std::vector<float> pointSearchSqDis(1);

            for (auto idx : planarPointIndexes)
            {
                PointType currentPt = currentCloud->points[idx];
                _transformToStart(&currentCloud->points[idx], &currentPt, tr, rt);
                if (iterationCount % 5 == 0)
                {
                    kdtreeSurfLast->nearestKSearch(currentPt, 1, pointSearchInd, pointSearchSqDis);
                    int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                    if (pointSearchSqDis[0] < threshold)
                    {
                        closestPointInd = pointSearchInd[0];
                        int closestPointScan = _pointID(previousCloud->points[closestPointInd]);

                        float pointSqDis, minPointSqDis2 = threshold, minPointSqDis3 = threshold;
                        for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++)
                        {
                            int currentPointID = _pointID(previousCloud->points[closestPointInd]);
                            if (currentPointID > closestPointScan + NEARBY_SCAN_)
                            {
                                break;
                            }
                            pointSqDis = (previousCloud->points[j].x - currentPt.x) *
                                         (previousCloud->points[j].x - currentPt.x) +
                                         (previousCloud->points[j].y - currentPt.y) *
                                         (previousCloud->points[j].y - currentPt.y) +
                                         (previousCloud->points[j].z - currentPt.z) *
                                         (previousCloud->points[j].z - currentPt.z);
                            if (currentPointID <= closestPointScan && pointSqDis < minPointSqDis2)
                            {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                            else if (currentPointID > closestPointScan && pointSqDis < minPointSqDis3)
                            {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                        for (int j = closestPointInd - 1; j >= 0; j--)
                        {
                            int currentPointID = _pointID(previousCloud->points[closestPointInd]);
                            if (currentPointID < closestPointScan - NEARBY_SCAN_)
                            {
                                break;
                            }
                            pointSqDis = (previousCloud->points[j].x - currentPt.x) *
                                         (previousCloud->points[j].x - currentPt.x) +
                                         (previousCloud->points[j].y - currentPt.y) *
                                         (previousCloud->points[j].y - currentPt.y) +
                                         (previousCloud->points[j].z - currentPt.z) *
                                         (previousCloud->points[j].z - currentPt.z);
                            if (currentPointID >= closestPointScan && pointSqDis < minPointSqDis2)
                            {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                            else if (_pointID(previousCloud->points[j]) < closestPointScan &&
                                     pointSqDis < minPointSqDis3)
                            {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }

                    planarJPointIndices[idx] = closestPointInd;
                    planarLPointIndices[idx] = minPointInd2;
                    planarMPointIndices[idx] = minPointInd3;
                    pointSearchInd.clear();
                    pointSearchSqDis.clear();
                }

                if (planarMPointIndices[idx] >= 0 && planarLPointIndices[idx] >= 0)
                {
                    Eigen::Vector3f J = previousCloud->points[planarJPointIndices[idx]].getVector3fMap();
                    Eigen::Vector3f L = previousCloud->points[planarLPointIndices[idx]].getVector3fMap();
                    Eigen::Vector3f M = previousCloud->points[planarMPointIndices[idx]].getVector3fMap();


                    float pa = (L[1] - J[1]) * (M[2] - J[2])
                               - (M[1] - J[1]) * (L[2] - L[2]);

                    float pb = (L[2] - J[2]) * (M[0] - J[0])
                               - (M[2] - J[2]) * (L[0] - J[0]);

                    float pc = (L[0] - J[0]) * (M[1] - J[1])
                               - (M[0] - J[0]) * (L[1] - J[1]);
                    float pd = -(pa * J[0] + pb * J[1] + pc * J[2]);

                    float ps = sqrt(pa * pa + pb * pb + pc * pc);

                    pa /= ps;
                    pb /= ps;
                    pc /= ps;
                    pd /= ps;

                    float pd2 = pa * currentPt.x + pb * currentPt.y + pc * currentPt.z + pd;


                    float s = 1;
                    if (iterationCount >= 5)
                    {
                        s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(currentPt.x * currentPt.x
                                                            + currentPt.y * currentPt.y +
                                                            currentPt.z * currentPt.z));
                    }

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1 && pd2 != 0)
                    {

                        laserCloudOri->push_back(currentCloud->points[idx]);
                        coeffSel->push_back(coeff);
                    }
                }
            }


            for (auto idx : edgePointIndexes)
            {
                PointType currentPt = currentCloud->points[idx];
                _transformToStart(&currentCloud->points[idx], &currentPt, tr, rt);
                if (iterationCount % 5 == 0)
                {
                    kdtreeCornerLast->nearestKSearch(currentPt, 1, pointSearchInd, pointSearchSqDis);
                    int closestPointInd = -1, minPointInd2 = -1;
                    if (pointSearchSqDis[0] < threshold)
                    {
                        closestPointInd = pointSearchInd[0];
                        int closestPointScan = _pointID(previousCloud->points[closestPointInd]);

                        float pointSqDis, minPointSqDis2 = threshold;
                        for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)
                        {
                            int currentPointID = _pointID(previousCloud->points[j]);
                            if (currentPointID > closestPointScan + NEARBY_SCAN_)
                            {
                                break;
                            }

                            if (currentPointID <= closestPointScan)
                            {
                                continue;
                            }
                            pointSqDis = (previousCloud->points[j].x - currentPt.x) *
                                         (previousCloud->points[j].x - currentPt.x) +
                                         (previousCloud->points[j].y - currentPt.y) *
                                         (previousCloud->points[j].y - currentPt.y) +
                                         (previousCloud->points[j].z - currentPt.z) *
                                         (previousCloud->points[j].z - currentPt.z);

                            if (pointSqDis < minPointSqDis2)
                            {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }

                        for (int j = closestPointInd - 1; j >= 0; j--)
                        {
                            int currentPointID = _pointID(previousCloud->points[j]);
                            if (currentPointID < closestPointScan - NEARBY_SCAN_)
                            {
                                break;
                            }
                            if (currentPointID >= closestPointScan)
                            {
                                continue;
                            }

                            pointSqDis = (previousCloud->points[j].x - currentPt.x) *
                                         (previousCloud->points[j].x - currentPt.x) +
                                         (previousCloud->points[j].y - currentPt.y) *
                                         (previousCloud->points[j].y - currentPt.y) +
                                         (previousCloud->points[j].z - currentPt.z) *
                                         (previousCloud->points[j].z - currentPt.z);

                            if (pointSqDis < minPointSqDis2)
                            {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                    /*********************************************/

                    edgeJPointIndices[idx] = closestPointInd;
                    edgeLPointIndices[idx] = minPointInd2;
                    pointSearchInd.clear();
                    pointSearchSqDis.clear();
                }

                if (edgeLPointIndices[idx] >= 0)
                {
                    Eigen::Vector3f I = currentPt.getVector3fMap();
                    Eigen::Vector3f J = previousCloud->points[edgeJPointIndices[idx]].getVector3fMap();
                    Eigen::Vector3f L = previousCloud->points[edgeLPointIndices[idx]].getVector3fMap();

                    float num = (I - J).cross(I - L).norm();

                    float deNum = (J - L).norm();


                    float d = num / deNum;

                    float la = ((J[1] - L[1]) * ((I[0] - J[0]) * (I[1] - L[1]) - (I[0] - L[0]) * (I[1] - J[1]))
                                + (J[2] - L[2]) * ((I[0] - J[0]) * (I[2] - L[2]) - (I[0] - L[0]) * (I[2] - J[2]))) /
                               num / deNum;

                    float lb = -((J[0] - L[0]) * ((I[0] - J[0]) * (I[1] - L[1]) - (I[0] - L[0]) * (I[1] - J[1]))
                                 -
                                 (J[2] - L[2]) * ((I[1] - J[1]) * (I[2] - L[2]) - (I[1] - L[1]) * (I[2] - J[2]))) /
                               num / deNum;

                    float lc = -((J[0] - L[0]) * ((I[0] - J[0]) * (I[2] - L[2]) - (I[0] - L[0]) * (I[2] - J[2]))
                                 +
                                 (J[1] - L[1]) * ((I[1] - J[1]) * (I[2] - L[2]) - (I[1] - L[1]) * (I[2] - J[2]))) /
                               num / deNum;

                    float s = 1;
                    if (iterationCount >= 5)
                    {
                        s = 1 - 1.8 * fabs(d);
                    }

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * d;

                    if (s > 0.1 && d != 0)
                    {
                        laserCloudOri->push_back(currentCloud->points[idx]);
                        coeffSel->push_back(coeff);
                    }
                }
            }
            int pointSelNum = laserCloudOri->points.size();

            if (pointSelNum < 10)
            {
                continue;
            }
            Eigen::MatrixXf matA = Eigen::MatrixXf::Zero(pointSelNum, 6);
            Eigen::MatrixXf matAt = Eigen::MatrixXf::Zero(6, pointSelNum);
            Eigen::MatrixXf matAtA = Eigen::MatrixXf::Zero(6, 6);
            Eigen::VectorXf matB = Eigen::VectorXf::Zero(pointSelNum);
            Eigen::VectorXf matAtB = Eigen::VectorXf::Zero(6);
            Eigen::VectorXf matX = Eigen::VectorXf::Zero(6);

            Eigen::MatrixXf matP = Eigen::MatrixXf::Zero(6, 6);
            bool isDegenerate = false;

            for (int i = 0; i < pointSelNum; i++)
            {
                PointType pointOri = laserCloudOri->points[i];
                coeff = coeffSel->points[i];

                float s = 1;

                float srx = sin(s * rt[0]);
                float crx = cos(s * rt[0]);
                float sry = sin(s * rt[1]);
                float cry = cos(s * rt[1]);
                float srz = sin(s * rt[2]);
                float crz = cos(s * rt[2]);
                float tx = s * tr[0];
                float ty = s * tr[1];
                float tz = s * tr[2];

                float arx = (-s * crx * sry * srz * pointOri.x + s * crx * crz * sry * pointOri.y +
                             s * srx * sry * pointOri.z
                             + s * tx * crx * sry * srz - s * ty * crx * crz * sry - s * tz * srx * sry) * coeff.x
                            + (s * srx * srz * pointOri.x - s * crz * srx * pointOri.y + s * crx * pointOri.z
                               + s * ty * crz * srx - s * tz * crx - s * tx * srx * srz) * coeff.y
                            + (s * crx * cry * srz * pointOri.x - s * crx * cry * crz * pointOri.y -
                               s * cry * srx * pointOri.z
                               + s * tz * cry * srx + s * ty * crx * cry * crz - s * tx * crx * cry * srz) *
                              coeff.z;

                float ary = ((-s * crz * sry - s * cry * srx * srz) * pointOri.x
                             + (s * cry * crz * srx - s * sry * srz) * pointOri.y - s * crx * cry * pointOri.z
                             + tx * (s * crz * sry + s * cry * srx * srz) +
                             ty * (s * sry * srz - s * cry * crz * srx)
                             + s * tz * crx * cry) * coeff.x
                            + ((s * cry * crz - s * srx * sry * srz) * pointOri.x
                               + (s * cry * srz + s * crz * srx * sry) * pointOri.y - s * crx * sry * pointOri.z
                               + s * tz * crx * sry - ty * (s * cry * srz + s * crz * srx * sry)
                               - tx * (s * cry * crz - s * srx * sry * srz)) * coeff.z;

                float arz = ((-s * cry * srz - s * crz * srx * sry) * pointOri.x +
                             (s * cry * crz - s * srx * sry * srz) * pointOri.y
                             + tx * (s * cry * srz + s * crz * srx * sry) -
                             ty * (s * cry * crz - s * srx * sry * srz)) * coeff.x
                            + (-s * crx * crz * pointOri.x - s * crx * srz * pointOri.y
                               + s * ty * crx * srz + s * tx * crx * crz) * coeff.y
                            + ((s * cry * crz * srx - s * sry * srz) * pointOri.x +
                               (s * crz * sry + s * cry * srx * srz) * pointOri.y
                               + tx * (s * sry * srz - s * cry * crz * srx) -
                               ty * (s * crz * sry + s * cry * srx * srz)) * coeff.z;

                float atx = -s * (cry * crz - srx * sry * srz) * coeff.x + s * crx * srz * coeff.y
                            - s * (crz * sry + cry * srx * srz) * coeff.z;

                float aty = -s * (cry * srz + crz * srx * sry) * coeff.x - s * crx * crz * coeff.y
                            - s * (sry * srz - cry * crz * srx) * coeff.z;

                float atz = s * crx * sry * coeff.x - s * srx * coeff.y - s * crx * cry * coeff.z;

                float d2 = coeff.intensity;

                matA(i, 0) = arx;
                matA(i, 1) = ary;
                matA(i, 2) = arz;
                matA(i, 3) = atx;
                matA(i, 4) = aty;
                matA(i, 5) = atz;
                matB(i) = -0.05 * d2;
            }
            matAt = matA.transpose();

            matAtA = matAt * matA;
            matAtB = matAt * matB;
            matX = matAtA.householderQr().solve(matAtB);

            if (iterationCount == 0)
            {

                Eigen::VectorXf matE = Eigen::VectorXf::Zero(6);
                Eigen::MatrixXf matV = Eigen::MatrixXf::Zero(6, 6);
                Eigen::MatrixXf matV2 = Eigen::MatrixXf::Zero(6, 6);

                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> solver(matAtA);
                matV = solver.eigenvectors();
                matE = solver.eigenvalues();
                matV2 = matV;

                isDegenerate = false;
                float eignThre[6] = {10, 10, 10, 10, 10, 10};
                for (int i = 5; i >= 0; i--)
                {
                    if (matE.transpose()(i) < eignThre[i])
                    {
                        for (int j = 0; j < 6; j++)
                        {
                            matV2(i, j) = 0;
                        }
                        isDegenerate = true;
                    }
                    else
                    {
                        break;
                    }
                }

                matP = matV.inverse() * matV2;
            }

            if (isDegenerate)
            {
                Eigen::VectorXf matX2(matX);
                matX = matP * matX2;
            }

            rt[0] += matX[0];
            rt[1] += matX[1];
            rt[2] += matX[2];
            tr[0] += matX[3];
            tr[1] += matX[4];
            tr[2] += matX[5];

            for (int i = 0; i < 3; i++)
            {
                if (isnan(tr[i]))
                    tr[i] = 0;
                if (isnan(rt[i]))
                    rt[i] = 0;
            }

            float deltaR = sqrt(
                pow((matX[0] / M_PI * 180.0), 2) +
                pow((matX[1] / M_PI * 180.0), 2) +
                pow((matX[2] / M_PI * 180.0), 2));
            float deltaT = sqrt(
                pow(matX[3] * 100, 2) +
                pow(matX[4] * 100, 2) +
                pow(matX[5] * 100, 2));

            //  std::cout << "delta R: " << deltaR << " delta T: " << deltaT << std::endl;

            if (deltaR < 0.1 && deltaT < 0.1)
            {
                break;
            }
        }
    }
    else
    {
        std::cerr << "Not enough features." << std::endl;
    }

    rotation = rt;
    translation = tr;
}

} // compute
} // olp

#endif //OLP_LOAMSLAMCALCULATOR_HPP
