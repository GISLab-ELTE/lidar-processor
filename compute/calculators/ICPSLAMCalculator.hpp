/*
 * BSD 3-Clause License
 * Copyright (c) 2020-2021, Levente Kiss & Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_ICPSLAMCALCULATOR_HPP
#define OLP_ICPSLAMCALCULATOR_HPP

#include <pointmatcher/PointMatcher.h>

#include "../../helpers/ViewerHelper.h"
#include "../../helpers/Utility.h"
#include "Calculator.h"

namespace olp
{
namespace compute
{

using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

template<typename PointType>
class ICPSLAMCalculator : public Calculator<PointType>
{
public:
    ICPSLAMCalculator(const TransformData& data, int processEveryX = 1) : Calculator<PointType>(data),
                                                             PROCESSEVERYX(processEveryX) {}

    TransformData calculate(typename pcl::PointCloud<PointType>::ConstPtr input) override;

    const std::string stringId() const override
    {
        return "ICP";
    }

protected:
    PM::ICP icp;
    const int PROCESSEVERYX;

    void configureICP();

    float calculateError(const DP& referenceCloud, const DP& readingCloud, const Eigen::MatrixXf& transform);

    inline void
    convertToDataPoints(typename pcl::PointCloud<PointType>::ConstPtr input, DP& out);

private:
    DP _clouds[2];
    int _cloudCount = 0;

};

template<typename PointType>
TransformData ICPSLAMCalculator<PointType>::calculate(typename pcl::PointCloud<PointType>::ConstPtr input)
{
    if (_cloudCount % PROCESSEVERYX == 0)
    {
        DP cloud;
        convertToDataPoints(input, cloud);
        if (_cloudCount == 0)
        {
            _clouds[0] = cloud;
            _cloudCount++;
            return TransformData();
        }
        configureICP();
        const DP reference = cloud;
        const DP reading = _clouds[0];
        try {
            PM::TransformationParameters transform = icp(reading, reference);
            _clouds[0] = cloud;
            _cloudCount++;
            return TransformData(transform.inverse().cast<double>(), 70);
        }
        // for non-converging ICP
        catch(...) {
            return TransformData();
        }

//            std::cerr << "---------------" << std::endl
//                    << "icp error: " << calculateError(reference,reading,transform) << std::endl
//                    << "---------------" << std::endl << std::endl;

    }
    else
    {
        _cloudCount++;
        return TransformData();
    }
}

template<typename PointType>
void ICPSLAMCalculator<PointType>::configureICP()
{
    icp = PM::ICP();
    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name;

    // Prepare reading filters
    name = "RandomSamplingDataPointsFilter";
    params["prob"] = "1";
    std::shared_ptr<PM::DataPointsFilter> rand_read =
        PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    // Prepare reference filters
    name = "RandomSamplingDataPointsFilter";
    params["prob"] = "1";
    std::shared_ptr<PM::DataPointsFilter> rand_ref =
        PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "ObservationDirectionDataPointsFilter";
    std::shared_ptr<PM::DataPointsFilter> sensor_direction =
        PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "SurfaceNormalDataPointsFilter";
    params["knn"] = "10";
    std::shared_ptr<PM::DataPointsFilter> normal =
        PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "OrientNormalsDataPointsFilter";
    std::shared_ptr<PM::DataPointsFilter> normal_orient =
        PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "KDTreeMatcher";
    params["knn"] = "3";
    std::shared_ptr<PM::Matcher> kdtree =
        PM::get().MatcherRegistrar.create(name, params);
    params.clear();

    name = "MaxDistOutlierFilter";
    params["maxDist"] = "1";
    std::shared_ptr<PM::OutlierFilter> dist =
        PM::get().OutlierFilterRegistrar.create(name, params);
    params.clear();

    name = "PointToPlaneErrorMinimizer";
    std::shared_ptr<PM::ErrorMinimizer> pointToPlane =
        PM::get().ErrorMinimizerRegistrar.create(name);

    name = "BoundTransformationChecker";
    params["maxRotationNorm"] = "0.8";
    params["maxTranslationNorm"] = "15";
    std::shared_ptr<PM::TransformationChecker> maxTrans =
        PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    name = "CounterTransformationChecker";
    params["maxIterationCount"] = "80";
    std::shared_ptr<PM::TransformationChecker> maxIter =
        PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    name = "DifferentialTransformationChecker";
    params["minDiffRotErr"] = "0.001";
    params["minDiffTransErr"] = "0.01";
    params["smoothLength"] = "2";
    std::shared_ptr<PM::TransformationChecker> diff =
        PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    std::shared_ptr<PM::Inspector> nullInspect =
        PM::get().InspectorRegistrar.create("NullInspector");

    std::shared_ptr<PM::Transformation> rigidTrans =
        PM::get().TransformationRegistrar.create("RigidTransformation");

    // Build ICP solution
    icp.readingDataPointsFilters.push_back(rand_read);

    icp.referenceDataPointsFilters.push_back(rand_ref);
    icp.referenceDataPointsFilters.push_back(sensor_direction);
    icp.referenceDataPointsFilters.push_back(normal);
    icp.referenceDataPointsFilters.push_back(normal_orient);

    icp.matcher.swap(kdtree);

    icp.outlierFilters.push_back(dist);

    icp.errorMinimizer.swap(pointToPlane);

    icp.transformationCheckers.push_back(maxTrans);
    icp.transformationCheckers.push_back(maxIter);
    icp.transformationCheckers.push_back(diff);

    icp.inspector.swap(nullInspect);

    icp.transformations.push_back(rigidTrans);
}

template<typename PointType>
float ICPSLAMCalculator<PointType>::calculateError(const DP& referenceCloud, const DP& readingCloud,
                                                    const Eigen::MatrixXf& transform)
{
    DP data_out(readingCloud);
    icp.transformations.apply(data_out, transform);

    PM::DataPointsFilters f;

    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name;

    name = "ObservationDirectionDataPointsFilter";
    std::shared_ptr<PM::DataPointsFilter> sensor_direction =
        PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "SurfaceNormalDataPointsFilter";
    params["knn"] = "10";
    std::shared_ptr<PM::DataPointsFilter> normal =
        PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "OrientNormalsDataPointsFilter";
    std::shared_ptr<PM::DataPointsFilter> normal_orient =
        PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    DP referenceWithNormals(referenceCloud);

    f.push_back(sensor_direction);
    f.push_back(normal);
    f.push_back(normal_orient);
    f.apply(referenceWithNormals);

    icp.matcher->init(referenceWithNormals);
    // 2) Get matches between transformed data and ref
    PM::Matches matches = icp.matcher->findClosests(data_out);
    // 3) Get outlier weights for the matches
    PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(data_out, referenceWithNormals, matches);
    // 4) Compute error
    return icp.errorMinimizer->getResidualError(data_out, referenceWithNormals, outlierWeights, matches);
}

template<typename PointType>
inline void
ICPSLAMCalculator<PointType>::convertToDataPoints(
    typename pcl::PointCloud<PointType>::ConstPtr input, DP& out)
{
    Eigen::MatrixXf points = Eigen::MatrixXf::Zero(3, input->points.size());
    for (int i = 0; i < input->points.size(); ++i)
    {
        Eigen::Vector3f pt = input->points[i].getVector3fMap();
        points.block<3, 1>(0, i) = pt;
    }
    out = DP();
    out.addFeature("x", points.row(0));
    out.addFeature("y", points.row(1));
    out.addFeature("z", points.row(2));
}

} // compute
} // olp

#endif //OLP_ICPSLAMCALCULATOR_HPP
