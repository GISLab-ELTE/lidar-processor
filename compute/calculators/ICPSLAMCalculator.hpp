/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Levente Kiss
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_ICPSLAMCALCULATOR_HPP
#define OLP_ICPSLAMCALCULATOR_HPP

#include <pointmatcher/PointMatcher.h>

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
    ICPSLAMCalculator(double angle, int processEveryX = 1) : Calculator<PointType>(angle),
                                                             PROCESSEVERYX_(processEveryX) {}

    TransformData calculate(typename pcl::PointCloud<PointType>::ConstPtr input) override;

protected:
    PM::ICP icp_;
    const int PROCESSEVERYX_;

    void configureICP_();

    float calculateError_(const DP& referenceCloud, const DP& readingCloud, const Eigen::MatrixXf& transform);

    inline void
    convertToDataPointsAndApplyDefaultRotation_(typename pcl::PointCloud<PointType>::ConstPtr input, DP& out);

    inline TransformData convertToTransformData_(const Eigen::Matrix4f& transformationMatrix);

private:
    DP _clouds[2];
    int _cloudCount = 0;

};

template<typename PointType>
TransformData ICPSLAMCalculator<PointType>::calculate(typename pcl::PointCloud<PointType>::ConstPtr input)
{
    if (_cloudCount % PROCESSEVERYX_ == 0)
    {
        DP cloud;
        convertToDataPointsAndApplyDefaultRotation_(input, cloud);
        if (_cloudCount == 0)
        {
            _clouds[0] = cloud;
            _cloudCount++;
            return TransformData();
        }
        configureICP_();
        const DP reference = cloud;
        const DP reading = _clouds[0];
        PM::TransformationParameters transform = icp_(reading, reference);
        _clouds[0] = cloud;

//            std::cerr << "---------------" << std::endl
//                    << "icp error: " << calculateError_(reference,reading,transform) << std::endl
//                    << "---------------" << std::endl << std::endl;
        _cloudCount++;
        return convertToTransformData_(transform);
    }
    else
    {
        _cloudCount++;
        return TransformData();
    }
}

template<typename PointType>
void ICPSLAMCalculator<PointType>::configureICP_()
{
    icp_ = PM::ICP();
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
    icp_.readingDataPointsFilters.push_back(rand_read);

    icp_.referenceDataPointsFilters.push_back(rand_ref);
    icp_.referenceDataPointsFilters.push_back(sensor_direction);
    icp_.referenceDataPointsFilters.push_back(normal);
    icp_.referenceDataPointsFilters.push_back(normal_orient);

    icp_.matcher.swap(kdtree);

    icp_.outlierFilters.push_back(dist);

    icp_.errorMinimizer.swap(pointToPlane);

    icp_.transformationCheckers.push_back(maxTrans);
    icp_.transformationCheckers.push_back(maxIter);
    icp_.transformationCheckers.push_back(diff);

    icp_.inspector.swap(nullInspect);

    icp_.transformations.push_back(rigidTrans);
}

template<typename PointType>
float ICPSLAMCalculator<PointType>::calculateError_(const DP& referenceCloud, const DP& readingCloud,
                                                    const Eigen::MatrixXf& transform)
{
    DP data_out(readingCloud);
    icp_.transformations.apply(data_out, transform);

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

    icp_.matcher->init(referenceWithNormals);
    // 2) Get matches between transformed data and ref
    PM::Matches matches = icp_.matcher->findClosests(data_out);
    // 3) Get outlier weights for the matches
    PM::OutlierWeights outlierWeights = icp_.outlierFilters.compute(data_out, referenceWithNormals, matches);
    // 4) Compute error
    return icp_.errorMinimizer->getResidualError(data_out, referenceWithNormals, outlierWeights, matches);
}

template<typename PointType>
inline void
ICPSLAMCalculator<PointType>::convertToDataPointsAndApplyDefaultRotation_(
    typename pcl::PointCloud<PointType>::ConstPtr input, DP& out)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(this->angle, Eigen::Vector3f::UnitZ()));

    Eigen::MatrixXf points = Eigen::MatrixXf::Zero(3, input->points.size());
    for (int i = 0; i < input->points.size(); ++i)
    {
        Eigen::Vector3f pt = input->points[i].getVector3fMap();
        points.block<3, 1>(0, i) = transform * pt;
    }
    out = DP();
    out.addFeature("x", points.row(0));
    out.addFeature("y", points.row(1));
    out.addFeature("z", points.row(2));
}

template<typename PointType>
inline TransformData ICPSLAMCalculator<PointType>::convertToTransformData_(const Eigen::Matrix4f& transformationMatrix)
{
    double rotX, rotY, rotZ;
    rotX = atan2(transformationMatrix(2, 1), transformationMatrix(2, 2));
    rotY = atan2(-transformationMatrix(2, 0), std::pow(transformationMatrix(2, 1) * transformationMatrix(2, 1) +
                                                       transformationMatrix(2, 2) * transformationMatrix(2, 2),
                                                       0.5));
    rotZ = atan2(transformationMatrix(1, 0), transformationMatrix(0, 0));
    return TransformData(-transformationMatrix(0, 3), -transformationMatrix(1, 3), -transformationMatrix(2, 3),
                         -rotZ, 70);

}

} // compute
} // olp

#endif //OLP_ICPSLAMCALCULATOR_HPP
