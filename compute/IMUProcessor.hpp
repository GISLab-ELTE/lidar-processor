#ifndef OLP_IMUPROCESSOR_HPP
#define OLP_IMUPROCESSOR_HPP

#include "Processor.h"
#include "calculators/Calculator.h"
#include "../helpers/Utility.h"
#include "../helpers/IMUHelper.h"
#include "IMU/AHRSFilter.h"
#include "IMU/FusionAHRS/FusionFilter.hpp"
#include "IMU/MadgwickAHRS/MadgwickFilter.hpp"

namespace olp
{
namespace compute
{

template<typename PointType>
class IMUProcessor : public Calculator<PointType>, public Processor<PointType>
{
public:
    IMUProcessor(TransformData data, AHRSFilter* filter, std::vector<helper::imu::IMUReading>* source, long skipMs)
        : Calculator<PointType>(
        data), _filter(filter), _readingIterator(source->begin()), _source(source)
    {
        for (int i = 0; i < skipMs / filter->getPeriod(); i++)
        {
            updateFilter();
        }
        _initialEstimation.roll = _filter->getRoll();
        _initialEstimation.pitch = _filter->getPitch();
        _initialEstimation.yaw = _filter->getYaw();
        _initialEstimation.quaternion = _filter->getQuaternion();
    };

    ~IMUProcessor()
    {
        delete _source;
    }

    TransformData calculate(typename pcl::PointCloud<PointType>::ConstPtr input) override
    {
        _currentTimeStamp = helper::calculatePointCloudTimeStamp(input->header.stamp, 1000);

        if (_prevTimeStamp > 0)
        {
            while (_prevTimeStamp <= _currentTimeStamp)
            {
                updateFilter();
                _prevTimeStamp += 20;
            }
            _currentEstimate.pitch = _filter->getPitch() - _initialEstimation.pitch;
            _currentEstimate.yaw = _filter->getYaw() - _initialEstimation.yaw;
            _currentEstimate.roll = _filter->getRoll() - _initialEstimation.roll;
            _currentEstimate.quaternion = (_filter->getQuaternion() * _initialEstimation.quaternion.inverse()).inverse();
        }

        _prevTimeStamp = _currentTimeStamp;
        return TransformData();
    }

    typename pcl::PointCloud<PointType>::ConstPtr process(
        typename pcl::PointCloud<PointType>::ConstPtr input) {
        calculate(input);
        typename pcl::PointCloud<PointType>::Ptr transformed_cloud (new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud (*(input->makeShared()), *transformed_cloud, Eigen::Affine3f(_currentEstimate.quaternion));
        return transformed_cloud;
    }

    void updateFilter()
    {
        if (_readingIterator != _source->end())
        {
            _filter->update(_readingIterator->aX,
                            _readingIterator->aY,
                            _readingIterator->aZ,
                            _readingIterator->gX,
                            _readingIterator->gY,
                            _readingIterator->gZ,
                            _readingIterator->mX,
                            _readingIterator->mY,
                            _readingIterator->mZ);
            _readingIterator++;
        }
    }

    const std::string stringId() const override
    {
        return "IMU";
    }

    const imu::IMUEstimate& getCurrentEstimate() const
    {
        return _currentEstimate;
    }

private:
    AHRSFilter* _filter;
    std::vector<imu::IMUReading>::iterator _readingIterator;
    std::vector<imu::IMUReading>* _source;
    imu::IMUEstimate _currentEstimate;
    imu::IMUEstimate _initialEstimation;
    uint32_t _prevTimeStamp = 0;
    uint32_t _currentTimeStamp = 0;
};

} // compute
} // olp

#endif //OLP_IMUPROCESSOR_HPP
