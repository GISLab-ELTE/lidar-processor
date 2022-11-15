//
// Created by dominik on 02.03.22.
//

#ifndef OLP_MADGWICKFILTER_H
#define OLP_MADGWICKFILTER_H

#include "../AHRSFilter.h"
#include "MadgwickAHRS.h"


class MadgwickFilter : public AHRSFilter
{
public:
    explicit MadgwickFilter(int samplePeriod)
    {
        _filter.begin(1000.0f / (float) samplePeriod);
        _samplePeriod = samplePeriod;
    }

    void update(float aX, float aY, float aZ,
                float gX, float gY, float gZ,
                float mX, float mY, float mZ) override
    {
        _filter.update(gX, gY, gZ, aX, aY, aZ, mX, mY, mZ);
    }

    void update(float aX, float aY, float aZ,
                float gX, float gY, float gZ)
    {
        _filter.updateIMU(gX, gY, gZ, aX, aY, aZ);
    }

    float getPitch() override { return _filter.getPitch(); }

    float getRoll() override { return _filter.getRoll(); }

    float getYaw() override { return _filter.getYaw(); }

    long getPeriod() override { return _samplePeriod; }

    Eigen::Quaternionf getQuaternion() override
    {
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(_filter.getRoll(), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(_filter.getPitch(), Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(_filter.getYaw(), Eigen::Vector3f::UnitZ());
        return q;
    }

private:
    Madgwick _filter;
    int _samplePeriod;
};


#endif //OLP_MADGWICKFILTER_H
