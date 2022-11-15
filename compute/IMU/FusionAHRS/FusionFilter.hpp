#ifndef OLP_FUSIONFILTER_HPP
#define OLP_FUSIONFILTER_HPP

#include "../AHRSFilter.h"
#include "Fusion.h"

class FusionFilter : public AHRSFilter
{
public:
    explicit FusionFilter(float samplePeriodS, float gyroscopeSensitivity, float accelerometerSensitivity,
                          float magnetometerSensitivity,
                          FusionVector3 hardIronBias, FusionRotationMatrix softIronMatrix)
    {
        _magnetometerSensitivity = magnetometerSensitivity;

        _gyroscopeSensitivity.axis.x = gyroscopeSensitivity;
        _gyroscopeSensitivity.axis.y = gyroscopeSensitivity;
        _gyroscopeSensitivity.axis.z = gyroscopeSensitivity;

        _accelerometerSensitivity.axis.x = accelerometerSensitivity;
        _accelerometerSensitivity.axis.y = accelerometerSensitivity;
        _accelerometerSensitivity.axis.z = accelerometerSensitivity;

        _hardIronBias = hardIronBias;
        _softIronMatrix = softIronMatrix;

        _samplePeriod = samplePeriodS;

        FusionBiasInitialise(&_fusionBias, 2.0f, samplePeriodS);

        FusionAhrsInitialise(&_fusionAhrs, 0.5f); // gain = 0.5

        FusionAhrsSetMagneticField(&_fusionAhrs, 0.0f, 70.0f); // valid magnetic field range = 20 uT to 70 uT
    }

    void update(float aX, float aY, float aZ,
                float gX, float gY, float gZ,
                float mX, float mY, float mZ) override
    {
        FusionVector3 uncalibratedGyroscope;
        uncalibratedGyroscope.axis.x = gX;
        uncalibratedGyroscope.axis.y = gY;
        uncalibratedGyroscope.axis.z = gZ;

        FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope,
                                                                      FUSION_ROTATION_MATRIX_IDENTITY,
                                                                      _gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

        FusionVector3 uncalibratedAccelerometer;
        uncalibratedAccelerometer.axis.x = aX;
        uncalibratedAccelerometer.axis.y = aY;
        uncalibratedAccelerometer.axis.z = aZ;
        FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer,
                                                                          FUSION_ROTATION_MATRIX_IDENTITY,
                                                                          _accelerometerSensitivity,
                                                                          FUSION_VECTOR3_ZERO);

        FusionVector3 uncalibratedMagnetometer;
        uncalibratedMagnetometer.axis.x = mX * _magnetometerSensitivity;
        uncalibratedMagnetometer.axis.y = mY * _magnetometerSensitivity;
        uncalibratedMagnetometer.axis.z = mZ * _magnetometerSensitivity;

        FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer,
                                                                         _softIronMatrix,
                                                                         _hardIronBias);

        calibratedGyroscope = FusionBiasUpdate(&_fusionBias, calibratedGyroscope);

        FusionAhrsUpdate(&_fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer,
                         _samplePeriod);
        _eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&_fusionAhrs));
    }

    float getPitch() override { return _eulerAngles.angle.pitch; }

    float getRoll() override { return _eulerAngles.angle.roll; }

    float getYaw() override { return _eulerAngles.angle.yaw; }

    long getPeriod() override { return (int) (_samplePeriod * 1000); }

    Eigen::Quaternionf getQuaternion() override
    {
        FusionQuaternion fq = FusionAhrsGetQuaternion(&_fusionAhrs);
        return (Eigen::Quaternionf) {
            fq.element.w,
            fq.element.x,
            fq.element.y,
            fq.element.z
        };
    }

private:
    float _samplePeriod;
    float _magnetometerSensitivity;

    FusionBias _fusionBias;
    FusionAhrs _fusionAhrs;

    FusionVector3 _gyroscopeSensitivity;
    FusionVector3 _accelerometerSensitivity;
    FusionVector3 _hardIronBias;
    FusionRotationMatrix _softIronMatrix;

    FusionEulerAngles _eulerAngles;
};

#endif //OLP_FUSIONFILTER_HPP
