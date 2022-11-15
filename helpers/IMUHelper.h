//
// Created by dominik on 03.03.22.
//

#ifndef OLP_IMUHELPER_H
#define OLP_IMUHELPER_H

namespace olp
{
namespace helper
{
namespace imu
{
struct IMUReading
{
    IMUReading(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) :
        aX(ax), aY(ay), aZ(az), gX(gx), gY(gy), gZ(gz), mX(mx), mY(my), mZ(mz) {}

    float aX;
    float aY;
    float aZ;
    float gX;
    float gY;
    float gZ;
    float mX;
    float mY;
    float mZ;
};

struct IMUEstimate
{
    IMUEstimate() : pitch(0), yaw(0), roll(0), quaternion(Eigen::Quaternionf{0.0f, 0.0f, 0.0f, 0.0f}) {}

    IMUEstimate(float p, float y, float r, Eigen::Quaternionf q) :
        pitch(p), yaw(y), roll(r), quaternion(q) {}

    float pitch;
    float yaw;
    float roll;
    Eigen::Quaternionf quaternion;
};
}
}
}

#endif //OLP_IMUHELPER_H
