//
// Created by dominik on 02.03.22.
//

#ifndef OLP_AHRSFILTER_H
#define OLP_AHRSFILTER_H


class AHRSFilter
{
public:
    virtual void update(float aX, float aY, float aZ,
                        float gX, float gY, float gZ,
                        float mX, float mY, float mZ) = 0;

    virtual float getPitch() = 0;

    virtual float getRoll() = 0;

    virtual float getYaw() = 0;

    virtual long getPeriod() = 0;

    virtual Eigen::Quaternionf getQuaternion() = 0;
};


#endif //OLP_AHRSFILTER_H
