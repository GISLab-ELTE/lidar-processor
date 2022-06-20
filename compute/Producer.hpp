/*
 * BSD 3-Clause License
 * Copyright (c) 2019-2020, Máté Cserép
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */
 
#ifndef OLP_PRODUCER_HPP
#define OLP_PRODUCER_HPP

#include <boost/function.hpp>
#include <boost/signals2/connection.hpp>
#include <boost/signals2/signal.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/grabber.h>

#include "../grabber/GPSVLPGrabber.h"


namespace olp
{
namespace compute
{
/**
 * Abstract point cloud producer base class
 * @tparam PointType Point type (pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGB, etc.)
 */
template<typename PointType>
class Producer
{
public:
    /**
     * Start the grabber to produce data
     */
    virtual void start() = 0;

    /**
     * Stop the grabber to produce data
     */
    virtual void stop() = 0;

    /**
     * Register a new handler for processed point cloud retrieval
     * @param f
     */
    inline void registerHandler(
        boost::function<void(typename pcl::PointCloud<PointType>::ConstPtr)> handler)
    {
        _cloudSignal.connect(handler);
    }

    /**
     * Register a new handler for processed gps packet retrieval
     * @param f
     */
    inline void registerGPSPacketHandler(
        boost::function<void(olp::grabber::GPSVLPGrabber::GPSPacketConstPtr)> handler)
    {
        _gpsSignal.connect(handler);
        _hasGPSCallback = true;
    }

protected:
    boost::signals2::signal<void(typename pcl::PointCloud<PointType>::ConstPtr)> _cloudSignal;
    boost::signals2::signal<void(olp::grabber::GPSVLPGrabber::GPSPacketConstPtr)> _gpsSignal;
    bool _hasGPSCallback = false;

    inline void onNewCloud(
        typename pcl::PointCloud<PointType>::ConstPtr input)
    {
        _cloudSignal(input);
    }

    inline void onNewGPSPacket(olp::grabber::GPSVLPGrabber::GPSPacketConstPtr input)
    {
        _gpsSignal(input);
    }
};

/**
 * Point cloud producer based on a PCL Grabber as a source
 * @tparam PointType Point type (pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGB, etc.)
 */
template<typename PointType>
class GrabberProducer : public Producer<PointType>
{
public:
    GrabberProducer(pcl::Grabber& grabber) : Producer<PointType>(), grabber(grabber) {}

    void start() override;

    void stop() override;

private:
    pcl::Grabber& grabber;
    boost::signals2::connection cloudConnection;
    boost::signals2::connection gpsConnection;
};

template<typename PointType>
void GrabberProducer<PointType>::start()
{
    // Retrieved point cloud callback function
    boost::function<void(const typename pcl::PointCloud<PointType>::ConstPtr&)> callback =
        boost::bind(&GrabberProducer::onNewCloud, this, _1);

    // Register Cloud Callback Function
    cloudConnection = grabber.registerCallback(callback);

    if(Producer<PointType>::_hasGPSCallback) {
        boost::function<void(const olp::grabber::GPSVLPGrabber::GPSPacketConstPtr&)> gpsCallback =
            boost::bind(&GrabberProducer::onNewGPSPacket, this, _1);

        gpsConnection = grabber.registerCallback(gpsCallback);
    }

    // Start Grabber
    grabber.start();
}

template<typename PointType>
void GrabberProducer<PointType>::stop()
{
    // Stop Grabber
    grabber.stop();

    // Disconnect Callback Function
    if (cloudConnection.connected())
        cloudConnection.disconnect();
    
    if (gpsConnection.connected())
        gpsConnection.disconnect();
}

} // compute
} // olp

#endif //OLP_PRODUCER_HPP
