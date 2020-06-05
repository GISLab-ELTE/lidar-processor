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
        _signal.connect(handler);
    }

protected:
    boost::signals2::signal<void(typename pcl::PointCloud<PointType>::ConstPtr)> _signal;

    inline void onNewCloud(
        typename pcl::PointCloud<PointType>::ConstPtr input)
    {
        _signal(input);
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
    boost::signals2::connection connection;
    boost::signals2::signal<void(const typename pcl::PointCloud<PointType>::ConstPtr)> _signal;
    pcl::Grabber& grabber;
};

template<typename PointType>
void GrabberProducer<PointType>::start()
{
    // Retrieved point cloud callback function
    boost::function<void(const typename pcl::PointCloud<PointType>::ConstPtr&)> callback =
        boost::bind(&GrabberProducer::onNewCloud, this, _1);

    // Register Callback Function
    connection = grabber.registerCallback(callback);

    // Start Grabber
    grabber.start();
}

template<typename PointType>
void GrabberProducer<PointType>::stop()
{
    // Stop Grabber
    grabber.stop();

    // Disconnect Callback Function
    if (connection.connected())
        connection.disconnect();
}

} // compute
} // olp

#endif //OLP_PRODUCER_HPP
