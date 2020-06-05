//
// Created by mate on 2019.05.03..
//

#ifndef HDLPROCESSOR_PROCESSOR_HPP
#define HDLPROCESSOR_PROCESSOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>

namespace olp
{
/**
 * Point cloud processor based on a PCL Grabber as a source
 * @tparam PointType Point type (pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGB, etc.)
 */
template<typename PointType>
class Processor
{
public:
    Processor(pcl::Grabber& grabber) : grabber(grabber) {}

    virtual ~Processor() {}

    /**
     * Start the grabber to produce data
     */
    void start();

    /**
     * Stop the grabber to produce data
     */
    void stop();

    /**
     * Register a new handler for processed point cloud retrieval
     * @param f
     */
    inline void registerHandler(
        boost::function<void(const typename pcl::PointCloud<PointType>::ConstPtr)> handler)
    {
        _signal.connect(handler);
    }

protected:
    inline void onNewCloud(
        const typename pcl::PointCloud<PointType>::ConstPtr& input)
    {
        typename pcl::PointCloud<PointType>::ConstPtr output = process(input);
        _signal(output);
    }

    /**
     * Point cloud process function to override
     * @param input Input point cloud
     * @return Processed output point cloud
     */
    virtual typename pcl::PointCloud<PointType>::ConstPtr process(
        const typename pcl::PointCloud<PointType>::ConstPtr input) = 0;

private:
    boost::signals2::connection connection;
    boost::signals2::signal<void(typename pcl::PointCloud<PointType>::ConstPtr)> _signal;
    pcl::Grabber& grabber;
};

template<typename PointType>
void Processor<PointType>::start()
{
    // Retrieved point cloud callback function
    boost::function<void(const typename pcl::PointCloud<PointType>::ConstPtr&)> callback =
        boost::bind(&Processor::onNewCloud, this, _1);

    // Register Callback Function
    connection = grabber.registerCallback(callback);

    // Start Grabber
    grabber.start();
}

template<typename PointType>
void Processor<PointType>::stop()
{
    // Stop Grabber
    grabber.stop();

    // Disconnect Callback Function
    if (connection.connected())
        connection.disconnect();
}

/**
 * Default Processor implementation with an indentity transformation
 * @tparam PointType Point type (pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGB, etc.)
 */
template<typename PointType>
class IdentityProcessor : public Processor<PointType>
{
public:
    IdentityProcessor(pcl::Grabber& grabber) : Processor<PointType>(grabber) {}

protected:
    typename pcl::PointCloud<PointType>::ConstPtr process(
        const typename pcl::PointCloud<PointType>::ConstPtr input) override
    {
        return input;
    }
};
} //olp

#endif //HDLPROCESSOR_PROCESSOR_HPP
