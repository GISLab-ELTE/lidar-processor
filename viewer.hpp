/*
 * BSD 3-Clause License
 * Copyright (c) 2019-2020, Máté Cserép
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef HDLPROCESSOR_VIEWER_HPP
#define HDLPROCESSOR_VIEWER_HPP

#include <memory>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace olp
{
/**
 * Point cloud visualizer based on the PCLVisualizer
 * @tparam PointType Point type (pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGB, etc.)
 */
template<typename PointType>
class Viewer
{
public:
    Viewer(
        pcl::visualization::PointCloudColorHandler<PointType>& handler)
        : _viewer(new pcl::visualization::PCLVisualizer("Online LiDAR Viewer")),
          _handler(handler)
    {
        _viewer->addCoordinateSystem(3.0);
        _viewer->setBackgroundColor(0.0, 0.0, 0.0);
        _viewer->initCameraParameters();
        _viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
    }

    /**
     * Run 3D point cloud visualizer
     */
    void run();
    void showCloud(typename pcl::PointCloud<PointType>::Ptr cloud);

    /**
     * Update visualized point cloud
     * @param cloud New point cloud to display
     */
    void update(typename pcl::PointCloud<PointType>::ConstPtr cloud);

private:
    pcl::visualization::PCLVisualizer::Ptr _viewer;
    pcl::visualization::PointCloudColorHandler<PointType>& _handler;
    typename pcl::PointCloud<PointType>::ConstPtr _cloud;
    std::mutex _mutex;
    bool _hasUpdate = false;
};

template<typename PointType>
void Viewer<PointType>::update(typename pcl::PointCloud<PointType>::ConstPtr cloud)
{
    std::lock_guard<std::mutex> lock(_mutex);
    cloud.swap(_cloud);
    _hasUpdate = true;
}

template<typename PointType>
void Viewer<PointType>::run()
{
    while (!_viewer->wasStopped())
    {
        // Update viewer
        _viewer->spinOnce();
        std::unique_lock<std::mutex> lock(_mutex, std::try_to_lock);
        if (lock.owns_lock() && _cloud && _hasUpdate)
        {
            // Update point cloud
            _handler.setInputCloud(_cloud);
            if (!_viewer->updatePointCloud(_cloud, _handler))
                _viewer->addPointCloud(_cloud, _handler,"cloud");
            _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
            _hasUpdate = false;
        }
    }
}

} //olp

#endif //HDLPROCESSOR_VIEWER_HPP
