/*
 * BSD 3-Clause License
 * Copyright (c) 2019-2021, Máté Cserép & Péter Farkas
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

#include "helpers/ViewerHelper.h"

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
          _handler(handler),
          _prevTrajectoryPoint(PointType())
    {
        _viewer->addCoordinateSystem(3.0);
        _viewer->setBackgroundColor(0.0, 0.0, 0.0);
        _viewer->initCameraParameters();
        _viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
    }

    /**
     * Add data shared with cloud transformer
     */
    void addShareData(std::shared_ptr<olp::helper::ViewerShareData<PointType>> shareData)
    {
        _shareData = shareData;
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
    std::shared_ptr<olp::helper::ViewerShareData<PointType>> _shareData;
    PointType _prevTrajectoryPoint;
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
    uint64_t segmentId = 0;
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
            // Add precision description and next point to trajectory line
            if(_shareData){
                const int yOffset = 20;
                int i = 0;
                for(auto it = _shareData->precisionMap.begin(); it != _shareData->precisionMap.end(); ++it)
                {
                    std::string line = it->first + ": " + std::to_string(it->second);
                    if(!_viewer->updateText(line, 0, (20 + i * yOffset), it->first))
                    {
                        olp::helper::Color color = olp::helper::getCalculatorColor(it->first);
                        _viewer->addText(line, 0, (20 + i * yOffset), color.r, color.g, color.b, it->first);
                    }
                    ++i;
                }
                while(!_shareData->trajectoryQueue.empty()){
                    auto current = _shareData->trajectoryQueue.front();
                    // Add next trajectory line
                    olp::helper::Color bestColor = olp::helper::getCalculatorColor(current.second);
                    std::string lineStringId = "line" + std::to_string(segmentId);
                    _viewer->addLine(current.first, _prevTrajectoryPoint,
                                    bestColor.r, bestColor.g, bestColor.b,
                                    lineStringId);
                    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, lineStringId);
                    _viewer->addSphere(current.first, 0.2, bestColor.r, bestColor.g, bestColor.b, "sphere" + std::to_string(segmentId));
                    _prevTrajectoryPoint = current.first;
                    ++segmentId;
                    _shareData->trajectoryQueue.pop();
                }
            }
        }
    }
}

} //olp

#endif //HDLPROCESSOR_VIEWER_HPP
