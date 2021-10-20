/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender & Máté Cserép
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_CONSUMER_HPP
#define OLP_CONSUMER_HPP

#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <string>

#include <boost/algorithm/string/replace.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLHeader.h>

#include "../helpers/LASHelper.h"
#include "../helpers/FileHelper.h"
#include "../helpers/Utility.h"

#include "Processor.h"
#include "../viewer.hpp"

namespace olp
{
namespace compute
{
template<typename PointType>
class Consumer
{
public:
    virtual void show(typename pcl::PointCloud<PointType>::Ptr input) = 0;
    virtual void addShareData(std::shared_ptr<olp::helper::ViewerShareData<PointType>> shareData) = 0;
};

template<typename PointType>
class ViewConsumer : public Consumer<PointType>
{
public:
    ViewConsumer(olp::Viewer<PointType>& viewer) : Consumer<PointType>(), viewer(viewer) {}

    void show(typename pcl::PointCloud<PointType>::Ptr input) override
    {
        viewer.update(input);
    }

    void addShareData(std::shared_ptr<olp::helper::ViewerShareData<PointType>> shareData) override
    {
        viewer.addShareData(shareData);
    }

private:
    olp::Viewer<PointType>& viewer;
};

template<typename PointType>
class FileWriter : public Consumer<PointType>
{
public:
    FileWriter(const std::string& pcapFile, const std::string& writeFileType, uint64_t start) : Consumer<PointType>()
    {
        pcap = pcapFile;
        fileType = writeFileType;
        startTime = start;
    }

    void show(typename pcl::PointCloud<PointType>::Ptr input) override
    {
        writeFile(input);
    }

    void addShareData(std::shared_ptr<olp::helper::ViewerShareData<PointType>> shareData) override
    {
        throw std::logic_error("Function not implemented");
    }

protected:
    std::string pcap;
    std::string fileType;
    uint64_t startTime;

    std::string currentDate();

    void writeFile(typename pcl::PointCloud<PointType>::ConstPtr cloud);
};

template<typename PointType>
std::string FileWriter<PointType>::currentDate()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d");
    return ss.str();
}

template<typename PointType>
void FileWriter<PointType>::writeFile(typename pcl::PointCloud<PointType>::ConstPtr cloud)
{
    std::string dir;
    uint64_t time;

    if (!pcap.empty())
    {
        dir = boost::replace_all_copy(pcap, ".pcap", "");
        //time = startTime + helper::calculatePointCloudTimeStamp(cloud->header.stamp);
    }
    else
    {
        dir = "lidar_" + currentDate();
    }

    time = std::time(0);

    dir += "_" + fileType;

    helper::createDirectory(dir);
    std::string filename = dir + "/" + std::to_string(time) + "." + fileType;

    if (fileType == "las")
        helper::writeLAS(filename, *helper::createLASHeader(), cloud);
    else
        pcl::io::savePCDFileBinary(filename, *cloud);
}

} // compute
} // olp

#endif //OLP_CONSUMER_HPP
