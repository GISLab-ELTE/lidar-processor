/*
 * BSD 3-Clause License
 * Copyright (c) 2019-2020, Máté Cserép
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_grabber.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "viewer.hpp"

#include "helpers/GPSHelper.h"
#include "helpers/FileHelper.h"
#include "helpers/ViewerHelper.h"

#include "piping/ProcessorPipe.hpp"

#include "compute/Processor.h"
#include "compute/Consumer.hpp"
#include "compute/Filter.hpp"
#include "compute/Producer.hpp"
#include "compute/Transformer.hpp"
#include "compute/GPSAligner.hpp"
#include "compute/calculators/GPSCalculator.hpp"
#include "compute/calculators/GPSPacketCalculator.hpp"
#include "grabber/GPSVLPGrabber.h"

#ifdef WITH_SLAM
#include "compute/calculators/ICPSLAMCalculator.hpp"
#include "compute/calculators/LOAMSLAMCalculator.hpp"
#endif

#include "compute/IMUProcessor.hpp"

using namespace olp;
using namespace olp::helper;

compute::GPSCalculator<pcl::PointXYZI>* createGPSCalculator(const std::string& csv, uint64_t time, gps::GPSSource source, const std::string& fileName) {
    std::cout << "Capture from CSV file: " << csv << std::endl;

    std::vector<gps::GPS> gpsData = gps::read(csv, source);

    int j = gps::getStartIndex(gpsData, time);
    std::vector<gps::GPS> filteredGpsData;

    helper::TransformData startData = TransformData();
    if(j < gpsData.size()) {
        std::vector<gps::GPS> gpsDataWithKalmanFilter = gps::kalmanFilter(gpsData, j);
        gps::write(fileName, gpsDataWithKalmanFilter);

        int start = gps::getStartIndex(gpsDataWithKalmanFilter, time);
        std::vector<gps::GPS>::const_iterator first = gpsDataWithKalmanFilter.begin() + start;
        std::vector<gps::GPS>::const_iterator last = gpsDataWithKalmanFilter.end();
        filteredGpsData = std::vector<gps::GPS>(first, last);

        if(filteredGpsData.size() >= 3) {
            helper::TransformData act = TransformData();
            startData = gps::calculateTransformData(filteredGpsData[0], filteredGpsData[2], act);
        }
    }

    compute::GPSCalculator<pcl::PointXYZI>* gpsCalculator = new compute::GPSCalculator<pcl::PointXYZI>(filteredGpsData, time, startData, source);
    return gpsCalculator;
}

compute::IMUProcessor<pcl::PointXYZI>* createIMUCalculator(const std::string& fileName)
{
    int tmp;

    float gyroX, gyroY, gyroZ;
    int magX, magY, magZ, accX, accY, accZ;

    auto* readings = new std::vector<imu::IMUReading>();

    ifstream imuStream(fileName);
    int skipMs;
    float samplePeriodMs, gyroSensitivity, accSensitivity, magSensitivity;

    float magxmax = 0, magxmin = 0, magymax = 0, magymin = 0, magzmax = 0, magzmin = 0;

    imuStream >> skipMs >> samplePeriodMs >> gyroSensitivity >> accSensitivity >> magSensitivity;
    while (!imuStream.eof())
    {
        imuStream >> tmp >> gyroX >> gyroY >> gyroZ
                  >> accX >> accY >> accZ
                  >> magX >> magY >> magZ;
        readings->emplace_back(accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ);

        if(magX * magSensitivity > magxmax) magxmax = magX * magSensitivity;
        if(magX * magSensitivity < magxmin) magxmin = magX * magSensitivity;
        if(magY * magSensitivity > magymax) magymax = magY * magSensitivity;
        if(magY * magSensitivity < magymin) magymin = magY * magSensitivity;
        if(magZ * magSensitivity > magzmax) magzmax = magZ * magSensitivity;
        if(magZ * magSensitivity < magzmin) magzmin = magZ * magSensitivity;
    }
    imuStream.close();

    float hardIronX = (magxmax + magxmin) / 2;
    float hardIronY = (magymax + magymin) / 2;
    float hardIronZ = (magzmax + magzmin) / 2;

    float scaleX = (magxmax - magxmin) / 2;
    float scaleY = (magymax - magymin) / 2;
    float scaleZ = (magzmax - magzmin) / 2;

    float avg = (scaleX + scaleY + scaleZ) / 3.0f;

    FusionRotationMatrix softIron{{avg * 1 / scaleX, 0, 0, 0, avg * 1 / scaleY, 0, 0, 0, avg * 1 / scaleZ}};
    FusionVector3 hardIron{hardIronX, hardIronY, hardIronZ};

    return new compute::IMUProcessor<pcl::PointXYZI>(
        TransformData(),
        new FusionFilter(0.02, 0.07f, 0.000244f, 0.029231219f,
                         hardIron, softIron),
        readings, skipMs
    );
}


int main(int argc, char *argv[])
{
    // Command-Line argument parsing
    if (pcl::console::find_switch(argc, argv, "--help") ||
        pcl::console::find_switch(argc, argv, "-h"))
    {
        std::cout << "usage: " << argv[0]
                  << " [--ip <192.168.1.201>]"
                  << " [--port <2368>]"
                  << " [--withgps]"
                  << " [--nopcappps]"
                  << " [--usepcapekf]"
                  << " [--exportpcapgps]"
                  << " [--file <*.pcap>]"
                  << " [--offset <0> ]"
                  << " [--stime <1571071222>]"
                  << " [--dir <pcd_dir>]"
                  << " [--csvfile <*.csv>]"
                  << " [--mcsvfile <*.csv>]"
                  << " [--pcapcsvfile <*.csv>]"
                  << " [--filter]"
                  << " [--wftype] <pcd> | <las>"
                  << " [--imucsv <*.csv>]"
#ifdef WITH_SLAM
                  << " [--slamtype] <none> | <icp> | <loam>"
#endif
                  << " [--cloudStep <1>]"
                  << " [--help]"
                  << std::endl;
        return 0;
    }

    std::string ipaddress("192.168.1.201");
    std::string port("2368");
    std::string pcap;
    std::string csv;
    std::string mobileCSV;
    std::string imuCSV;
    std::string pcapCSV;
    std::string pcd_dir;
    std::string writeFileType = "las";
    int start_time = 0;
    int cloud_offset = 0;

    pcl::console::parse_argument(argc, argv, "--ip", ipaddress);
    pcl::console::parse_argument(argc, argv, "--port", port);
    pcl::console::parse_argument(argc, argv, "--file", pcap);
    pcl::console::parse_argument(argc, argv, "--csvfile", csv);
    pcl::console::parse_argument(argc, argv, "--mcsvfile", mobileCSV);
    pcl::console::parse_argument(argc, argv, "--pcapcsvfile", pcapCSV);
    pcl::console::parse_argument(argc, argv, "--dir", pcd_dir);
    pcl::console::parse_argument(argc, argv, "--wftype", writeFileType);
    pcl::console::parse_argument(argc, argv, "--stime", start_time);
    pcl::console::parse_argument(argc, argv, "--offset", cloud_offset);
    pcl::console::parse_argument(argc, argv, "--imucsv", imuCSV);

#ifdef WITH_SLAM
    std::string slamType = "none";
    int processEveryXCloud = -1;
    pcl::console::parse_argument(argc, argv, "--slamtype", slamType);
    pcl::console::parse_argument(argc, argv, "--cloudStep", processEveryXCloud);
#endif

    bool filter = pcl::console::find_switch(argc, argv, "--filter");
    bool withGPS = pcl::console::find_switch(argc, argv, "--withgps");
    bool exportPcapGps = pcl::console::find_switch(argc, argv, "--exportpcapgps");
    bool noPcapPps = pcl::console::find_switch(argc, argv, "--nopcappps");
    bool usePcapEkf = pcl::console::find_switch(argc, argv, "--usepcapekf");

    // Color handler
    std::shared_ptr<pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>> color_handler;
    color_handler = std::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>>(
        "intensity");

    // 3D viewer
    olp::Viewer<pcl::PointXYZI> viewer(*color_handler);

    std::shared_ptr<pcl::Grabber> grabber;

    if (!pcd_dir.empty()) {
        std::vector<std::string> filenames = helper::getFilenames(pcd_dir);

        std::cout << "Capture from PCD files: " << pcap << std::endl;
        grabber = std::make_shared<pcl::PCDGrabber<pcl::PointXYZI>>(filenames, 10);

    } else {
        if (!pcap.empty())
        {
            std::cout << "Capture from PCAP file: " << pcap << std::endl;
            grabber = std::make_shared<grabber::GPSVLPGrabber>(pcap, exportPcapGps, !noPcapPps);
        }
        else if (!ipaddress.empty() && !port.empty())
        {
            std::cout << "Capture from sensor " << ipaddress << ":" << port << std::endl;
            grabber = std::make_shared<grabber::GPSVLPGrabber>(boost::asio::ip::address::from_string(ipaddress),
                                                               boost::lexical_cast<unsigned short>(port),
                                                               !noPcapPps);
        }
        else
        {
            std::cerr << "No source given" << std::endl;
            return 1;
        }
    }

    // Point cloud processor
    compute::GrabberProducer<pcl::PointXYZI> grabberProducer(*grabber);
    pipe::ProcessorPipe<pcl::PointXYZI> processorPipe;

    if (filter)
    {
        compute::Processor<pcl::PointXYZI>* filter = new compute::OrigoFilter<pcl::PointXYZI>();
        processorPipe.add(filter);
        //compute::Processor<pcl::PointXYZI>* axisfilter = new compute::AxisFilter<pcl::PointXYZI>();
        //processorPipe.add(axisfilter);

    }

    std::vector<compute::Calculator<pcl::PointXYZI>*> calculators;
    std::shared_ptr<ViewerShareData<pcl::PointXYZI>> shareData = std::make_shared<ViewerShareData<pcl::PointXYZI>>();

    uint64_t time = !pcap.empty() ? start_time : std::time(0);
    if (!csv.empty()) {
        calculators.push_back(createGPSCalculator(csv, time, gps::GPSSource::stonex, "kalman_filter_test.csv"));
        shareData->precisionMap.emplace(std::make_pair(calculators.back()->stringId(), 0.0));
    }

    if (!mobileCSV.empty()) {
        calculators.push_back(createGPSCalculator(mobileCSV, time, gps::GPSSource::mobile, "kalman_filter_test_mobile.csv"));
        shareData->precisionMap.emplace(std::make_pair(calculators.back()->stringId(), 0.0));
    }

    if (!pcapCSV.empty()) {
        calculators.push_back(createGPSCalculator(pcapCSV, time, gps::GPSSource::pcap, "kalman_filter_test_pcap.csv"));
        shareData->precisionMap.emplace(std::make_pair(calculators.back()->stringId(), 0.0));
    }

    TransformData startData;
    if (!pcap.empty() && calculators.size() != 0)
        startData = calculators[0]->startData;

#ifdef WITH_SLAM
    if (slamType == "icp") {
        calculators.push_back(new compute::ICPSLAMCalculator<pcl::PointXYZI>(startData, processEveryXCloud < 1 ? 1 : processEveryXCloud));
        shareData->precisionMap.emplace(std::make_pair(calculators.back()->stringId(), 0.0));
    }
    else if(slamType == "loam") {
        calculators.push_back(new compute::LOAMSLAMCalculator<pcl::PointXYZI>(startData));
        shareData->precisionMap.emplace(std::make_pair(calculators.back()->stringId(), 0.0));
    }
#endif

    if (withGPS) {
        compute::GPSPacketCalculator<pcl::PointXYZI>* calculator = new compute::GPSPacketCalculator<pcl::PointXYZI>(startData, time, 500000, usePcapEkf);
        calculators.push_back(calculator);
        shareData->precisionMap.emplace(std::make_pair(calculators.back()->stringId(), 0.0));

        compute::GPSAligner<pcl::PointXYZI>* gpsAligner = new compute::GPSAligner<pcl::PointXYZI> ();
        processorPipe.add(gpsAligner);
        grabberProducer.registerGPSPacketHandler(
        [calculator, gpsAligner] (const olp::grabber::GPSVLPGrabber::GPSPacketConstPtr& packet) -> void
        {
            gpsAligner->UpdateGPSTimestamp(packet->timestamp);
            calculator->addPacket(packet);
        });
    }
    if (!imuCSV.empty()) {
        processorPipe.add(createIMUCalculator(imuCSV));
        //shareData->precisionMap.emplace(std::make_pair(calculators.back()->stringId(), 0.0));
    }
    if (calculators.size() > 0) {
        compute::CloudTransformer<pcl::PointXYZI>* cloudTransformer = new compute::CloudTransformer<pcl::PointXYZI>(
            calculators, calculators[0]->startData, shareData);
        processorPipe.add(cloudTransformer);
    }
    compute::MergeTransformer<pcl::PointXYZI>* mergeProcessor = new compute::MergeTransformer<pcl::PointXYZI>();
    processorPipe.add(mergeProcessor);

    compute::ViewConsumer<pcl::PointXYZI> consumer(viewer);

    if (calculators.size() > 0){
        consumer.addShareData(shareData);
    }

    int packetIdx = 0;
    grabberProducer.registerHandler(
        [&](const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
        {
            if(packetIdx++ < cloud_offset)
                return;

            auto result = processorPipe.execute(cloud);
            consumer.show(result);

            if (packetIdx % 100 == 0)
            {
                compute::FileWriter<pcl::PointXYZI> fileWriter(pcap, writeFileType, time);
                fileWriter.show(result);
            }

        }
    );
    grabberProducer.start();
    viewer.run();
    grabberProducer.stop();

    compute::FileWriter<pcl::PointXYZI> fileWriter(pcap, writeFileType, time);
    fileWriter.show(mergeProcessor->mergeCloud);

    return 0;
}
