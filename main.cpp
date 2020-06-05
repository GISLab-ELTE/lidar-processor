#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "processor.hpp"
#include "viewer.hpp"

int main( int argc, char *argv[] )
{
    // Command-Line argument parsing
    if (pcl::console::find_switch(argc, argv, "--help") ||
        pcl::console::find_switch(argc, argv, "-h"))
    {
        std::cout << "usage: " << argv[0]
                  << " [--ip <192.168.1.201>]"
                  << " [--port <2368>]"
                  << " [--file <*.pcap>]"
                  << " [--help]"
                  << std::endl;
        return 0;
    }

    std::string ipaddress("192.168.1.201");
    std::string port("2368");
    std::string pcap;

    pcl::console::parse_argument(argc, argv, "--ip", ipaddress);
    pcl::console::parse_argument(argc, argv, "--port", port);
    pcl::console::parse_argument(argc, argv, "--file", pcap);

    std::shared_ptr<pcl::VLPGrabber> grabber;
    if (!pcap.empty())
    {
        std::cout << "Capture from PCAP file: " << pcap << std::endl;
        grabber = std::make_shared<pcl::VLPGrabber>(pcap);
    }
    else if (!ipaddress.empty() && !port.empty())
    {
        std::cout << "Capture from sensor " << ipaddress << ":" << port << std::endl;
        grabber = std::make_shared<pcl::VLPGrabber>(boost::asio::ip::address::from_string(ipaddress),
                                                    boost::lexical_cast<unsigned short>(port));
    }
    else
    {
        std::cerr << "No source given" << std::endl;
        return 1;
    }

    // Color handler
    std::shared_ptr<pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>> color_handler;
    color_handler = std::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>>(
        "intensity");

    // Point cloud processor
    olp::IdentityProcessor<pcl::PointXYZI> processor(*grabber);

    // 3D viewer
    olp::Viewer<pcl::PointXYZI> viewer(*color_handler);

    // Connect processor output to viewer input
    processor.registerHandler(
        [&viewer](typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
        {
            viewer.update(cloud);
        });

    // Start processor & view
    processor.start();
    viewer.run();

    // Stop processor when user closed the view
    processor.stop();

    return 0;
}
