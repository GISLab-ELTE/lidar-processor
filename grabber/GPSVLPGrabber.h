/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_GPSVLPGRABBER_H
#define OLP_GPSVLPGRABBER_H

#include <cstdint>
#include <memory>
#include <thread>

#include <boost/asio.hpp>

#include <pcl/io/vlp_grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>

#include "../helpers/CsvFileWriter.h"

#include "GPSPacket.h"


namespace olp
{
namespace grabber
{


/**
 * Grabber to process gps packets too from the incoming (pcap or socket) datastream along with the VLPGrabber functionality.
 * Packets are externally available via the registerCallback method of the grabber with a SigGPSPacketReadyT function type.
 */
class GPSVLPGrabber : public pcl::VLPGrabber
{
public:
    using GPSPacketConstPtr = std::shared_ptr<const GPSPacket>;
    using SigGPSPacketReadyT = void (const GPSPacketConstPtr&);

    GPSVLPGrabber (const std::string& pcapFile,
                   bool logGpsPackets = false,
                   bool requirePPS = true);

    GPSVLPGrabber (const boost::asio::ip::address& ipAddress,
                   const std::uint16_t port,
                   bool requirePPS = true);
    
    virtual ~GPSVLPGrabber () noexcept override;

    virtual void start () override;

    virtual void stop () override;

    virtual std::string getName () const override;

private:
    static const std::uint16_t GPS_DATA_PORT = 8308; // maybe param?
    static const std::size_t GPS_PACKET_LENGTH = 512;

    using GPSData = std::array<std::uint8_t, GPS_PACKET_LENGTH>;

    bool terminateReadThread;
    pcl::SynchronizedQueue<GPSData> gpsDataQueue;
    boost::signals2::signal<SigGPSPacketReadyT>* sigGPSPacketReady;

    std::string pcapFile;
    std::unique_ptr<std::thread> readGPSPacketThread;
    std::unique_ptr<std::thread> processGPSPacketThread;
    std::unique_ptr<helper::csvfile> gpsPacketOutputFile;
    std::uint32_t firstWrittenTimestamp;

    boost::asio::ip::udp::endpoint listenGPSPacketEndpoint;
    boost::asio::io_service listenGPSPacketSocketService;
    std::unique_ptr<boost::asio::ip::udp::socket> listenGPSPacketSocket;

    bool requirePPS;
    GPSPacket lastGpsPacket;
    
    void Initialize ();
    void ProcessGPSPacket ();
    void EnqueueGPSData (const std::uint8_t* buffer, std::size_t length);
    std::string GetNmeaSentenceFromGPSData (GPSData data);

    void ReadGPSPacketsFromPcap ();
    void ReadGPSPacketsFromSocket ();
};


} // grabber
} // olp

#endif // OLP_GPSVLPGRABBER_H
