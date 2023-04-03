/*
 * BSD 3-Clause License
 * Copyright (c) 2021, Péter Farkas
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>

#include <pcap/pcap.h>

#include "../helpers/Utility.h"

#include "GPSVLPGrabber.h"
#include "NmeaParser.h"

using namespace olp::grabber;
using boost::asio::ip::udp;


GPSVLPGrabber::GPSVLPGrabber (const std::string& pcapFile,
                              bool logGpsPackets/* = false */,
                              bool requirePPS/* = true */)
    : pcl::VLPGrabber               (pcapFile)
    , pcapFile                      (pcapFile)
    , sigGPSPacketReady             (nullptr)
    , firstWrittenTimestamp         (0)
    , requirePPS(requirePPS)
{
    if (logGpsPackets) {
        std::size_t startPos = pcapFile.find_last_of ("/");
        startPos = startPos == std::string::npos ? 0 : startPos + 1;
        std::string fileName = pcapFile.substr (startPos, pcapFile.find_last_of (".") - startPos) + ".csv";
        
        gpsPacketOutputFile = std::make_unique<helper::csvfile> (fileName);
        *gpsPacketOutputFile << "latitude" << "longitude" << "elevation" << "accuracy" << "timestamp" << helper::endrow;
    }
    Initialize ();
}


GPSVLPGrabber::GPSVLPGrabber (const boost::asio::ip::address& ipAddress,
                              const std::uint16_t port,
                              bool requirePPS /* = true */)
    : pcl::VLPGrabber               (ipAddress, port)
    , sigGPSPacketReady             (nullptr)
    , listenGPSPacketEndpoint       (ipAddress, port)
    , requirePPS(requirePPS)
{
    Initialize ();
}


GPSVLPGrabber::~GPSVLPGrabber () noexcept
{
    stop ();

    disconnect_all_slots<SigGPSPacketReadyT> ();

    if (firstWrittenTimestamp != 0)
        std::cout << "Starting exported timestamp: " << firstWrittenTimestamp << std::endl;
}


void GPSVLPGrabber::Initialize ()
{
    sigGPSPacketReady = createSignal<SigGPSPacketReadyT> ();
    lastGpsPacket.timestamp = -1; // won't match first packet
}


void GPSVLPGrabber::start ()
{
    terminateReadThread = false;
    if (pcapFile.empty ()) {
        try {
            try {
                if (listenGPSPacketEndpoint.address ().is_unspecified ())
                    listenGPSPacketEndpoint.address (boost::asio::ip::address::from_string ("192.168.3.255"));

                if (listenGPSPacketEndpoint.port () == 0)
                    listenGPSPacketEndpoint.port (GPS_DATA_PORT);

                listenGPSPacketSocket = std::make_unique<udp::socket> (listenGPSPacketSocketService, listenGPSPacketEndpoint);
            } catch (const std::exception&)
            {
                listenGPSPacketSocket = nullptr;
                listenGPSPacketSocket = std::make_unique<udp::socket> (listenGPSPacketSocketService,
                                                                       udp::endpoint (boost::asio::ip::address_v4::any (), listenGPSPacketEndpoint.port ()));
            }
            listenGPSPacketSocketService.run ();
            readGPSPacketThread = std::make_unique<std::thread> (&GPSVLPGrabber::ReadGPSPacketsFromSocket, this);
        } catch (std::exception &e) {
            std::cerr << ("[olp::grabber::GPSVLPGrabber::start] Unable to bind to socket! %s\n", e.what ());
        }

    } else
        readGPSPacketThread = std::make_unique<std::thread> (&GPSVLPGrabber::ReadGPSPacketsFromPcap, this);
    
    processGPSPacketThread = std::make_unique<std::thread> (&GPSVLPGrabber::ProcessGPSPacket, this);
    pcl::VLPGrabber::start ();
}


void GPSVLPGrabber::stop ()
{
    pcl::VLPGrabber::stop ();
    terminateReadThread = true;
    gpsDataQueue.stopQueue ();

    if (readGPSPacketThread != nullptr)
    {
        readGPSPacketThread->join ();
        readGPSPacketThread.reset ();
    }

    if (processGPSPacketThread != nullptr)
    {
        processGPSPacketThread->join ();
        processGPSPacketThread.reset ();
    }

    listenGPSPacketSocket.reset ();
}


std::string GPSVLPGrabber::getName () const
{
    return "Velodyne LiDAR (VLP) Grabber with GPS packets";
}


/* Packet structure:
 * 198 byte unused
 * 4 byte timestamp (microseconds)
 * 1 byte Pulse Per Second status
 * 3 byte unused
 * var byte NMEA sentence
 * var byte padding to 512 total bytes
 */
void GPSVLPGrabber::ProcessGPSPacket ()
{
    GPSData data;
    GPSPacket packet;
    while (gpsDataQueue.dequeue (data)) {
        if (requirePPS && data[202] != 2) // skip packets with non-synced status
            continue;

        std::uint32_t packetTimestamp = *reinterpret_cast<std::uint32_t*> (&data[198]);
        std::string sentence = GetNmeaSentenceFromGPSData (data);

        // the exact same timestamp generation as in the pcl source for the point cloud data
        time_t system_time;
        time (&system_time);
        packet.timestamp = (system_time & 0x00000000ffffffffl) << 32 | packetTimestamp;

        if (NmeaParser::TryParseSentence (sentence, packet)) {
            if (lastGpsPacket == packet)
                continue;

            lastGpsPacket = packet;
            if (gpsPacketOutputFile != nullptr) {
                static std::uint32_t prevTransformedTimestamp;
                std::uint32_t transformedTimestamp = helper::calculatePointCloudTimeStamp (packet.timestamp);
                if (prevTransformedTimestamp != transformedTimestamp) {
                    prevTransformedTimestamp = transformedTimestamp;
                    *gpsPacketOutputFile << packet.latitude
                                        << packet.longitude
                                        << packet.elevation
                                        << packet.accuracy
                                        << transformedTimestamp
                                        << helper::endrow;
                }
                if (firstWrittenTimestamp == 0)
                    firstWrittenTimestamp = prevTransformedTimestamp;
            }
            sigGPSPacketReady->operator() (std::make_shared<GPSPacket> (packet));
        }
    }
}


void GPSVLPGrabber::EnqueueGPSData (const std::uint8_t* buffer, std::size_t length)
{
    if (length != GPS_PACKET_LENGTH)
        return;

    GPSData data;
    std::copy (buffer, buffer + GPS_PACKET_LENGTH, data.begin ());
    gpsDataQueue.enqueue (data);
}


std::string GPSVLPGrabber::GetNmeaSentenceFromGPSData (GPSData data)
{
    const static std::array<std::uint8_t, 2> endSequence = { 0x0d, 0x0a };  // CR LF
    std::uint8_t* msgBegin = data.begin () + 206 + 3;                       // 206 byte offset until the NMEA sentence, additional 3 bytes to skip the leading $GP
    std::uint8_t* msgEnd = std::search (msgBegin, data.end (), endSequence.begin (), endSequence.end ());
    return std::string { msgBegin, msgEnd };
}


void GPSVLPGrabber::ReadGPSPacketsFromPcap ()
{
    pcap_pkthdr *header;
    const std::uint8_t *data;
    std::int8_t errbuff[1024];

    pcap_t *pcap = pcap_open_offline (pcapFile.c_str (), reinterpret_cast<char *> (errbuff));

    bpf_program filter;
    std::ostringstream ss;
    ss << "udp and src port " << GPS_DATA_PORT;

    // PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
    if (pcap_compile (pcap, &filter, ss.str ().c_str (), 0, 0xffffffff) == -1)
    {
        std::cerr << ("[olp::grabber::GPSVLPGrabber::ReadGPSPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr (pcap));
    }
    else if (pcap_setfilter(pcap, &filter) == -1)
    {
        std::cerr << ("[olp::grabber::GPSVLPGrabber::ReadGPSPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr (pcap));
    }

    timeval lasttime;

    lasttime.tv_sec = lasttime.tv_usec = 0;

    std::int32_t returnValue = pcap_next_ex (pcap, &header, &data);

    while (returnValue >= 0 && !terminateReadThread)
    {
        if (lasttime.tv_sec == 0) {
            lasttime.tv_sec = header->ts.tv_sec;
            lasttime.tv_usec = header->ts.tv_usec;
        }
        if (lasttime.tv_usec > header->ts.tv_usec) {
            lasttime.tv_usec -= 1000000;
            lasttime.tv_sec++;
        }
        std::uint64_t usec_delay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
                                    (header->ts.tv_usec - lasttime.tv_usec);

        std::this_thread::sleep_for(std::chrono::microseconds(usec_delay));

        lasttime.tv_sec = header->ts.tv_sec;
        lasttime.tv_usec = header->ts.tv_usec;

        // skip UDP header 42 bytes
        EnqueueGPSData (data + 42, header->len - 42);

        returnValue = pcap_next_ex (pcap, &header, &data);
    }
}


void GPSVLPGrabber::ReadGPSPacketsFromSocket ()
{
    std::uint8_t buffer[1024];
    udp::endpoint senderEndpoint;

    while (!terminateReadThread && listenGPSPacketSocket->is_open ()) {
        std::size_t length = listenGPSPacketSocket->receive_from (boost::asio::buffer (buffer, 1024), senderEndpoint);

        EnqueueGPSData (buffer, length);
    }
}
