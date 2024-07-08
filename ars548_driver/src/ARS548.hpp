#pragma once
#include <pcap/pcap.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "ARS548Coder.hpp"

namespace Driver
{
class ARS548
{
   public:
    ARS548() = default;
    ARS548(std::string device,
           std::string multicastIP,
           int multicastPort,
           std::string unicastIP,
           int unicastSrcPort,
           int unicastDstPort);
    ~ARS548();
    ARS548(ARS548 &another) = delete;
    ARS548(ARS548 &&another) = delete;
    ARS548 &operator=(const ARS548 &another) = delete;
    ARS548 &operator=(ARS548 &&another) = delete;

    void init(std::string device,
              std::string multicastIP,
              int multicastPort,
              std::string unicastIP,
              int unicastSrcPort,
              int unicastDstPort);

    void receiveMsg();
    void sendMsg();

    bool getDetectionListReady() const;
    bool getObjectListReady() const;
    bool getRadarStatusReady() const;
    void clearReadyFlag();

    ARS548RadarDetectionList &getDetectionList();
    ARS548RadarObjectList &getObjectList();
    ARS548RadarStatus &getRadarStatus();

   private:
    class UdpIO
    {
       public:
        UdpIO() = default;
        ~UdpIO() = default;
        UdpIO(UdpIO &another) = delete;
        UdpIO(UdpIO &&another) = delete;
        UdpIO &operator=(const UdpIO &another) = delete;
        UdpIO &operator=(UdpIO &&another) = delete;

        bool initDevice(std::string device);
        bool initUdpMulticastServer(std::string ipAddress, int port);
        bool initUdpUnicastClient(std::string dstIpAddress,
                                  int dstPort,
                                  int localPort);
        static void listAllDevice();
        long receiveFromRadar(uint8_t *data);
        long sendToRadar(uint8_t *data, int len);
        bool closeIO() const;
        static bool decodeIPHeader(const uint8_t *data,
                            bool &noFragment,
                            bool &moreFragment,
                            int &offsetFragment,
                            int &frameID,
                            int &dataOffset,
                            int &dataLength);

       private:
        pcap_t *deviceHandler = nullptr;
    } io{};

    uint8_t data[40000]{};
    ARS548Coder coder = {};
    bool detectionListReady = false;
    bool objectListReady = false;
    bool radarStatusReady = false;
    ARS548RadarDetectionList detectionList = {};
    ARS548RadarObjectList objectList = {};
    ARS548RadarStatus radarStatus = {};
};
}  // namespace Driver