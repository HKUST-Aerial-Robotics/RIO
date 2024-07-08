#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <netinet/in.h>
#include <pcap/pcap.h>

#include <cstdint>
#include <cstring>

#include "ARS548.hpp"
#include "pcap.h"

namespace Driver
{

bool ARS548::UdpIO::initDevice(std::string device)
{
    char buffer[1024];
    deviceHandler = pcap_open_live(device.c_str(), 65535, 1, 20, buffer);
    if (deviceHandler == nullptr)
    {
        std::cout << "open device " << device << " error!" << std::endl
                  << "available device list:" << std::endl;
        listAllDevice();
        return false;
    }
    return true;
}

bool ARS548::UdpIO::initUdpMulticastServer(std::string ipAddress, int port)
{
    bpf_program filter{};
    if (pcap_compile(deviceHandler, &filter, "dst host 224.0.2.2", 1, 0) < 0)
    {
        std::cout << "compile filter rule error" << std::endl;
        return false;
    }
    if (pcap_setfilter(deviceHandler, &filter) < 0)
    {
        std::cout << "set filter error" << std::endl;
        return false;
    }
    return true;
}

bool ARS548::UdpIO::initUdpUnicastClient(std::string dstIpAddress,
                                         int dstPort,
                                         int localPort)
{
    // sending message to radar, no support yet.
    // TODO
    return true;
}

void ARS548::UdpIO::listAllDevice()
{
    pcap_if_t *allDev = nullptr;
    char buffer[1024];
    pcap_findalldevs(&allDev, buffer);
    for (auto *ptr = allDev; ptr != nullptr; ptr = ptr->next)
        std::cout << ptr->name << std::endl;
    pcap_freealldevs(allDev);
}

long ARS548::UdpIO::receiveFromRadar(uint8_t *data)
{
    pcap_pkthdr packetInfo{};
    const uint8_t *packetContent = nullptr;
    packetContent = pcap_next(deviceHandler, &packetInfo);

    bool noFragment = false;
    bool moreFragment = false;
    int offsetFragment = 0;
    int frameID = 0;
    int dataOffset = 0;
    int dataLength = 0;
    bool res = decodeIPHeader(packetContent,
                              noFragment,
                              moreFragment,
                              offsetFragment,
                              frameID,
                              dataOffset,
                              dataLength);
    if (!res)
        return -1;
    // single frame
    if ((!noFragment) && (!moreFragment) && (offsetFragment == 0))
    {
        memcpy(data, packetContent + dataOffset, dataLength);
        return dataLength;
    }
    bool startFrameReceived = false;
    bool endFrameReceived = false;
    long lengthReceived = 0;
    // multi-frame
    memcpy(data + offsetFragment - 8, packetContent + dataOffset, dataLength);
    lengthReceived += dataLength;
    while ((!startFrameReceived) || (!endFrameReceived))
    {
        packetContent = pcap_next(deviceHandler, &packetInfo);
        res = decodeIPHeader(packetContent,
                             noFragment,
                             moreFragment,
                             offsetFragment,
                             frameID,
                             dataOffset,
                             dataLength);
        if (!res)
        {
            std::cout << "error!" << std::endl;
            return 0;
        }
        if (!moreFragment)
            endFrameReceived = true;
        // the start frame contains padding only
        if (offsetFragment == 0)
        {
            startFrameReceived = true;
            break;
        }
        memcpy(data + offsetFragment - 8,
               packetContent + dataOffset,
               packetInfo.caplen - dataOffset);
        lengthReceived += packetInfo.caplen - dataOffset;
    }

    if (lengthReceived == dataLength)
    {
        return lengthReceived;
    }
    return 0;
}

long ARS548::UdpIO::sendToRadar(uint8_t *data, int len)
{
    // sending message to radar, no support yet.
    // TODO
    return -1;
}

bool ARS548::UdpIO::closeIO() const
{
    pcap_close(deviceHandler);
    return true;
}
bool ARS548::UdpIO::decodeIPHeader(const uint8_t *data,
                                   bool &noFragment,
                                   bool &moreFragment,
                                   int &offsetFragment,
                                   int &frameID,
                                   int &dataOffset,
                                   int &dataLength)
{
    uint16_t type = 0;
    int offset = 0;
    const ethhdr *eth = nullptr;
    eth = reinterpret_cast<const ethhdr *>(data);
    type = ntohs(eth->h_proto);
    offset = sizeof(ethhdr);

    while (type == ETH_P_8021Q)
    {
        type = (*(data + offset + 2) << 8) | *(data + offset + 3);
        offset += 4;
    }
    if (type != ETH_P_IP)
        return false;

    const iphdr *ipHeader = reinterpret_cast<const iphdr *>(data + offset);
    uint16_t fragmentFlag = ntohs(ipHeader->frag_off);
    noFragment = (fragmentFlag & 0x4000) != 0;
    moreFragment = (fragmentFlag & 0x2000) != 0;
    offsetFragment = (fragmentFlag & 0x1FFF) << 3;
    frameID = ntohs(ipHeader->id);
    offset += sizeof(iphdr);
    if (ipHeader->protocol != IPPROTO_UDP)
        return false;

    // single frame or multi-frame start frame
    if (offsetFragment == 0)
    {
        dataOffset = offset + 8;
        dataLength = (*(data + offset + 4)) << 8 | (*(data + offset + 5)) - 8;
    }

    // multi-frame data frame
    if ((noFragment) && (offsetFragment != 0))
    {
        dataOffset = offset;
        dataLength = ntohs(ipHeader->tot_len) - 20;
    }
    return true;
}

}  // namespace Driver