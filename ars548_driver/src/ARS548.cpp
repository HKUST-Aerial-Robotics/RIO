#include "ARS548.hpp"

namespace Driver
{
ARS548::ARS548(std::string device,
               std::string multicastIP,
               int multicastPort,
               std::string unicastIP,
               int unicastSrcPort,
               int unicastDstPort)
{
    init(device,
         multicastIP,
         multicastPort,
         unicastIP,
         unicastSrcPort,
         unicastDstPort);
}

void ARS548::init(std::string device,
                  std::string multicastIP,
                  int multicastPort,
                  std::string unicastIP,
                  int unicastSrcPort,
                  int unicastDstPort)
{
    if (!io.initDevice(device))
        std::cout << "device init error" << std::endl;
    if (!io.initUdpMulticastServer(multicastIP, multicastPort))
        std::cout << "multicast server init error" << std::endl;
    if (!io.initUdpUnicastClient(unicastIP, unicastDstPort, unicastSrcPort))
        std::cout << "unicast client init error" << std::endl;
}

void ARS548::receiveMsg()
{
    long length = io.receiveFromRadar(data);
    // std::cout << "length:" << length << std::endl;
    switch (length)
    {
    case 9401:
    {
        // std::cout << "receive object list" << std::endl;
        objectList = coder.decodeObjectListMessage(data);
        objectListReady = true;
    }
    break;
    case 35336:
    {
        // std::cout << "receive detection list" << std::endl;
        detectionList = coder.decodeDetectionListMessage(data);
        detectionListReady = true;
    }
    break;
    case 69:
    {
        // std::cout << "receive radar status" << std::endl;
        radarStatus = coder.decodeBasicStatusMessage(data);
        radarStatusReady = true;
    }
    break;
    default:
        std::cout << "unknown message" << std::endl;
        break;
    }
}
void ARS548::sendMsg()
{
    // int n;
    // n = sendto(socket_client_fd,
    //            data,
    //            len,
    //            0,
    //            (struct sockaddr *)&addr_serv,
    //            sizeof(addr_serv));
    // return (n);
}

bool ARS548::getDetectionListReady() const { return detectionListReady; }

bool ARS548::getObjectListReady() const { return objectListReady; }

bool ARS548::getRadarStatusReady() const { return radarStatusReady; }

void ARS548::clearReadyFlag()
{
    detectionListReady = false;
    objectListReady = false;
    radarStatusReady = false;
}

ARS548RadarDetectionList &ARS548::getDetectionList() { return detectionList; }

ARS548RadarObjectList &ARS548::getObjectList() { return objectList; }

ARS548RadarStatus &ARS548::getRadarStatus() { return radarStatus; }

ARS548::~ARS548() { io.closeIO(); }

};  // namespace Driver