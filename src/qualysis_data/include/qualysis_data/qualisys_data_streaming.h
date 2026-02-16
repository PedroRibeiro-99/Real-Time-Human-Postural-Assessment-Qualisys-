#ifndef QUALISYSDATASTREAMING_H
#define QUALISYSDATASTREAMING_H

#include "qualysis_move_data.h"
#include "RTProtocol.h"


class QualisysDataStreaming{
private:
  char* serverAddr = "localhost";
  unsigned short basePort = 22222;
  unsigned short udpServerPort = 0; // Use random port.
  bool dataAvailable = false;
  bool streamFrames  = false;
  bool isConnected = false;
  std::unique_ptr<CRTProtocol>  rtProtocol = std::make_unique<CRTProtocol>();
  //CRTProtocol*  rtProtocol = NULL;

public:
  QualisysDataStreaming();
  void setServerSettings(char* serverAddr, unsigned short basePort,unsigned short udpServerPort);
  bool connectServer();
  bool initStreaming();
  bool readMarkersData(vector<Marker> &markersFrameData);
  void disconnectServer();
};

#endif // QUALISYSDATASTREAMING_H
