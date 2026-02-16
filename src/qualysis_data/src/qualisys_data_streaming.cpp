#include "../include/qualysis_data/qualisys_data_streaming.h"
#include <iostream>
#include <thread>
#include <chrono>

const vector<string> markersNamesStr = {
"RFH","RBH","LBH","LFH","C7","IJ_chest","RAC_sho","LAC_sho","RHGT_sho", "LHGT_sho","RPSI_back","LPSI_back","RASI",
"LASI","RLE_elb","RME_elb","LLE_elb","LME_elb","RUS_wrist","RRS_wrist","LUS_wrist","LRS_wrist","RTF_hand","RHM2_hand",
"RIF_hand","RHL5_hand","RLF_hand","LTF_hand","LHM2_hand","LIF_hand","LHL5_hand","LLF_hand"};

QualisysDataStreaming::QualisysDataStreaming(){

}

bool QualisysDataStreaming::connectServer(){
  if (!rtProtocol->Connected()){
      if (rtProtocol->Connect(this->serverAddr, basePort, &udpServerPort)) {
          std::string ver;
          if (rtProtocol->GetQTMVersion(ver)) {
              std::cout << "Connected to QTM. Version: " << ver << "\n";
              std::cout << "Using UDP port: " << udpServerPort << "\n";
          }
      } else {
          std::cerr << "Failed to connect: " << rtProtocol->GetErrorString() << "\n";
          return false;
      }
  }
  return true;
}

void QualisysDataStreaming::setServerSettings(char* serverAddr, unsigned short basePort,unsigned short udpServerPort){
  this->serverAddr = serverAddr;
  this->basePort = basePort;
  this->udpServerPort = udpServerPort;
}

bool QualisysDataStreaming::initStreaming(){
  if (!this->streamFrames) {
      if (!rtProtocol->StreamFrames(
              CRTProtocol::EStreamRate::RateAllFrames,
              0, udpServerPort, nullptr,
              CRTPacket::Component3d)) {
          std::cerr << "StreamFrames error: " << rtProtocol->GetErrorString() << "\n";
          std::this_thread::sleep_for(std::chrono::seconds(1));
          return false;
      }
      this->streamFrames = true;
      std::cout << "Streaming 3D markers...\n";
  }
  return true;
}

bool QualisysDataStreaming::readMarkersData(vector<Marker> &markersFrameData){
  CRTPacket::EPacketType packetType;
  auto resp = rtProtocol->Receive(packetType, true);

  if (resp == CNetwork::ResponseType::success &&
      packetType == CRTPacket::PacketData) {

      CRTPacket* rtPacket = rtProtocol->GetRTPacket();
      if (!rtPacket) { // guard extra
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          return false;
      }

      std::cout << "Frame " << rtPacket->GetFrameNumber() << "\n";

      float x, y, z;
      const unsigned int count = rtPacket->Get3DMarkerCount();
      for (unsigned int i = 0; i < count; ++i) {
          rtPacket->Get3DMarker(i, x, y, z);
          if(std::isnan(x) || std::isnan(y) || std::isnan(z)){
              x = 0.0;
              y = 0.0;
              z = 0.0;
          }
          x*=0.001;
          y*=0.001;
          z*=0.001;
          markersFrameData.push_back({markersNamesStr.at(i),{x,y,z}});
          std::cout << "  Marker " << i << " ( " << markersNamesStr.at(i) << " )"
                    << " | X: " << x << "  Y: " << y << "  Z: " << z << "\n";
      }
      std::cout << std::endl;
      return true;
  } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      return false;
  }
}

void QualisysDataStreaming::disconnectServer(){
  rtProtocol->StopCapture();
  rtProtocol->Disconnect();
}
