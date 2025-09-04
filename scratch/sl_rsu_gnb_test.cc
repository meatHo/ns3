#include "ns3/address.h"
#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/csma-helper.h"
#include "ns3/epc-tft.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/hybrid-buildings-propagation-loss-model.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-module.h"
#include "ns3/nr-sl-ue-cphy-sap.h"
#include "ns3/oh-buildings-propagation-loss-model.h"
#include "ns3/point-to-point-module.h"
#include <ns3/pointer.h>
using namespace ns3;


void
UeSlMeasCallback(uint16_t RNTI, uint32_t L2ID, double RSRP)
{
    std::cout << "📶 [Meas] RNTI=" << RNTI << " L2ID=" << L2ID << " RSRP=" << RSRP << " dB\n";
}

#include <ns3/spectrum-model.h>
#include <ns3/spectrum-value.h>

#include <cmath>

#include "ns3/spectrum-phy.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/net-device.h"
#include "ns3/node.h"



// 패킷 정보를 출력할 콜백 함수
void
Ipv6PacketTraceAtRsu(Ptr<const Packet> packet, Ptr<Ipv6> ipv6, uint32_t interfaceIndex)
{
    Ipv6Header ipv6Header;
    packet->PeekHeader(ipv6Header);

    std::cout << "[RSU Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}

void
Ipv6PacketTraceAtPgw(Ptr<const Packet> packet, Ptr<Ipv6> ipv6, uint32_t interfaceIndex)
{
    Ipv6Header ipv6Header;
    packet->PeekHeader(ipv6Header);

    std::cout << "[PGW Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}


int
main(void)
{
    Time simTime = Seconds(10);
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    NodeContainer rsuNodeContainer;
    rsuNodeContainer.Create(1);
    NodeContainer ueNodeContainer;
    ueNodeContainer.Create(1);

    Ptr<Node> rsuNode = rsuNodeContainer.Get(0);
    Ptr<Node> ueNode = ueNodeContainer.Get(0);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(rsuNodeContainer);
    mobility.Install(ueNodeContainer);
    rsuNode->GetObject<MobilityModel>()->SetPosition(Vector(1,1,1));
    ueNode->GetObject<MobilityModel>()->SetPosition(Vector(2,2,2));

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    double rsuFrequencyBand = 5.9e9;
    double rsuBandwidth = 1e8;
    uint8_t rsuNumContiguousCc = 1;
    uint16_t rsuNumerology = 1;
    double RsuTxPower = 23.0; // 단위dBm

    CcBwpCreator RsuCcBwpCreator;
    CcBwpCreator::SimpleOperationBandConf rsuBandConf(rsuFrequencyBand,
                                                      rsuBandwidth,
                                                      rsuNumContiguousCc,
                                                      BandwidthPartInfo::V2V_Highway);
    OperationBandInfo rsuBand = RsuCcBwpCreator.CreateOperationBandContiguousCc(rsuBandConf);

    nrHelper->InitializeOperationBand(&rsuBand);
    BandwidthPartInfoPtrVector rsuBwp = CcBwpCreator::GetAllBwps({rsuBand});
}



