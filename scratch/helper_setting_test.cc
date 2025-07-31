#include "ns3/antenna-model.h"
#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/attribute.h" // GetAttribute 위해
#include "ns3/cc-bwp-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/epc-tft.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ideal-beamforming-algorithm.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/node-container.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/parabolic-antenna-model.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/pointer.h"
#include "ns3/spectrum-module.h" // SpectrumPhy 선언
#include "ns3/uniform-planar-array.h"
using namespace ns3;

int
main(void)
{
    NodeContainer ueNode;
    NodeContainer rsuNode;

    ueNode.Create(1);
    rsuNode.Create(1);

    Ptr<ListPositionAllocator> rsuPositionAlloc =
        CreateObject<ListPositionAllocator>(); // gnb위치 저장
    Ptr<ListPositionAllocator> uePositionAlloc =
        CreateObject<ListPositionAllocator>(); // ue위치 저장

    rsuPositionAlloc->Add(Vector(0.0, 0.0, 0.0));
    uePositionAlloc->Add(Vector(0.0, 0.0, 0.0));

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    mobility.Install(rsuNode);
    mobility.Install(ueNode);

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetEpcHelper(nrEpcHelper);

    // Antennas for all the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    // 2*4 즉 8개 안테나
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(10)); // dBm그대로 넣는듯

    CcBwpCreator gNbCcBwpCreators;
    OperationBandInfo gNbBand;
    double gNbFrequencyBand = 3.5e9; // 3.5GHz
    double gNbBandwidthBand = 1e8;   // 100MHz
    uint8_t gNbnumContiguousCc = 1;  // 100MHz 안에 몇개의 CC가 들어가 있는지
    CcBwpCreator::SimpleOperationBandConf gNbBandConf(
        gNbFrequencyBand,
        gNbBandwidthBand,
        gNbnumContiguousCc,
        BandwidthPartInfo::UMi_StreetCanyon_LoS); // 고속도로 시나리오 설정
    gNbBandConf.m_numBwp = 1;                     // 1 BWP per CC
    gNbBand = gNbCcBwpCreators.CreateOperationBandContiguousCc(gNbBandConf);

    nrHelper->InitializeOperationBand(&gNbBand);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({gNbBand});

    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNode, allBwps);

    // Antennas for all the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(16));
    // 2*4 즉 8개 안테나
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<ParabolicAntennaModel>()));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(100)); // dBm그대로 넣는듯
    CcBwpCreator rsuCcBwpCreators;
    OperationBandInfo rsuBand;
    double rsuFrequencyBand = 3.9e9; // 3.5GHz
    double rsuBandwidthBand = 1e8;   // 100MHz
    uint8_t rsunumContiguousCc = 1;  // 100MHz 안에 몇개의 CC가 들어가 있는지
    CcBwpCreator::SimpleOperationBandConf rsuBandConf(
        rsuFrequencyBand,
        rsuBandwidthBand,
        rsunumContiguousCc,
        BandwidthPartInfo::UMi_StreetCanyon_LoS); // 고속도로 시나리오 설정
    rsuBandConf.m_numBwp = 1;                     // 1 BWP per CC
    rsuBand = rsuCcBwpCreators.CreateOperationBandContiguousCc(rsuBandConf);

    nrHelper->InitializeOperationBand(&rsuBand);
    BandwidthPartInfoPtrVector allBwpsss = CcBwpCreator::GetAllBwps({rsuBand});

    NetDeviceContainer rsuNetDev = nrHelper->InstallUeDevice(rsuNode, allBwpsss);
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = ueNetDev.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<NrUePhy> uePhy = ueDev->GetPhy(0); // 첫 번째 BWP
        double freq = uePhy->GetCentralFrequency();
        std::cout << "UE[" << i << "] 실제 적용된 Center Frequency: " << freq << " Hz" << std::endl;
    }

    // RSU NetDevice에서 실제 적용된 BWP의 중심 주파수 확인
    for (uint32_t i = 0; i < rsuNetDev.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> rsuDev = rsuNetDev.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<NrUePhy> rsuPhy = rsuDev->GetPhy(0); // 첫 번째 BWP
        double freq = rsuPhy->GetCentralFrequency();
        std::cout << "RSU[" << i << "] 실제 적용된 Center Frequency: " << freq << " Hz"
                  << std::endl;
    }
    // ...existing code...

    // UE NetDevice에서 실제 적용된 Tx Power 확인
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = ueNetDev.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<NrUePhy> uePhy = ueDev->GetPhy(0); // 첫 번째 BWP
        double txPower = uePhy->GetTxPower();
        std::cout << "UE[" << i << "] 실제 적용된 Tx Power: " << txPower << " dBm" << std::endl;
    }

    // RSU NetDevice에서 실제 적용된 Tx Power 확인
    for (uint32_t i = 0; i < rsuNetDev.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> rsuDev = rsuNetDev.Get(i)->GetObject<NrUeNetDevice>();
        Ptr<NrUePhy> rsuPhy = rsuDev->GetPhy(0); // 첫 번째 BWP
        double txPower = rsuPhy->GetTxPower();
        std::cout << "RSU[" << i << "] 실제 적용된 Tx Power: " << txPower << " dBm" << std::endl;
    }

    // ...existing code...
}