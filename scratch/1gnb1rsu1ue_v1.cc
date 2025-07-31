/*
    KOH GIHO
    1gnb1rsu1ue_v1.cc
    GOAL : OBU detect two signals?

    RSU uses Sidelink
    gNb uses Uu
    OBU uses Sl && Uu

    0                     100
    gNb ---------------- RSU
    OBU ->

    코드 진행 순서
    gnb설정 -> rsu설정 -> obu설정 -> 인터넷 설정

    todo : 할것
    dmdkdkd
*/

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/epc-tft.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-sl-ue-cphy-sap.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"
#include <ns3/pointer.h>

using namespace ns3;

void
UeMeasCallback(uint16_t cellId, uint16_t IMSI, uint16_t RNTI, double RSRP, uint8_t BWPId)
{
    std::cout << "📶 [Meas] cellId=" << cellId << " IMSI=" << IMSI << " BWPId=" << BWPId
              << "  RNTI=" << RNTI << " RSRP=" << RSRP << " dB\n";
}

void
LogUeFlowInfo(Ptr<Node> ue)
{
    Vector pos = ue->GetObject<MobilityModel>()->GetPosition();
    Vector velocity = ue->GetObject<MobilityModel>()->GetVelocity();
    std::cout << "🕒 " << Simulator::Now().GetSeconds() << "s | UE 위치: (" << pos.x << ", "
              << pos.y << ", " << pos.z << ")\n";
    std::cout << "UE 위치: (" << velocity.x << ", " << velocity.y << ", " << velocity.z << ")\n";

    // --- UU NetDevice (인덱스 0) ---
    // Ptr<NrUeNetDevice> ueUuNetDev = ue->GetObject<NrUeNetDevice>();
    Ptr<NrUeNetDevice> uuDevice = nullptr;
    Ptr<NrUeNetDevice> slDevice = nullptr;

    // ue 노드에 설치된 모든 NetDevice 수를 가져옵니다.
    for (uint32_t i = 0; i < ue->GetNDevices(); ++i)
    {
        // i번째 NetDevice를 가져와 NrUeNetDevice 타입으로 변환합니다.
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(ue->GetDevice(i));

        if (dev) // 타입 변환이 성공했다면
        {
            // 해당 디바이스에서 Sidelink 컴포넌트(SlPhyMac)를 찾아봅니다.
            if (dev->GetObject<ns3::NrSlUeMac>() != nullptr)
            {
                // 컴포넌트가 존재하면 -> Sidelink 디바이스입니다.
                slDevice = dev;
                std::cout << "sidelink" << std::endl;
            }
            else
            {
                // 컴포넌트가 없으면 -> Uu 디바이스입니다.
                uuDevice = dev;
                std::cout << "uuDevice" << std::endl;
            }
        }
    }

    // 1초 뒤 재호출
    Simulator::Schedule(Seconds(1.0), &LogUeFlowInfo, ue);
}

int
main(void)
{
    Time simTime = Seconds(30);

    // 헬퍼 설정
    // todo : pathloss모델을 Uu와 SL다르게 하고 싶은데 이게 안됨..;;
    // 모델을 어케 설정하는지도 모르겠음
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true)); // 신호 감쇠 현상 ON
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    // gnb<->core network 전송 속도 0ms
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    // gNb, Uu 기본 설정=======================================================
    uint16_t gNbNum = 1;
    double gNbFrequencyBand = 3.5e9; // 3.5GHz
    double gNbBandwidthBand = 1e8;   // 100MHz
    uint8_t gNbnumContiguousCc = 1;  // 100MHz 안에 몇개의 CC가 들어가 있는지
    uint16_t gNbNumerology = 0;
    double gNbTxPower = 8.0;                // 단위dBm
    double gNbx = pow(10, gNbTxPower / 10); // to mW

    NodeContainer gNbNode;
    gNbNode.Create(gNbNum);

    // gNb, Uu 정적 위치 설정
    Ptr<ListPositionAllocator> gNbPositionAlloc = CreateObject<ListPositionAllocator>();
    gNbPositionAlloc->Add(Vector(0.0, 1.0, 0.0));

    MobilityHelper gNbMobility;
    gNbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gNbMobility.SetPositionAllocator(gNbPositionAlloc);
    gNbMobility.Install(gNbNode);

    // gNb band 설정
    CcBwpCreator gNbCcBwpCreators;
    OperationBandInfo gNbBand;

    CcBwpCreator::SimpleOperationBandConf gNbBandConf(
        gNbFrequencyBand,
        gNbBandwidthBand,
        gNbnumContiguousCc,
        BandwidthPartInfo::UMi_StreetCanyon_LoS); // 고속도로 시나리오 설정
    gNbBandConf.m_numBwp = 1;                     // 1 BWP per CC
    gNbBand = gNbCcBwpCreators.CreateOperationBandContiguousCc(gNbBandConf);

    nrHelper->InitializeOperationBand(&gNbBand);
    BandwidthPartInfoPtrVector gNbBwp;
    gNbBwp = CcBwpCreator::GetAllBwps({gNbBand});

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gNbNode, gNbBwp);

    // 빔포밍 설정
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // 안테나 설정
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_PREMIUM",
                                                 UintegerValue(0)); // bwp하나만 한거

    std::string pattern = "F|F|F|F|F|F|F|F|F|F|";
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(gNbNumerology));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("TxPower", DoubleValue(10 * log10(gNbx)));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("Pattern", StringValue(pattern));

    // 설정 적용
    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    // RSU, SL 기본 설정=======================================================
    uint16_t RsuNum = 1;
    double RsuFrequencyBand = 5.89e9;
    uint16_t RsuBandwidthBand = 400;
    uint8_t RsunumContiguousCc = 1;
    uint16_t RsuNumerology = 0;
    double RsuTxPower = 23.0; // 단위dBm
    // double Rsux = pow(10, RsuTxPower / 10); // to mW

    NodeContainer rsuNode;
    rsuNode.Create(RsuNum);

    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);

    // Rsu 정적 위치 설정
    Ptr<ListPositionAllocator> RsuPositionAlloc = CreateObject<ListPositionAllocator>();
    RsuPositionAlloc->Add(Vector(100.0, 0.0, 0.0));

    MobilityHelper RsuMobility;
    RsuMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    RsuMobility.SetPositionAllocator(RsuPositionAlloc);
    RsuMobility.Install(rsuNode);

    // RSU band 설정
    CcBwpCreator RsuCcBwpCreator;
    CcBwpCreator::SimpleOperationBandConf RsuBandConf(RsuFrequencyBand,
                                                      RsuBandwidthBand,
                                                      RsunumContiguousCc,
                                                      BandwidthPartInfo::V2V_Highway);
    OperationBandInfo RsuBand = RsuCcBwpCreator.CreateOperationBandContiguousCc(RsuBandConf);

    nrHelper->InitializeOperationBand(&RsuBand);
    BandwidthPartInfoPtrVector RsuBwp = CcBwpCreator::GetAllBwps({RsuBand});

    // todo : path loss모델 설정

    // RSU 안테나 설정
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(RsuTxPower)); // dBm그대로 넣는듯

    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId()); // 이거 필수임 이유는 찾아봐 todo
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(false));
    nrHelper->SetUeMacAttribute("T1", UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(33));
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));

    // todo : bwp설정. 여기 부분은 구현이 안된듯 주석 보면됨.
    uint8_t bwpIdForGbrMcptt = 0;
    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    // following parameter has no impact at the moment because:
    // 1. No support for PQI based mapping between the application and the LCs
    // 2. No scheduler to consider PQI
    // However, till such time all the NR SL examples should use GBR_MC_PUSH_TO_TALK
    // because we hard coded the PQI 65 in UE RRC.
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdForGbrMcptt));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdForGbrMcptt);

    std::vector<ObjectFactory> macSlFactory;
    ObjectFactory slfactory;
    slfactory.SetTypeId(NrSlUeMac::GetTypeId());
    macSlFactory.push_back(slfactory);

    NetDeviceContainer RsuNetDev = nrHelper->InstallUeDevice(rsuNode, RsuBwp, macSlFactory);

    // 설정 적용
    for (auto it = RsuNetDev.Begin(); it != RsuNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
        // Update the RRC config.Must be called only once.
    }

    //  SideLink 설정===============================================================
    // 스케쥴러 설정

    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());

    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));

    nrSlHelper->PrepareUeForSidelink(RsuNetDev, bwpIdContainer);

    // PrepareSingleUeForSidelink시발 여기서 에러남

    //  SlResourcePoolNr IE
    LteRrcSap::SlResourcePoolNr slResourcePoolNr;
    // get it from pool factory
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    std::vector<std::bitset<1>> slBitmap =
        {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1}; // The sidelink time resource bitmap

    ptrFactory->SetSlTimeResources(slBitmap);
    ptrFactory->SetSlSensingWindow(100);    //!< Start of the sensing window in milliseconds.
    ptrFactory->SetSlSelectionWindow(5);    //!< End of the selection window in number of slots.
    ptrFactory->SetSlFreqResourcePscch(10); // PSCCH RBs
    ptrFactory->SetSlSubchannelSize(50);
    ptrFactory->SetSlMaxNumPerReserve(3);
    std::list<uint16_t> resourceReservePeriodList = {0, 100}; // in ms
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);

    LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool();
    slResourcePoolNr = pool;

    // Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
    LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
    slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
    // Pool id, ranges from 0 to 15
    uint16_t poolId = 0;
    LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
    slResourcePoolIdNr.id = poolId;
    slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
    slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

    // Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
    // Array for pools, we insert the pool in the array as per its poolId
    slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[slResourcePoolIdNr.id] = slresoPoolConfigNr;
    // 풀을 여러개 쓸 수 있지만 우리는 영상 데이터를 전송하는 거니까 풀 하나만 쓰는게 맞을 듯

    // Configure the BWP IE
    LteRrcSap::Bwp bwp;
    bwp.numerology = RsuNumerology;
    bwp.symbolsPerSlots = 14; // ofdm symbol
    bwp.rbPerRbg = 1;         // Resource block per resource block group
    bwp.bandwidth = RsuBandwidthBand;

    // Configure the SlBwpGeneric IE
    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwp;
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    // Configure the SlBwpConfigCommonNr IE
    LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
    slBwpConfigCommonNr.haveSlBwpGeneric = true;
    slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
    slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
    slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

    // Configure the SlFreqConfigCommonNr IE, which hold the array to store
    // the configuration of all Sidelink BWP (s).
    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
    // Array for BWPs. Here we will iterate over the BWPs, which
    // we want to use for SL.
    for (const auto& it : bwpIdContainer)
    {
        // it is the BWP id
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    // Configure the TddUlDlConfigCommon IE
    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    tddUlDlConfigCommon.tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";

    // Configure the SlPreconfigGeneralNr IE
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Configure the SlUeSelectedConfig IE
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    slUeSelectedPreConfig.slProbResourceKeep = 0;
    // Configure the SlPsschTxParameters IE
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = 5;
    // Configure the SlPsschTxConfigList IE
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    /*
     * Finally, configure the SidelinkPreconfigNr This is the main structure
     * that needs to be communicated to NrSlUeRrc class
     */
    LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;

    // Communicate the above pre-configuration to the NrSlHelper
    nrSlHelper->InstallNrSlPreConfiguration(RsuNetDev, slPreConfigNr);

    // OBU 기본 설정===============================================================
    // obu 인스톨 할때 nrSlHelper->InstallNrSlPreConfiguration(obunetdev, slPreConfigNr);
    // 하면 될듯
    NodeContainer ueNode;
    uint16_t ueNum = 1;
    ueNode.Create(ueNum);
    double ueTxPower = 3.0;

    Ptr<ListPositionAllocator> uePositionAlloc =
        CreateObject<ListPositionAllocator>(); // ue위치 저장
    uePositionAlloc->Add(Vector(00.0, 0.0, 0.0));

    // ue이동성 설정
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    ueMobility.SetPositionAllocator(uePositionAlloc);
    ueMobility.Install(ueNode);

    // obu Uu연결 설정===============================================================
    nrHelper->SetUeMacTypeId(NrUeMac::GetTypeId()); // SL MAC에서 일반 MAC으로 변경

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    // 2*4 즉 8개 안테나
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(ueTxPower)); // dBm그대로 넣는듯

    // NetDeviceContainer ueUuNetDev = nrHelper->InstallUeDevice(ueNode, gNbBwp);

    std::vector<ObjectFactory> macUuFactory;
    ObjectFactory uufactory;
    uufactory.SetTypeId(NrUeMac::GetTypeId());
    macUuFactory.push_back(uufactory);

    NetDeviceContainer ueUuNetDev = nrHelper->InstallUeDevice(ueNode, gNbBwp, macUuFactory);

    DynamicCast<NrUeNetDevice>(ueUuNetDev.Get(0))->UpdateConfig();

    //========================================Set Uu settings same as the gnb
    // attach안하려면 저거 밑에로 설정해야함

    // Ptr<NrUeNetDevice> ueUusettingNetDev = ueUuNetDev.Get(0)->GetObject<NrUeNetDevice>();
    // Ptr<NrGnbNetDevice> gnbUusettingNetDev = gnbNetDev.Get(0)->GetObject<NrGnbNetDevice>();
    // for (uint32_t i = 0; i < gnbUusettingNetDev->GetCcMapSize(); ++i)
    // {
    //     ueUusettingNetDev->GetPhy(0)->SetDlAmc(
    //         DynamicCast<NrMacSchedulerNs3>(gnbUusettingNetDev->GetScheduler(i))->GetDlAmc());
    //     ueUusettingNetDev->GetPhy(i)->SetDlCtrlSyms(gnbUusettingNetDev->GetMac(i)->GetDlCtrlSyms());
    //     ueUusettingNetDev->GetPhy(i)->SetUlCtrlSyms(gnbUusettingNetDev->GetMac(i)->GetUlCtrlSyms());
    //     ueUusettingNetDev->GetPhy(i)->SetNumRbPerRbg(
    //         gnbUusettingNetDev->GetMac(i)->GetNumRbPerRbg());
    //     ueUusettingNetDev->GetPhy(i)->SetRbOverhead(gnbUusettingNetDev->GetPhy(i)->GetRbOverhead());
    //     ueUusettingNetDev->GetPhy(i)->SetSymbolsPerSlot(
    //         gnbUusettingNetDev->GetPhy(i)->GetSymbolsPerSlot());
    //     ueUusettingNetDev->GetPhy(i)->SetNumerology(gnbUusettingNetDev->GetPhy(i)->GetNumerology());
    //     ueUusettingNetDev->GetPhy(i)->SetPattern(gnbUusettingNetDev->GetPhy(i)->GetPattern());
    // }

    // obu Sl연결 설정===============================================================

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(10.0)); // dBm그대로 넣는듯

    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId()); // 이거 필수임 이유는 찾아봐 todo
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(false));
    nrHelper->SetUeMacAttribute("T1", UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(33));
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));

    NetDeviceContainer ueSlNetDev = nrHelper->InstallUeDevice(ueNode, RsuBwp, macSlFactory);
    DynamicCast<NrUeNetDevice>(ueSlNetDev.Get(0))->UpdateConfig(); // todo: obu sl

    nrSlHelper->PrepareUeForSidelink(ueSlNetDev, bwpIdContainer);
    nrSlHelper->InstallNrSlPreConfiguration(ueSlNetDev, slPreConfigNr);

    // Uu PHY에서 RSRP 측정 콜백 연결 (gNb와의 Uu 통신)
    Ptr<NrUeNetDevice> ueUuDev = DynamicCast<NrUeNetDevice>(ueUuNetDev.Get(0));
    // Get the first PHY (BWP) from the Uu NetDevice
    Ptr<NrUePhy> ueUuPhy = ueUuDev->GetPhy(0);
    ueUuPhy->TraceConnectWithoutContext("ReportRsrp", MakeCallback(&UeMeasCallback));

    // ueUuPhy->TraceConnectWithoutContext("ReportUeSlRsrpMeasurements",MakeCallback(&UeMeasCallback));
    // todo : 이거 왜 동작 안하냐 ReportUeSlRsrpMeasurements nr-ue-phy에 있음

    // 서버노드, pgw노드 설치===============================================================
    // todo : 한 노드 만들어서 gnb와 rsu 연결하고 인터넷 스텍 설치후 패킷 전송 되나 확인
    NodeContainer serverNodeContainer;
    serverNodeContainer.Create(1);
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    InternetStackHelper internet;
    internet.Install(serverNodeContainer);
    internet.Install(pgw);

    UdpServerHelper server(8000);
    server.Install(serverNodeContainer);

    // 서버와 pgw 연결----------------------------------------------------------------
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer pgwToCloud = p2ph.Install(pgw, serverNodeContainer.Get(0));
    NetDeviceContainer rsuToCloud = p2ph.Install(rsuNode.Get(0), serverNodeContainer.Get(0));

    // gnb rsu obu ip 설정-------------------------------------------------------------
    internet.Install(gNbNode);
    internet.Install(rsuNode);
    internet.Install(ueNode);

    Ipv6AddressHelper ipv6h;
    ipv6h.SetBase("fd00:a516:7c1b:17cd::", Ipv6Prefix(64));

    Ipv6InterfaceContainer ueUuIface =
        epcHelper->AssignUeIpv6Address(ueUuNetDev); // 이거는 eps베어러 생성 기지국과 연결해줌
    Ipv6InterfaceContainer ueSlIface = ipv6h.Assign(ueSlNetDev);
    Ipv6InterfaceContainer rsuIface = ipv6h.Assign(RsuNetDev);
    Ipv6InterfaceContainer pgwToCloudIface = ipv6h.Assign(pgwToCloud);
    Ipv6InterfaceContainer rsuToCloudIface = ipv6h.Assign(rsuToCloud);

    // gnb ue 연결
    nrHelper->AttachToClosestEnb(ueUuNetDev, gnbNetDev);

    // gNb, RSU와 서버 연결=====================================================

    // 데이터 통신==============================================================

    // SL bearer 설정해야함. 이거 해야지 sl rsrp 측정 일단 가능
    Time finalSlBearersActivationTime = Seconds(1.1);
    Time finalSimTime = simTime + finalSlBearersActivationTime;
    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Unicast;
    slInfo.m_dstL2Id = 255;
    slInfo.m_rri = MilliSeconds(100);
    slInfo.m_pdb = Seconds(0);
    slInfo.m_harqEnabled = true;
    {
        Ptr<LteSlTft> tft =
            Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, rsuIface.GetAddress(0, 0), slInfo);
        // Set Sidelink bearers
        nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, ueSlNetDev, tft);
    }

    // 패킷 설정
    //  -- 송신 애플리케이션 생성 (클라이언트)
    uint16_t port = 5678;
    double dataRateBe = 16;
    std::string dataRateBeString = std::to_string(dataRateBe) + "kb/s";

    Ipv6Address rsuAddr = rsuIface.GetAddress(0, 0);
    Address remoteAddr = Inet6SocketAddress(rsuAddr, port);
    OnOffHelper slClient("ns3::UdpSocketFactory", remoteAddr);
    slClient.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    slClient.SetConstantRate(DataRate(dataRateBeString), 200);

    // ueVoiceContainer.Get(0) 노드에 OnOffApplication 을 설치
    ApplicationContainer slClientApps = slClient.Install(ueNode.Get(0));

    // 애플리케이션 시작 시점
    slClientApps.Start(finalSlBearersActivationTime);
    slClientApps.Stop(finalSimTime);

    Simulator::Schedule(Seconds(1.0), &LogUeFlowInfo, ueNode.Get(0));
    // sl에서는 rsrp가 측정결과가 아니니까 이거 어케 해결하지?


    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();
}