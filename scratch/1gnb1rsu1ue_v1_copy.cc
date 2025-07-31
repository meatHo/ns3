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
LogUeFlowInfo (Ptr<Node> ue)
{
    Vector pos = ue->GetObject<MobilityModel> ()->GetPosition ();
    std::cout << "🕒 " << Simulator::Now ().GetSeconds ()
              << "s | UE 위치: (" << pos.x << ", " << pos.y << ")\n";

    // --- UU NetDevice (인덱스 0) ---
    Ptr<NrUeNetDevice> ueUuDev =
        ue->GetDevice (0)->GetObject<NrUeNetDevice> ();
    if (ueUuDev && ueUuDev->GetTargetEnb ())
    {
        std::cout << "🔗 UU 연결된 gNb ID: "
                  << ueUuDev->GetTargetEnb ()->GetCellId ()
                  << "  CELL ID: "
                  << ueUuDev->GetRrc ()->GetCellId ()
                  << "\n";
    }
    else
    {
        std::cout << "⚠️ UU 아직 gNB에 연결되지 않았습니다.\n";
    }

    // --- SL NetDevice (인덱스 1) ---
    Ptr<NrUeNetDevice> ueSlDev =
        ue->GetDevice (1)->GetObject<NrUeNetDevice> ();
    if (ueSlDev)
    {
        // 1) RRC 상 SL Bearer 개수 조회
        // auto slBearerMap = ueSlDev->GetRrc ()->GetSlBearerMap ();
        // std::cout << "🔗 SL Bearer 수: " << slBearerMap.size () << "\n";

        // 2) PHY 상으로도 SL 계측 가능: 예를 들어 현재 Tx 전력
        double txPwr = ueSlDev->GetPhy (0)->GetTxPower ();
        std::cout << "⚙️ SL PHY TxPower = " << txPwr << " dBm\n";
    }
    else
    {
        std::cout << "⚠️ SL 장치가 없습니다.\n";
    }

    // 1초 뒤 재호출
    Simulator::Schedule (Seconds (1.0), &LogUeFlowInfo, ue);
}

int
main(void)
{
    Time simTime = Seconds(2);

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
    // double Rsux = pow(10, RsuTxPower / 10); // to mWdddd

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

    NetDeviceContainer RsuNetDev = nrHelper->InstallUeDevice(rsuNode, RsuBwp);

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

    std::vector<ObjectFactory> macSlFactory;
    ObjectFactory slfactory;
    slfactory.SetTypeId(NrSlUeMac::GetTypeId());
    macSlFactory.push_back(slfactory);

    NetDeviceContainer ueSlNetDev = nrHelper->InstallUeDevice(ueNode, RsuBwp, macSlFactory);
    DynamicCast<NrUeNetDevice>(ueSlNetDev.Get(0))->UpdateConfig();

    nrSlHelper->PrepareUeForSidelink(ueSlNetDev, bwpIdContainer);
    nrSlHelper->InstallNrSlPreConfiguration(ueSlNetDev, slPreConfigNr);

    // Uu PHY에서 RSRP 측정 콜백 연결 (gNb와의 Uu 통신)
    Ptr<NrUeNetDevice> ueUuDev = DynamicCast<NrUeNetDevice>(ueUuNetDev.Get(0));
    // Get the first PHY (BWP) from the Uu NetDevice
    Ptr<NrUePhy> ueUuPhy = ueUuDev->GetPhy(0);
    ueUuPhy->TraceConnectWithoutContext("ReportRsrp", MakeCallback(&UeMeasCallback));





    // ueUuPhy->TraceConnectWithoutContext("ReportUeSlRsrpMeasurements",MakeCallback(&UeMeasCallback));
    // todo : 이거 왜 동작 안하냐 ReportUeSlRsrpMeasurements nr-ue-phy에 있음

    // 인터넷 설정===============================================================

    // 인터넷 설치(gNb, RSU)================================================프로토콜 설정

    // 서버 설치===============================================================
    // todo : 한 노드 만들어서 gnb와 rsu 연결하고 인터넷 스텍 설치후 패킷 전송 되나 확인

    // gNb, RSU와 서버 연결=====================================================

    // 데이터 통신==============================================================

    // 테스트용 코드 여 밑에 다 지워도 됨
    Ptr<Node> pgw = epcHelper->GetPgwNode(); // 인터넷으로 나가는 출구 역할(EPC에서 PGW로 보냄)
    NodeContainer remoteHostContainer;       // 원격 호스트를 담을 컨테이너
    remoteHostContainer.Create(1);           // 인터넷에 연결된 서버 노드  생성
    Ptr<Node> remoteHost = remoteHostContainer.Get(0); // 생성한 노드 포인터로 꺼냄
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    internet.Install(pgw);
    // remoteHost에 TCP/IP 스택 설치 이걸해야 인터넷에서 패킷 주고 받을 수 있음

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    // pgw와 remotehost직접 연결(100Gbps 속도, 0초 지연, 2500 MTU(최대 전송 패킷))

    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    // PGW/remoteHost에 IP 주소 부여(prefix는 /8 (255.0.0.0))

    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    // remotehost에 정적 라우팅
    // 목적지가 7.0.0.0/8인 패킷은 인터페이스 1 (→ PGW)로 보냄
    // UE들(보통 7.x.x.x IP를 받음)에게 보내는 패킷을 PGW를 통해 전달

    internet.Install(ueNode);
    Ipv4InterfaceContainer ueUUIpIface;

    // OBU에서 UU스택에서 ip설치 todo: 여기 밑은 지우면 안됨!!!!!!!!!!!
    ueUUIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueUuNetDev));
    // 각 ue에 TCP/IP 스택 설치, EPC를 통해 각 UE주소 할당(위에 정의한대로 7.0.0.X형태)
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);
    // remotehost ip주소 저장 (서버 접속용으로 사용)
    //[UE] ---(5G 무선)--> [gNB] --> [PGW] <--(P2P)-- [remoteHost]

    // Set the default gateway for the UEs
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNode.Get(0)->GetObject<Ipv4>());
    // UE 노드의 라우팅 테이블에 접근
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    // 기본 게이트웨이 주소 설정
    uint16_t numFlowsUe = 1; // UE마다 한개의 flow 생성
    uint16_t dlPort = 1234;
    uint16_t ulPort = dlPort + gNbNum * ueNum * numFlowsUe + 1;

    ApplicationContainer clientApps;
    ApplicationContainer serverApps;

    bool disableDl = false;
    bool disableUl = false;
    uint32_t udpPacketSizeBe = 100;
    uint32_t lambdaUll = 2;

    uint16_t u = 0;
    if (!disableDl)
    { // 이게 무조건 처음 설치한 인터페이스로 패킷을 보냄
        PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        // UE가 수신쪽임 (지정된 포트로 오는 어떤 데이터든(GetAny) 받아줌)
        serverApps.Add(dlPacketSinkHelper.Install(ueNode.Get(u)));

        UdpClientHelper dlClient(ueUUIpIface.GetAddress(u),
                                 dlPort); // 목적지(UE IP), 목적지 포트
        dlClient.SetAttribute("PacketSize",
                              UintegerValue(udpPacketSizeBe)); // 패킷 하나의 크기
        dlClient.SetAttribute("Interval",
                              TimeValue(Seconds(1.0 / lambdaUll))); // 전송 주기(패킷 전송률 제어)
        dlClient.SetAttribute("MaxPackets",
                              UintegerValue(0xFFFFFFFF)); // 엄청나게 많은 패킷 16진수
        clientApps.Add(dlClient.Install(remoteHost));

        Ptr<EpcTft> tft = Create<EpcTft>(); // Traffic Flow Template(특정 조건을 만족하는 패킷을
                                            // 어떤 Bearer에 매핑할지 결정)
        EpcTft::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd = dlPort;
        ++dlPort; // 다음 flow위해 포트번호 증가 => 즉 각 flow마다 다른 포트 사용중
        tft->Add(dlpf);

        // QoS클래스(QCI) 결정
        enum EpsBearer::Qci q;

        q = EpsBearer::NGBR_VIDEO_TCP_PREMIUM;

        // EPS Bearer 생성 + 활성화
        EpsBearer bearer(q);
        nrHelper->ActivateDedicatedEpsBearer(ueUuNetDev.Get(u), bearer, tft);
    }

    if (!disableUl)
    {
        PacketSinkHelper ulPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        serverApps.Add(ulPacketSinkHelper.Install(remoteHost));

        UdpClientHelper ulClient(remoteHostAddr, ulPort);
        ulClient.SetAttribute("PacketSize", UintegerValue(udpPacketSizeBe));
        ulClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambdaUll)));
        ulClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        clientApps.Add(ulClient.Install(ueNode.Get(u)));

        Ptr<EpcTft> tft = Create<EpcTft>();
        EpcTft::PacketFilter ulpf;
        ulpf.remotePortStart = ulPort;
        ulpf.remotePortEnd = ulPort;
        ++ulPort;
        tft->Add(ulpf);

        enum EpsBearer::Qci q;
        q = EpsBearer::NGBR_VIDEO_TCP_PREMIUM;

        EpsBearer bearer(q);
        nrHelper->ActivateDedicatedEpsBearer(ueUuNetDev.Get(u), bearer, tft);
    }
    nrHelper->AttachToClosestEnb(ueUuNetDev, gnbNetDev);

    //------------------------end of UU setting------------------------------------------

    // SL 전용 AddressHelper

    Ipv4AddressHelper slIpv4h;
    slIpv4h.SetBase ("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer ueSlIpIface = slIpv4h.Assign (ueSlNetDev);

    // OBU에서 SideLink스택에서 ip설치 //todo : 여기 sidelink 스택 설치 후 rsrp측정되나 안되나 확인
    // Ipv4InterfaceContainer ueSlIpIface;
    // ueSlIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueSlNetDev));
    Address ueRemoteAddress;
    Address ueLocalAddress;
    Ipv4Address ueAddress4("225.0.0.0");
    uint16_t port = 8000;

    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Unicast;
    slInfo.m_dstL2Id = 255;
    slInfo.m_rri = MilliSeconds(100);
    slInfo.m_pdb = Seconds(0); // delay budget
    slInfo.m_harqEnabled = true;

    for (uint32_t u = 0; u < ueSlNetDev.GetN(); ++u)
    {
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueSlNetDev.Get(u)->GetNode()->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

        ueRemoteAddress = InetSocketAddress(ueAddress4, port);
        ueLocalAddress = InetSocketAddress(Ipv4Address::GetAny(), port);
        Ptr<LteSlTft> tft =
            Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, ueAddress4, slInfo);
        // Set Sidelink bearers
        nrSlHelper->ActivateNrSlBearer(Seconds(1), ueSlNetDev, tft);
    }

    //rsu sidelink스택에 ip 설치 todo:

    // Rsu에서 SideLink스택에서 ip설치 //todo : 여기 sidelink 스택 설치 후 rsrp측정되나 안되나 확인
    // Ipv4InterfaceContainer rsuSlIpIface;
    // rsuSlIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(RsuNetDev));
    internet.Install(rsuNode);
    Ipv4InterfaceContainer rsuSlIpIface = slIpv4h.Assign (RsuNetDev);
    Address rsuRemoteAddress;
    Address rsuLocalAddress;
    Ipv4Address groupAddress4("225.0.0.0");

    // SidelinkInfo slInfo;
    // slInfo.m_castType = SidelinkInfo::CastType::Unicast;
    // slInfo.m_dstL2Id = 255;
    // slInfo.m_rri = MilliSeconds(100);
    // slInfo.m_pdb = Seconds(0); // delay budget
    // slInfo.m_harqEnabled = true;

    for (uint32_t u = 0; u < RsuNetDev.GetN(); ++u)
    {
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> rsuStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(RsuNetDev.Get(u)->GetNode()->GetObject<Ipv4>());
        rsuStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

        rsuRemoteAddress = InetSocketAddress(groupAddress4, port);
        rsuLocalAddress = InetSocketAddress(Ipv4Address::GetAny(), port);
        Ptr<LteSlTft> tft =
            Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, slInfo);
        // Set Sidelink bearers
        nrSlHelper->ActivateNrSlBearer(Seconds(1), RsuNetDev, tft);
    }

    // Ptr<NrUeNetDevice> ueSlDev = DynamicCast<NrUeNetDevice>(ueSlNetDev.Get(0));
    // Ptr<NrUePhy> ueSlPhy = ueSlDev->GetPhy(0);
    // ueSlPhy->SetSlReport();
    //
    // Ptr<NrUeNetDevice> rsuSlDev = DynamicCast<NrUeNetDevice>(RsuNetDev.Get(0));
    // Ptr<NrUePhy> rsuSlPhy = rsuSlDev->GetPhy(0);
    // rsuSlPhy->SetSlReport();

    Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice> (RsuNetDev.Get (0));
    Ptr<NrUePhy>       uePhy   = ueDev->GetPhy (0);
    uePhy->GetNrSlUeCphySapProvider ()->EnableUeSlRsrpMeasurements ();

    Ptr<NrUeNetDevice> rsuDev = DynamicCast<NrUeNetDevice> (ueSlNetDev.Get (0));
    Ptr<NrUePhy>       rsuPhy   = ueDev->GetPhy (0);
    rsuPhy->GetNrSlUeCphySapProvider ()->EnableUeSlRsrpMeasurements ();

    double udpAppStartTime = 0.4; // seconds

    // start UDP server and client apps
    serverApps.Start(Seconds(udpAppStartTime));
    clientApps.Start(Seconds(udpAppStartTime));
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    // enable the traces provided by the nr module
    // nrHelper->EnableTraces();//nr모듈에서 제공하는 내부(PHY,MAX,RLC) 트레이스

    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(ueNode);

    Ptr<ns3::FlowMonitor> monitor =
        flowmonHelper.Install(endpointNodes); // 각 노드 간의 UDP flow 추적
    // BinWidth는 히스토그램 형태로 분포 나타낼 때 간격을 몇 초 단위로 나눌지
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001)); // 지연 측정 구간(1ms)
    monitor->SetAttribute("JitterBinWidth",
                          DoubleValue(0.001)); // 지터(패킷 간 전송 간격의 변화량) 측정 구간(1ms)
    // 패킷이 규칙적으로 오지 않고 도착할 때 그 불안정성 측정
    monitor->SetAttribute("PacketSizeBinWidth",
                          DoubleValue(20)); // 패킷 크기 히스토그램 구간(20바이트 단위)
                                            // 패킷의 크기를 몇 바이트 단위로 나눌지

    Simulator::Schedule(Seconds(1.0), &LogUeFlowInfo, ueNode.Get(0));
    // sl에서는 rsrp가 측정결과가 아니니까 이거 어케 해결하지?
    // 일단 gnb rsrp측정되는지 먼저 확인 ㄱㄱ 패킷관련 설정이 없어서 안되는듯
    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();
}