#include "ns3/address.h"
#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/csma-helper.h"
#include "ns3/epc-tft.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-module.h"
#include "ns3/nr-sl-ue-cphy-sap.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"
#include <ns3/pointer.h>
using namespace ns3;


void
UeRssiPerProcessedChunk(Ptr<SpectrumPhy> phy, double rssidBm)
{
    static uint8_t cnt = 0;
    static double sum = 0.0;
    static double totalsum = 0.0;
    static uint16_t totalcnt=0;
    cnt++;
    sum += rssidBm;

    if (cnt == 50)
    {
        double avg = sum / cnt;

        Ptr<NrSpectrumPhy> nrPhy = DynamicCast<NrSpectrumPhy>(phy);
        Ptr<NetDevice> dev = nrPhy->GetDevice();
        Ptr<Node> node = dev->GetNode();

        uint32_t nodeId  = node->GetId();
        uint32_t devIdx  = dev->GetIfIndex();

        uint16_t cellId  = nrPhy->GetCellId();
        //uint16_t rnti    = nrPhy->GetRnti();
        uint16_t bwpId   = nrPhy->GetBwpId();

        std::cout << "[Uu Node "   << nodeId
                  << " | Dev "   << devIdx
                  << " | Cell "  << cellId
                  //<< " | RNTI "  << rnti
                  << " | BWP "   << bwpId
                  << "] 10‑Chunk Avg RSSI = "
                  << avg << " dBm"
                  << std::endl;
        totalsum += sum;
        totalcnt += cnt;
        cnt = 0;
        sum = 0.0;
    }
}

void
UeSlRssiPerProcessedChunk(Ptr<SpectrumPhy> phy, double rssidBm)
{
    static uint8_t cnt = 0;
    static double sum = 0.0;
    static double totalsum = 0.0;
    static uint16_t totalcnt=0;
    cnt++;
    sum += rssidBm;

    if (cnt == 50)
    {
        double avg = sum / cnt;

        Ptr<NrSpectrumPhy> nrPhy = DynamicCast<NrSpectrumPhy>(phy);
        Ptr<NetDevice> dev = nrPhy->GetDevice();
        Ptr<Node> node = dev->GetNode();

        uint32_t nodeId  = node->GetId();
        uint32_t devIdx  = dev->GetIfIndex();

        uint16_t cellId  = nrPhy->GetCellId();
        //uint16_t rnti    = nrPhy->GetRnti();
        uint16_t bwpId   = nrPhy->GetBwpId();

        std::cout << "[SideLink Node "   << nodeId
                  << " | Dev "   << devIdx
                  << " | Cell "  << cellId
                  //<< " | RNTI "  << rnti
                  << " | BWP "   << bwpId
                  << "] 10‑Chunk Avg RSSI = "
                  << avg << " dBm"
                  << std::endl;
        totalsum += sum;
        totalcnt += cnt;
        cnt = 0;
        sum = 0.0;
    }
}

// 패킷 정보를 출력할 콜백 함수
void Ipv6PacketTraceAtRsu(Ptr<const Packet> packet, Ptr<Ipv6> ipv6, uint32_t interfaceIndex)
{
    // 패킷에서 IPv6 헤더 정보를 읽어옵니다. (PeekHeader는 원본 패킷을 수정하지 않습니다)
    Ipv6Header ipv6Header;
    packet->PeekHeader(ipv6Header);

    std::cout << "[RSU Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex
              << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}
void Ipv6PacketTraceAtPgw(Ptr<const Packet> packet, Ptr<Ipv6> ipv6, uint32_t interfaceIndex)
{
    // 패킷에서 IPv6 헤더 정보를 읽어옵니다. (PeekHeader는 원본 패킷을 수정하지 않습니다)
    Ipv6Header ipv6Header;
    packet->PeekHeader(ipv6Header);

    std::cout << "[PGW Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex
              << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}


struct clientInfo
{
    Address address;
    uint32_t lastSequenceNum;
    Time connectionTime;
    uint32_t RTT;
    float_t packetLossRate;
    uint64_t totalBytesReceived;
};


// udp 서버 설정===============================================================
class UdpServerk : public Application
{
  public:
    static TypeId GetTypeId();
    UdpServerk();
    ~UdpServerk() override {};
    void SetPacketWindowSize(uint16_t size);
    uint16_t GetPacketWindowSize() const;

  private:
    std::map<uint16_t, clientInfo> clients;
    void StartApplication() override;
    void StopApplication() override;
    void SendPacket(uint16_t clientId, std::string message);
    void HandleRead(Ptr<Socket> socket);
    uint16_t m_port; //!< Port on which we listen for incoming packets.
    uint8_t m_tos;   //!< The packets Type of Service
    // Ptr<Socket> m_socket;            //!< IPv4 Socket
    Ptr<Socket> m_socket6;           //!< IPv6 Socket
    uint64_t m_received;             //!< Number of received packets
    PacketLossCounter m_lossCounter; //!< Lost packet counter
    uint16_t m_nextClientId;

    /// Callbacks for tracing the packet Rx events
    TracedCallback<Ptr<const Packet>> m_rxTrace;

    /// Callbacks for tracing the packet Rx events, includes source and destination addresses
    TracedCallback<Ptr<const Packet>, const Address&, const Address&> m_rxTraceWithAddresses;
};

NS_OBJECT_ENSURE_REGISTERED(UdpServerk); // GetTypeId 위에 추가

TypeId
UdpServerk::GetTypeId()
{
    static TypeId tid =
        TypeId("UdpServerk") // ★★★ 고유한 이름 사용 (매우 중요)
            .SetParent<Application>()
            .SetGroupName("Applications")
            .AddConstructor<UdpServerk>()
            // 포트는 Attribute로 설정하는 것이 표준입니다.
            .AddAttribute("Port",
                          "Port on which we listen for incoming packets.",
                          UintegerValue(9000), // 기본 포트값
                          MakeUintegerAccessor(&UdpServerk::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("Tos",
                          "The Type of Service used to send IPv4 packets. "
                          "All 8 bits of the TOS byte are set (including ECN bits).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&UdpServerk::m_tos),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("PacketWindowSize",
                          "The size of the window used to compute the packet loss. This value "
                          "should be a multiple of 8.",
                          UintegerValue(32),
                          MakeUintegerAccessor(&UdpServerk::GetPacketWindowSize,
                                               &UdpServerk::SetPacketWindowSize),
                          MakeUintegerChecker<uint16_t>(8, 256))
            .AddTraceSource("Rx",
                            "A packet has been received",
                            MakeTraceSourceAccessor(&UdpServerk::m_rxTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("RxWithAddresses",
                            "A packet has been received",
                            MakeTraceSourceAccessor(&UdpServerk::m_rxTraceWithAddresses),
                            "ns3::Packet::TwoAddressTracedCallback");
    return tid;
}

UdpServerk::UdpServerk()
    : m_received(0),
      m_lossCounter(0)
{
    m_nextClientId = 0;
}

void
UdpServerk::StartApplication()
{
    // 소켓 만들어서 대입
    std::cout << "udp serverk StartApplication" << std::endl;
    m_socket6 = Socket::CreateSocket(GetNode(), TypeId(UdpSocketFactory::GetTypeId()));
    Inet6SocketAddress local = Inet6SocketAddress(Ipv6Address::GetAny(), m_port);

    if (m_socket6->Bind(local) == -1)
    {
        NS_FATAL_ERROR("Failed to bind socket");
    }
    m_socket6->SetRecvCallback(MakeCallback(&UdpServerk::HandleRead, this));
}

void
UdpServerk::StopApplication()
{
    if (m_socket6)
    {
        m_socket6->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
}

uint16_t
UdpServerk::GetPacketWindowSize() const
{
    return m_lossCounter.GetBitMapSize();
}

void
UdpServerk::SetPacketWindowSize(uint16_t size)
{
    std::cout << "bitmap size:" << size << std::endl;
    m_lossCounter.SetBitMapSize(size);
}

void
UdpServerk::HandleRead(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    Address localAddress;
    while ((packet = socket->RecvFrom(from)))
    {
        bool clientFound = false;
        uint16_t clientId;
        for (const auto& pair : clients)
        {
            if (pair.second.address == from)
            {
                clientFound = true;
                clientId = pair.first;
                from = clients[clientId].address;

                break;
            }
        }
        // 새 클라이언트 저장
        if (!clientFound)
        {
            std::cout << "new client detected" << std::endl;
            uint16_t newId = m_nextClientId++;
            clientInfo newClient;
            newClient.address = from;
            newClient.lastSequenceNum = 0;
            newClient.connectionTime = Simulator::Now();
            newClient.packetLossRate = 0;
            newClient.RTT = 0;
            newClient.totalBytesReceived = 0;
            clients[newId] = newClient;
            clientId = newId;
        }

        // 수신
        socket->GetSockName(localAddress);
        m_rxTrace(packet);
        m_rxTraceWithAddresses(packet, from, localAddress);
        if (packet->GetSize() > 0)
        {
            uint32_t receivedSize = packet->GetSize();
            SeqTsHeader seqTs;
            packet->RemoveHeader(seqTs);
            uint32_t currentSequenceNumber = seqTs.GetSeq();
            if (InetSocketAddress::IsMatchingType(from))
            {
                std::cout << "TraceDelay: RX " << receivedSize << " bytes from "
                          << InetSocketAddress::ConvertFrom(from).GetIpv4()
                          << "port: "<<InetSocketAddress::ConvertFrom(from).GetPort()
                          << " Sequence Number: " << currentSequenceNumber
                          << " Uid: " << packet->GetUid() << " TXtime: " << seqTs.GetTs()
                          << " RXtime: " << Simulator::Now()
                          << " Delay: " << Simulator::Now() - seqTs.GetTs() << std::endl;
            }
            else if (Inet6SocketAddress::IsMatchingType(from))
            {
                std::cout << "TraceDelay: RX " << receivedSize << " bytes from "
                          << Inet6SocketAddress::ConvertFrom(from).GetIpv6()
                          << " port: "<<Inet6SocketAddress::ConvertFrom(from).GetPort()
                          << " Sequence Number: " << currentSequenceNumber
                          << " Uid: " << packet->GetUid() << " TXtime: " << seqTs.GetTs()
                          << " RXtime: " << Simulator::Now()
                          << " Delay: " << Simulator::Now() - seqTs.GetTs() << std::endl;
            }

            m_lossCounter.NotifyReceived(currentSequenceNumber);
            m_received++;

            SendPacket(clientId, "good");
            std::cout << "sent to client - clientId : "<< clientId << std::endl;
        }
    }
}

void
UdpServerk::SendPacket(uint16_t clientId, std::string message)
{
    auto it = clients.find(clientId);

    if (it == clients.end())
    {
        std::cout << "SendPacket failed: Client with ID " << clientId << " not found." << std::endl;
        return;
    }

    Address destAddress = it->second.address;
    Ptr<Packet> packet = Create<Packet>((uint8_t*)message.c_str(), message.length());

    // if (InetSocketAddress::IsMatchingType(destAddress))
    // {
    //     // IPv4 주소일 경우 m_socket 사용
    //     m_socket->SendTo(packet, 0, destAddress);
    //     NS_LOG_INFO("Sent an IPv4 packet to client ID " << clientId);
    // }
    if (Inet6SocketAddress::IsMatchingType(destAddress))
    {
        // IPv6 주소일 경우 m_socket6 사용
        m_socket6->SendTo(packet, 0, destAddress);
        std::cout << "Sent an IPv6 packet to client ID " << clientId << std::endl;
        ;
    }
}

int
main(void)
{
    Time simTime = Seconds(30);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    NodeContainer gnbNodeContainer;
    gnbNodeContainer.Create(1);
    NodeContainer rsuNodeContainer;
    rsuNodeContainer.Create(1);
    NodeContainer serverNodeContainer;
    serverNodeContainer.Create(1);
    NodeContainer ueNodeContainer;
    ueNodeContainer.Create(1);


    Ptr<Node> pgw = epcHelper->GetPgwNode(); // ipv4, ipv6 둘다 설치되어 있음. 듀얼스택
    Ptr<Node> server = serverNodeContainer.Get(0);
    Ptr<Node> rsu = rsuNodeContainer.Get(0);
    Ptr<Node> gnb = gnbNodeContainer.Get(0);
    Ptr<Node> ue = ueNodeContainer.Get(0);


    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(ueNodeContainer);
    ueNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.0));
    mobility.Install(gnbNodeContainer);
    gnbNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(10.0, 0.0, 0.0));
    mobility.Install(rsuNodeContainer);
    rsuNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 10.0, 0.0));
    mobility.Install(serverNodeContainer);
    serverNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(2000.0, 0.0, 0.0));


    Ptr<ListPositionAllocator> rsuPositionAlloc = CreateObject<ListPositionAllocator>();
    rsuPositionAlloc->Add(Vector(1, 1, 1));

    mobility.SetPositionAllocator(rsuPositionAlloc);
    mobility.Install(rsuNodeContainer);

    // gnb bwp 설정

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    double gNbFrequencyBand = 3.5e9; // 3.5GHz
    double gNbBandwidthBand = 1e8;   // 100MHz
    uint8_t gNbnumContiguousCc = 1;  // 100MHz 안에 몇개의 CC가 들어가 있는지
    uint16_t gNbNumerology = 0;
    double gNbTxPower = 8.0;                // 단위dBm
    double gNbx = pow(10, gNbTxPower / 10); // to mW

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

    // 빔포밍 설정
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    std::vector<ObjectFactory> macUuFactory;
    ObjectFactory uufactory;
    uufactory.SetTypeId(NrUeMac::GetTypeId());
    macUuFactory.push_back(uufactory);
    NetDeviceContainer gnbNetDev =
        nrHelper->InstallGnbDevice(gnbNodeContainer,
                                   gNbBwp); // todo:결국 gnb, pgw에 ip할당하려면 bwp세팅 해야함

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
    NetDeviceContainer ueUuNetDev =
        nrHelper->InstallUeDevice(ueNodeContainer, gNbBwp, macUuFactory);

    DynamicCast<NrUeNetDevice>(ueUuNetDev.Get(0))->UpdateConfig();

    // 설정 적용
    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    // RSU, SL 기본 설정=======================================================
    double RsuFrequencyBand = 5.89e9;
    uint16_t RsuBandwidthBand = 400;
    uint8_t RsunumContiguousCc = 1;
    uint16_t RsuNumerology = 0;
    double RsuTxPower = 23.0; // 단위dBm
    // double Rsux = pow(10, RsuTxPower / 10); // to mW

    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);

    // RSU band 설정
    CcBwpCreator RsuCcBwpCreator;
    CcBwpCreator::SimpleOperationBandConf RsuBandConf(RsuFrequencyBand,
                                                      RsuBandwidthBand,
                                                      RsunumContiguousCc,
                                                      BandwidthPartInfo::V2V_Highway);
    OperationBandInfo RsuBand = RsuCcBwpCreator.CreateOperationBandContiguousCc(RsuBandConf);

    nrHelper->InitializeOperationBand(&RsuBand);
    BandwidthPartInfoPtrVector RsuBwp = CcBwpCreator::GetAllBwps({RsuBand});

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

    NetDeviceContainer rsuNetDev = nrHelper->InstallUeDevice(rsuNodeContainer, RsuBwp, macSlFactory);

    // 설정 적용
    for (auto it = rsuNetDev.Begin(); it != rsuNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
        // Update the RRC config.Must be called only once.
    }

    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());

    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));


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


    NetDeviceContainer ueSlNetDev = nrHelper->InstallUeDevice(ueNodeContainer, RsuBwp, macSlFactory);
    DynamicCast<NrUeNetDevice>(ueSlNetDev.Get(0))->UpdateConfig(); // todo: obu sl

    NetDeviceContainer SlNetDev;
    SlNetDev.Add(ueSlNetDev);
    SlNetDev.Add(rsuNetDev);




    // 진짜 시작todo:
    // ===============================================================================

    NodeContainer nodes(server);
    NodeContainer routers(pgw,rsu);

    // 여기서 p2p를 쓸지 csma를 쓸지 결정해야할듯
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("10ms"));
    NetDeviceContainer pgwToServerNetDev = p2ph.Install(pgw, server);
    NetDeviceContainer rsuToServerNetDev = p2ph.Install(rsu, server);
    p2ph.EnablePcap("rsu-to-server", rsuToServerNetDev.Get(0), true);

    //인터넷 설정
    InternetStackHelper internet;
    internet.SetIpv4StackInstall(false);
    internet.Install(ueNodeContainer);
    internet.Install(nodes);

    InternetStackHelper internetv6;
    internetv6.SetIpv4StackInstall(false);
    internetv6.Install(routers);




    // ip설정
    Ipv6AddressHelper ipv6h;

    //rsu server
    ipv6h.SetBase("fd00:1::", Ipv6Prefix(64));
    Ipv6InterfaceContainer iic1 = ipv6h.Assign(rsuToServerNetDev);
    iic1.SetForwarding(0, true);

    Ipv6Address rsuServerIpv6 = iic1.GetAddress(1, 1);

    //pgw server
    ipv6h.SetBase("fd00:3::", Ipv6Prefix(64));
    Ipv6InterfaceContainer iic3 = ipv6h.Assign(pgwToServerNetDev);
    iic3.SetForwarding(0, true);
    Ipv6Address gnbServerIpv6 = iic3.GetAddress(1, 1);

    Ipv6InterfaceContainer ueUuIface = epcHelper->AssignUeIpv6Address(ueUuNetDev);
    nrHelper->AttachToClosestEnb(ueUuNetDev, gnbNetDev); // 이거는 eps베어러 생성 기지국과 연결해줌

    Ipv6InterfaceContainer ueSlIface = epcHelper->AssignUeIpv6Address(SlNetDev);
    ueSlIface.SetForwarding(1, true);
    Ipv6Address temp = ueSlIface.GetAddress(1, 1);

    nrSlHelper->PrepareUeForSidelink(SlNetDev, bwpIdContainer);
    nrSlHelper->InstallNrSlPreConfiguration(SlNetDev, slPreConfigNr);


    //sidelink 무선 베어러 설정
    Ptr<LteSlTft> tft;
    uint32_t dstL2Id = 255;
    Time delayBudget = Seconds(0);

    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id = dstL2Id;
    slInfo.m_rri = MilliSeconds(100);
    slInfo.m_pdb = delayBudget;
    slInfo.m_harqEnabled = true;

    tft = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, temp, slInfo);
    nrSlHelper->ActivateNrSlBearer(Seconds(1.0),ueSlNetDev,tft);

    tft = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, temp, slInfo);
    nrSlHelper->ActivateNrSlBearer(Seconds(1.0),rsuNetDev,tft);

    //앱 설치
    ApplicationContainer clientAppContainer;
    ApplicationContainer serverAppContainer;

    u_int16_t serverPort = 5000;
    // u_int16_t clientPort = 4000;
    // AdaptiveUdpClientk adaptiveUdpClientk(rsuIpv6,serverIpv6, serverPort);
    // adaptiveUdpClientk.SetAttribute("Interval",TimeValue(Seconds(3)));

    Ptr<UdpServerk> serverApp = CreateObject<UdpServerk>();
    server->AddApplication(serverApp);
    serverApp->SetAttribute("Port", UintegerValue(serverPort));
    serverApp->SetStartTime(Seconds(1.0));
    serverApp->SetStopTime(simTime);


    // 세팅 끝 todo: =============================================================

    // **시나리오 1: 5G UE가 서버로 메시지 전송**


    Ptr<UdpClient> clientApp = CreateObject<UdpClient>();
    clientApp->SetAttribute("MaxPackets", UintegerValue(3));
    clientApp->SetAttribute("Interval", TimeValue(Seconds(5.0)));
    clientApp->SetAttribute("PacketSize", UintegerValue(100));

    ue->AddApplication(clientApp);
    clientApp->SetStartTime(Seconds(9.0));
    clientApp->SetStopTime(simTime);
//todo:여기다가포트랑 주소 넣어야함
    clientApp->setAddressSlUu(gnbServerIpv6,serverPort,temp,serverPort);

    //ue pgw 라우팅
    Ipv6StaticRoutingHelper ipv6RoutingHelper;
    Ptr<Ipv6StaticRouting> ueUuStaticRouting = ipv6RoutingHelper.GetStaticRouting(ue->GetObject<Ipv6>());
    ueUuStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress6(),1);

    //fucking라우팅
    Ipv6StaticRoutingHelper ipv6RoutingHelperRsu;
    Ptr<Ipv6> ipv6 = clientApp->GetNode()->GetObject<Ipv6>();
    Ptr<Ipv6StaticRouting> staticRouting = ipv6RoutingHelperRsu.GetStaticRouting(ipv6);
    uint32_t slInterfaceIndex = ipv6->GetInterfaceForDevice(ueSlNetDev.Get(0));
    staticRouting->AddNetworkRouteTo(Ipv6Address::ConvertFrom(rsuServerIpv6),
                                     Ipv6Prefix(128),
                                     Ipv6Address::ConvertFrom(temp),
                                     slInterfaceIndex);


    //인터페이스 바꾸는거 그냥 예시
    clientApp->setInterface(ueUuNetDev.Get(0),ueSlNetDev.Get(0));
    Simulator::Schedule(Seconds(10.0),&UdpClient::changeInterface, clientApp);


    // Ptr<Ipv6> ipv6 = ue->GetObject<Ipv6>();
    for (uint32_t ifIndex = 0; ifIndex < ipv6->GetNInterfaces (); ++ifIndex) {
        for (uint32_t addrIndex = 0; addrIndex < ipv6->GetNAddresses (ifIndex); ++addrIndex) {
            auto ifAddr = ipv6->GetAddress(ifIndex, addrIndex);
            std::cout << "Iface " << ifIndex
                      << ", Addr " << addrIndex
                      << ": " << ifAddr.GetAddress()
                      << "/" << ifAddr.GetPrefix()
                      << std::endl;
        }
    }
    std::cout<<"rsutoserver server ip : "<<rsuServerIpv6<<std::endl;
    std::cout<<"uetorsu rsu ip"<<temp<<std::endl;
    // main 함수 내부, 인터넷 스택 설치 이후


    // RSU 노드의 IPv6 L3 프로토콜의 "Rx" Trace Source에 콜백 함수를 연결합니다.
    // "Rx"는 IP 계층에서 패킷을 수신하는 이벤트입니다.
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(rsu->GetId()) + "/$ns3::Ipv6L3Protocol/Rx",
                                  MakeCallback(&Ipv6PacketTraceAtRsu));
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(pgw->GetId()) + "/$ns3::Ipv6L3Protocol/Rx",
                              MakeCallback(&Ipv6PacketTraceAtPgw));
    // Ptr<NetDevice> dev = ueUuNetDev.Get (0);
    // Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice> (dev);
    // Ptr<NrSpectrumPhy> spectrumPhy = ueDev->GetPhy (0)->GetSpectrumPhy ();
    // Ptr<NrInterference> interference = spectrumPhy->GetNrInterferenceCtrl();
    // interference->TraceConnectWithoutContext(
    //     "RssiPerProcessedChunk",
    //     MakeBoundCallback(&UeRssiPerProcessedChunk, spectrumPhy));

    Ptr<NetDevice> dev1 = rsuNetDev.Get (0);
    Ptr<NrUeNetDevice> ueDev1 = DynamicCast<NrUeNetDevice> (dev1);
    Ptr<NrSpectrumPhy> spectrumPhy1 = ueDev1->GetPhy (0)->GetSpectrumPhy ();
    Ptr<NrSlInterference> interference1 = spectrumPhy1->GetSlInterference();
    interference1->TraceConnectWithoutContext(
    "SlRssiPerProcessedChunk",
    MakeBoundCallback(&UeSlRssiPerProcessedChunk, spectrumPhy1));
    NS_LOG_UNCOND("main Connecting trace to Interference object at address: " << &(*interference1));

    // 위의 Ptr<...> interference1을 얻는 코드는 더 이상 필요 없습니다.
    // 아래 한 줄로 대체합니다.
    // std::string path = "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/1/SidelinkPhy/SpectrumPhy/SlInterference/SlRssiPerProcessedChunk";
    // Config::ConnectWithoutContext(
    //     path,
    //     MakeBoundCallback(&UeSlRssiPerProcessedChunk, ueDev1->GetPhy(0)->GetSpectrumPhy()));

    interference1->TraceConnectWithoutContext(
        "SlRssiPerProcessedChunk",
        MakeBoundCallback(&UeSlRssiPerProcessedChunk, spectrumPhy1));

    // Config::SetDefault("ns3::ConfigStore::Filename", StringValue("output-attributes.xml"));
    // Config::SetDefault("ns3::ConfigStore::FileFormat", StringValue("Xml"));
    // Config::SetDefault("ns3::ConfigStore::Mode", StringValue("Save"));
    // ConfigStore outputConfig;
    // outputConfig.ConfigureDefaults();
    // outputConfig.ConfigureAttributes();
    // Simulator::Schedule(
    //     Seconds(11.0),
    //     &ConfigStore::ConfigureAttributes,  // 멤버 함수 포인터
    //     &outputConfig                      // 호출할 객체 포인터
    // );


    // --- 시뮬레이션 실행 ---
    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();
}