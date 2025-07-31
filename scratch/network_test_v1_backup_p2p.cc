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
            std::cout << "sent to client : "<<clientId << std::endl;
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
    NodeContainer routerNodeContainer;
    routerNodeContainer.Create(1);

    Ptr<Node> pgw = epcHelper->GetPgwNode(); // ipv4, ipv6 둘다 설치되어 있음. 듀얼스택
    Ptr<Node> server = serverNodeContainer.Get(0);
    Ptr<Node> rsu = rsuNodeContainer.Get(0);
    Ptr<Node> gnb = gnbNodeContainer.Get(0);
    Ptr<Node> ue = ueNodeContainer.Get(0);
    Ptr<Node> router = routerNodeContainer.Get(0);

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
    mobility.Install(routerNodeContainer);
    routerNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(100.0, 0.0, 0.0));

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

    // 진짜 시작todo:
    // ===============================================================================
    // NodeContainer net1(rsu, router);
    // NodeContainer net2(router, server);
    // NodeContainer net3(pgw, router);
    NodeContainer nodes(rsu, server);
    NodeContainer routers(pgw,router);
    //
    // CsmaHelper csma;
    // csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
    // csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));
    // NetDeviceContainer ndc1 = csma.Install(net1);
    // NetDeviceContainer ndc2 = csma.Install(net2);
    // NetDeviceContainer ndc3 = csma.Install(net3);
    //
    RipNgHelper ripngHelper;
    //ripngHelper.ExcludeInterface()


    Ipv6ListRoutingHelper listRH;
    listRH.Add(ripngHelper, 0);
    // 여기서 p2p를 쓸지 csma를 쓸지 결정해야할듯
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("10ms"));
    NetDeviceContainer pgwToRouterNetDev = p2ph.Install(pgw, router);
    NetDeviceContainer rsuToRouterNetDev = p2ph.Install(rsu, router);
    NetDeviceContainer routerToServerNetDev = p2ph.Install(router, server);



    //인터넷 설정
    InternetStackHelper internet;
    internet.SetIpv4StackInstall(false);
    internet.Install(ueNodeContainer);
    internet.Install(nodes);

    InternetStackHelper internetv6;
    internetv6.SetIpv4StackInstall(false);
    internetv6.SetRoutingHelper(listRH);
    internetv6.Install(routers);



    // ip설정
    Ipv6AddressHelper ipv6h;

    //rsu router
    ipv6h.SetBase("fd00:1::", Ipv6Prefix(64));
    Ipv6InterfaceContainer iic1 = ipv6h.Assign(rsuToRouterNetDev);
    iic1.SetForwarding(1, true);
    iic1.SetDefaultRouteInAllNodes(1);

    //router server
    ipv6h.SetBase("fd00:2::", Ipv6Prefix(64));
    Ipv6InterfaceContainer iic2 = ipv6h.Assign(routerToServerNetDev);
    iic2.SetForwarding(0, true);
    iic2.SetDefaultRouteInAllNodes(0);
    Ipv6Address serverIpv6 = iic2.GetAddress(1, 1);

    //pgw router
    ipv6h.SetBase("fd00:3::", Ipv6Prefix(64));
    Ipv6InterfaceContainer iic3 = ipv6h.Assign(pgwToRouterNetDev);
    iic3.SetForwarding(0, true);
    iic3.SetForwarding(1, true);
    iic3.SetDefaultRouteInAllNodes(1);

    Ipv6InterfaceContainer ueUuIface = epcHelper->AssignUeIpv6Address(ueUuNetDev);
    nrHelper->AttachToClosestEnb(ueUuNetDev, gnbNetDev); // 이거는 eps베어러 생성 기지국과 연결해줌

    //ue pgw 라우팅
    Ipv6StaticRoutingHelper ipv6RoutingHelper;
    Ptr<Ipv6StaticRouting> ueStaticRouting = ipv6RoutingHelper.GetStaticRouting(ue->GetObject<Ipv6>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress6(),1);


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
    UdpClientHelper ueClient(serverIpv6, serverPort);
    ueClient.SetAttribute("MaxPackets", UintegerValue(3));      // 3개의 패킷을
    ueClient.SetAttribute("Interval", TimeValue(Seconds(1.0))); // 1초 간격으로
    ueClient.SetAttribute("PacketSize", UintegerValue(100));

    ApplicationContainer clientAppsUe = ueClient.Install(ue);
    clientAppsUe.Start(Seconds(9.0));
    clientAppsUe.Stop(simTime);

    // **시나리오 2: RSU 노드가 서버로 메시지 전송**
    // UdpClientHelper rsuClient(serverIpv6, serverPort);
    // rsuClient.SetAttribute("MaxPackets", UintegerValue(2));      // 2개의 패킷을
    // rsuClient.SetAttribute("Interval", TimeValue(Seconds(1.0))); // 1초 간격으로
    // rsuClient.SetAttribute("PacketSize", UintegerValue(150));
    //
    // ApplicationContainer clientAppsRsu = rsuClient.Install(rsu);
    // clientAppsRsu.Start(Seconds(7.0));
    // clientAppsRsu.Stop(simTime);

    //인터페이스 바꾸는거 그냥 예시
    Ptr<Application> tempApp = clientAppsUe.Get(0);
    Ptr<UdpClient> ueRealClient = DynamicCast<UdpClient>(tempApp);
    Simulator::Schedule(Seconds(10.0),&UdpClient::changeInterface, ueRealClient);

    Ptr<Ipv6> ipv6 = ue->GetObject<Ipv6>();
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


    // --- 시뮬레이션 실행 ---
    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();
}