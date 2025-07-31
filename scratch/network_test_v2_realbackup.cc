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

struct WaypointData
{
    double time;
    double x;
    double y;
    double z;
    double speed;
};


void
UeMeasCallback(uint16_t cellId, uint16_t IMSI, uint16_t RNTI, double RSRP, uint8_t BWPId)
{
    std::cout << "📶 [Meas] cellId=" << cellId << " IMSI=" << IMSI << " BWPId=" << BWPId
              << "  RNTI=" << RNTI << " RSRP=" << RSRP << " dB\n";
}

#include <ns3/spectrum-model.h>
#include <ns3/spectrum-value.h>

#include <cmath>

#include "ns3/spectrum-phy.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/net-device.h"
#include "ns3/node.h"



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



void
printRssi(Ptr<const SpectrumValue> psd)
{
    // 스펙트럼 모델을 통해 sub‑carrier 간격(Hz)을 구함
    Ptr<const SpectrumModel> sm = psd->GetSpectrumModel();
    double binWidth = sm->Begin()->fh - sm->Begin()->fl;

    // PSD[W/Hz] × binWidth → 각 bin 전력[W], 모두 합산
    double powerW = Sum(*psd * binWidth);

    // W → dBm 변환
    double rssiDbm = 10.0 * std::log10(powerW * 1e3);

    std::cout << "[printRssi] RSSI = " << rssiDbm << " dBm" << std::endl << std::endl;
}

void
psdCallback(const SfnSf& sfnSf,
            Ptr<const SpectrumValue> v,
            const Time& phyTime,
            uint16_t rnti,
            uint64_t imsi,
            uint16_t bwpId,
            uint16_t cellId)
{
    // 1) 시뮬레이션 현재 시각(Time)과 SFN/Subframe
    std::cout << "[PSD Callback] SimTime="
              << Simulator::Now().GetSeconds()
              //<< "s, SFN=" << sfnSf.m_sfn << ", SF=" << sfnSf.m_sf
              << ", PhyTime=" << phyTime.GetSeconds() << "s" << std::endl;

    // 2) 식별자 정보
    std::cout << "  RNTI=" << rnti << ", IMSI=" << imsi << ", BWP=" << bwpId
              << ", CellId=" << cellId << std::endl;

    // 3) PSD 벡터 값 출력 (Hz당 W 단위)
    std::cout << "  PSD values (W/Hz):";
    uint32_t idx = 0;
    for (auto it = v->ConstValuesBegin(); it != v->ConstValuesEnd(); ++it, ++idx)
    {
        // 8개 단위로 줄 바꿈
        if (idx % 8 == 0)
            std::cout << std::endl << "   ";
        std::cout << *it;
        if (it + 1 != v->ConstValuesEnd())
            std::cout << "\t";
    }
    printRssi(v);
}

void
RxDataCallback(const SfnSf& sfnSf,
               Ptr<const SpectrumValue> rxPsd,
               const Time& duration,
               uint16_t bwpId,
               uint16_t cellId)
{
    // 1) 스펙트럼 모델에서 주파수 분할폭(Hz) 가져오기
    Ptr<const SpectrumModel> sm = rxPsd->GetSpectrumModel();
    double binWidth = sm->Begin()->fh - sm->Begin()->fl; // 예: subcarrier 간격

    // 2) PSD 벡터 × binWidth → 각 bin별 W 단위 전력 → 모두 합산
    double powerW = Sum(*rxPsd * binWidth);

    // 3) W → dBm 변환: 10·log10(powerW·1000)
    double rssiDbm = 10.0 * std::log10(powerW * 1e3);

    std::cout << "RSSI = " << rssiDbm << " dBm (BWP " << bwpId << ", Cell " << cellId << ")\n";

    // 여기서 강화학습 환경으로 넘기시면 됩니다.
}

// UE의 위치와 속도를 출력하는 함수
void
PrintUeInfo(Ptr<Node> ueNode)
{
    Ptr<MobilityModel> mob = ueNode->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    Vector vel = mob->GetVelocity();

    NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << "s");
    NS_LOG_UNCOND("UE Position: x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z);
    NS_LOG_UNCOND("UE Velocity: x=" << vel.x << ", y=" << vel.y << ", z=" << vel.z << " (m/s)");
    Simulator::Schedule(Seconds(1.0), &PrintUeInfo, ueNode);
}

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

// RSU에 설치할 UDP 릴레이 애플리케이션
class UdpRelay : public Application
{
  public:
    static TypeId GetTypeId();

    UdpRelay()
    {
    }

    ~UdpRelay() override
    {
    }

  private:
    void StartApplication() override;
    void StopApplication() override;
    void HandleRead(Ptr<Socket> socket); // UE로부터 패킷을 수신하는 콜백 함수

    Ptr<Socket> m_inSocket;  // UE로부터 수신용 소켓
    Ptr<Socket> m_outSocket; // 서버로 송신용 소켓

    uint16_t m_inPort;    // 수신 포트 (UE가 여기로 보냄)
    Address m_outAddress; // 최종 목적지 서버 주소
    uint16_t m_outPort;   // 최종 목적지 서버 포트
};

NS_OBJECT_ENSURE_REGISTERED(UdpRelay);

TypeId
UdpRelay::GetTypeId()
{
    static TypeId tid =
        TypeId("UdpRelay")
            .SetParent<Application>()
            .SetGroupName("Applications")
            .AddConstructor<UdpRelay>()
            .AddAttribute("InPort",
                          "Port on which we listen for incoming packets from UEs.",
                          UintegerValue(8000), // 기본 수신 포트
                          MakeUintegerAccessor(&UdpRelay::m_inPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("OutAddress",
                          "The destination Address of the outbound packets to the server.",
                          AddressValue(),
                          MakeAddressAccessor(&UdpRelay::m_outAddress),
                          MakeAddressChecker())
            .AddAttribute("OutPort",
                          "The destination port of the outbound packets to the server.",
                          UintegerValue(5000), // 서버의 포트
                          MakeUintegerAccessor(&UdpRelay::m_outPort),
                          MakeUintegerChecker<uint16_t>());
    return tid;
}

void
UdpRelay::StartApplication()
{
    // 1. UE로부터 패킷을 받을 소켓(수신용) 설정
    m_inSocket = Socket::CreateSocket(GetNode(), TypeId::LookupByName("ns3::UdpSocketFactory"));
    Inet6SocketAddress local = Inet6SocketAddress(Ipv6Address::GetAny(), m_inPort);
    if (m_inSocket->Bind(local) == -1)
    {
        NS_FATAL_ERROR("UdpRelay: Failed to bind In-Socket");
    }
    m_inSocket->SetRecvCallback(MakeCallback(&UdpRelay::HandleRead, this));

    // 2. 서버로 패킷을 보낼 소켓(송신용) 설정
    m_outSocket = Socket::CreateSocket(GetNode(), TypeId::LookupByName("ns3::UdpSocketFactory"));
    if (m_outSocket->Connect(
            Inet6SocketAddress(Ipv6Address::ConvertFrom(m_outAddress), m_outPort)) == -1)
    {
        NS_FATAL_ERROR("UdpRelay: Failed to connect Out-Socket to server");
    }

    std::cout << "RSU Relay Application Started. Listening on port " << m_inPort << ", Relaying to "
              << m_outAddress << ":" << m_outPort << std::endl;
}

void
UdpRelay::StopApplication()
{
    if (m_inSocket)
    {
        m_inSocket->Close();
        m_inSocket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
    if (m_outSocket)
    {
        m_outSocket->Close();
    }
}

void
UdpRelay::HandleRead(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
        if (packet->GetSize() == 0)
        {
            break;
        }

        std::cout << "RSU Relay: Received " << packet->GetSize() << " bytes from "
                  << Inet6SocketAddress::ConvertFrom(from).GetIpv6() << std::endl;

        // 받은 패킷 그대로 서버로 전달 (송신용 소켓 사용)
        m_outSocket->Send(packet->Copy());

        std::cout << "RSU Relay: Relayed packet to server." << std::endl;
    }
}

// udp 서버 설정===============================================================
struct clientInfo
{
    Address address;
    uint32_t lastSequenceNum;
    Time connectionTime;
    uint32_t RTT;
    float_t packetLossRate;
    uint64_t totalBytesReceived;
};

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
        TypeId("UdpServerk")
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
                          << "port: " << InetSocketAddress::ConvertFrom(from).GetPort()
                          << " Sequence Number: " << currentSequenceNumber
                          << " Uid: " << packet->GetUid() << " TXtime: " << seqTs.GetTs()
                          << " RXtime: " << Simulator::Now()
                          << " Delay: " << Simulator::Now() - seqTs.GetTs() << std::endl;
            }
            else if (Inet6SocketAddress::IsMatchingType(from))
            {
                std::cout << "TraceDelay: RX " << receivedSize << " bytes from "
                          << Inet6SocketAddress::ConvertFrom(from).GetIpv6()
                          << " port: " << Inet6SocketAddress::ConvertFrom(from).GetPort()
                          << " Sequence Number: " << currentSequenceNumber
                          << " Uid: " << packet->GetUid() << " TXtime: " << seqTs.GetTs()
                          << " RXtime: " << Simulator::Now()
                          << " Delay: " << Simulator::Now() - seqTs.GetTs() << std::endl;
            }

            m_lossCounter.NotifyReceived(currentSequenceNumber);
            m_received++;

            SendPacket(clientId, "good");
            std::cout << "sent to client - clientId : " << clientId << std::endl;
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
    // 1. CSV 파일 읽기
    // ==========================================================
    std::vector<WaypointData> waypoints;
    std::string csvFileName = "/home/loapp/ns-simulator/ns-3-dev/scratch/final_3d_trace.csv";
    std::ifstream file(csvFileName);

    if (!file.is_open())
    {
        std::cout << "Could not open CSV file: " << csvFileName << std::endl;
        return 2;
    }

    std::string line;
    // 헤더 라인 무시
    std::getline(file, line);

    double maxTime = 0.0;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        WaypointData data;

        // 각 열 파싱
        std::getline(ss, value, ','); // time
        data.time = std::stod(value);
        std::getline(ss, value, ','); // vehicle_id (skip)
        std::getline(ss, value, ','); // x
        data.x = std::stod(value);
        std::getline(ss, value, ','); // y
        data.y = std::stod(value);
        std::getline(ss, value, ','); // z
        data.z = std::stod(value);
        std::getline(ss, value, ','); // speed
        data.speed = std::stod(value);
        std::getline(ss, value, ','); // lon (skip)
        std::getline(ss, value, ','); // lat (skip)

        waypoints.push_back(data);
        if (data.time > maxTime)
        {
            maxTime = data.time;
        }
    }
    file.close();
    NS_LOG_UNCOND("Successfully read " << waypoints.size() << " waypoints from CSV.");

    // ns3 세팅 시작
    Time simTime = Seconds(93);

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
    mobility.Install(gnbNodeContainer);
    gnbNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(1500, 4288, 80.0));
    mobility.Install(rsuNodeContainer);
    rsuNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(294.0, 4350.0, 70.0));
    mobility.Install(serverNodeContainer);
    serverNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(
        Vector(1900.0, 3800.0, 60.0));

    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(ueNodeContainer);
    ueNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(299.89, 4315.03, 59));
    Ptr<WaypointMobilityModel> ueMobility =
        ueNodeContainer.Get(0)->GetObject<WaypointMobilityModel>();

    // 읽어온 CSV 데이터를 Waypoint로 추가
    for (const auto& data : waypoints)
    {
        Waypoint waypoint(Seconds(data.time), Vector(data.x, data.y, data.z));
        ueMobility->AddWaypoint(waypoint);
    }

    // gnb bwp 설정

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    double gNbFrequencyBand = 3.5e9; // 3.5GHz
    double gNbBandwidthBand = 1e8;   // 100MHz
    uint8_t gNbnumContiguousCc = 1;  // 100MHz 안에 몇개의 CC가 들어가 있는지
    uint16_t gNbNumerology = 1;
    double gNbTxPower = 43.0; // 단위dBm

    CcBwpCreator gNbCcBwpCreators;
    OperationBandInfo gNbBand;

    CcBwpCreator::SimpleOperationBandConf gNbBandConf(gNbFrequencyBand,
                                                      gNbBandwidthBand,
                                                      gNbnumContiguousCc,
                                                      BandwidthPartInfo::UMi_StreetCanyon_nLoS); //
    gNbBandConf.m_numBwp = 1; // 1 BWP per CC
    gNbBand = gNbCcBwpCreators.CreateOperationBandContiguousCc(gNbBandConf);

    nrHelper->InitializeOperationBand(&gNbBand);
    BandwidthPartInfoPtrVector gNbBwp;
    gNbBwp = CcBwpCreator::GetAllBwps({gNbBand});

    std::vector<ObjectFactory> macUuFactory;
    ObjectFactory uufactory;
    uufactory.SetTypeId(NrUeMac::GetTypeId());
    macUuFactory.push_back(uufactory);
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodeContainer, gNbBwp);

    // Ptr<OhBuildingsPropagationLossModel> bldgLoss =
    // CreateObject<OhBuildingsPropagationLossModel>();
    //
    // bldgLoss->SetAttribute("ShadowSigmaOutdoor", DoubleValue(6.0));
    // bldgLoss->SetAttribute("ShadowSigmaIndoor", DoubleValue(7.0));
    // bldgLoss->SetAttribute("ShadowSigmaExtWalls", DoubleValue(5.0));
    // bldgLoss->SetAttribute("InternalWallLoss", DoubleValue(5.0));
    //
    // // NRHelper에 적용
    // nrHelper->SetPathlossAttribute("PathlossModel", PointerValue(bldgLoss));

    // 안테나 설정
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_PREMIUM",
                                                 UintegerValue(0)); // bwp하나만 한거

    std::string pattern = "DL|DL|DL|DL|UL|UL|UL|UL|UL|UL|";
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(gNbNumerology));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(gNbTxPower));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("Pattern", StringValue(pattern));

    // 설정 적용
    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    // ue uu 설정
    NetDeviceContainer ueUuNetDev =
        nrHelper->InstallUeDevice(ueNodeContainer, gNbBwp, macUuFactory);

    double ueTxPower = 23.0;
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->GetUePhy(ueUuNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(ueTxPower));

    DynamicCast<NrUeNetDevice>(ueUuNetDev.Get(0))->UpdateConfig();

    // RSU, SL 기본 설정=======================================================
    double RsuFrequencyBand = 5.89e9;
    uint16_t RsuBandwidthBand = 400;
    uint8_t RsunumContiguousCc = 1;
    uint16_t RsuNumerology = 1;
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
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
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

    NetDeviceContainer rsuNetDev =
        nrHelper->InstallUeDevice(rsuNodeContainer, RsuBwp, macSlFactory);

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
    tddUlDlConfigCommon.tddPattern = "DL|DL|DL|DL|UL|UL|UL|UL|UL|UL|";

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

    // ue안테나 설정
    NetDeviceContainer ueSlNetDev =
        nrHelper->InstallUeDevice(ueNodeContainer, RsuBwp, macSlFactory);

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->GetUePhy(ueSlNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(ueTxPower));

    DynamicCast<NrUeNetDevice>(ueSlNetDev.Get(0))->UpdateConfig(); // todo: obu sl

    NetDeviceContainer SlNetDev;
    SlNetDev.Add(ueSlNetDev);
    SlNetDev.Add(rsuNetDev);

    // 진짜 시작todo:
    // ===============================================================================

    NodeContainer nodes(server);
    NodeContainer routers(pgw, rsu);

    // 여기서 p2p를 쓸지 csma를 쓸지 결정해야할듯
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("10ms"));
    NetDeviceContainer pgwToServerNetDev = p2ph.Install(pgw, server);
    NetDeviceContainer rsuToServerNetDev = p2ph.Install(rsu, server);
    p2ph.EnablePcap("rsu-to-server", rsuToServerNetDev.Get(0), true);

    // 인터넷 설정
    InternetStackHelper internet;
    internet.SetIpv4StackInstall(false);
    internet.Install(ueNodeContainer);
    internet.Install(nodes);

    InternetStackHelper internetv6;
    internetv6.SetIpv4StackInstall(false);
    internetv6.Install(routers);

    // ip설정
    Ipv6AddressHelper ipv6h;

    // rsu server
    ipv6h.SetBase("fd00:1::", Ipv6Prefix(64));
    Ipv6InterfaceContainer iic1 = ipv6h.Assign(rsuToServerNetDev);
    iic1.SetForwarding(0, true);

    Ipv6Address rsuServerIpv6 = iic1.GetAddress(1, 1);

    // pgw server
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

    // sidelink 무선 베어러 설정
    Ptr<LteSlTft> tft;
    uint32_t dstL2Id = 255;
    Time delayBudget = Seconds(0);

    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id = dstL2Id;
    slInfo.m_rri = MilliSeconds(100);
    slInfo.m_pdb = delayBudget;
    slInfo.m_harqEnabled = true;
    Ipv6Address groupAddress6("ff0e::1");
    tft = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, temp, slInfo);
    nrSlHelper->ActivateNrSlBearer(Seconds(0.0), ueSlNetDev, tft);

    tft = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, temp, slInfo);
    nrSlHelper->ActivateNrSlBearer(Seconds(0.0), rsuNetDev, tft);

    // // Uu PHY에서 RSRP 측정 콜백 연결 (gNb와의 Uu 통신)
    // Ptr<NrUeNetDevice> ueUuDev = DynamicCast<NrUeNetDevice>(ueUuNetDev.Get(0));
    // // Get the first PHY (BWP) from the Uu NetDevice
    // Ptr<NrUePhy> ueUuPhy = ueUuDev->GetPhy(0);
    // ueUuPhy->TraceConnectWithoutContext("ReportRsrp", MakeCallback(&UeMeasCallback));

    //  todo: 앱설치 =============================================================
    ApplicationContainer clientAppContainer;
    ApplicationContainer serverAppContainer;

    u_int16_t serverPort = 5000;
    u_int16_t rsuPort = 6000;

    // u_int16_t clientPort = 4000;
    // AdaptiveUdpClientk adaptiveUdpClientk(rsuIpv6,serverIpv6, serverPort);
    // adaptiveUdpClientk.SetAttribute("Interval",TimeValue(Seconds(3)));

    Ptr<UdpServerk> serverApp = CreateObject<UdpServerk>();
    server->AddApplication(serverApp);
    serverApp->SetAttribute("Port", UintegerValue(serverPort));
    serverApp->SetStartTime(Seconds(1.0));
    serverApp->SetStopTime(simTime);

    Ptr<UdpClient> clientApp = CreateObject<UdpClient>();
    clientApp->SetAttribute("MaxPackets", UintegerValue(3));
    clientApp->SetAttribute("Interval", TimeValue(Seconds(1.0)));
    clientApp->SetAttribute("PacketSize", UintegerValue(100));

    ue->AddApplication(clientApp);
    clientApp->SetStartTime(Seconds(40.0));
    clientApp->SetStopTime(simTime);
    // todo:여기다가포트랑 주소 넣어야함
    clientApp->setAddressSlUu(gnbServerIpv6, serverPort, temp, rsuPort);

    // rsu앱 설정
    Ptr<UdpRelay> relayApp = CreateObject<UdpRelay>();
    rsu->AddApplication(relayApp);
    relayApp->SetAttribute("InPort", UintegerValue(rsuPort));          // 수신 포트 설정
    relayApp->SetAttribute("OutAddress", AddressValue(rsuServerIpv6)); // 최종 서버 주소 설정
    relayApp->SetAttribute("OutPort", UintegerValue(serverPort));      // 최종 서버 포트 설정
    relayApp->SetStartTime(Seconds(1.0));
    relayApp->SetStopTime(simTime);


    // ue pgw 라우팅
    Ipv6StaticRoutingHelper ipv6RoutingHelper;
    Ptr<Ipv6StaticRouting> ueUuStaticRouting =
        ipv6RoutingHelper.GetStaticRouting(ue->GetObject<Ipv6>());
    ueUuStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress6(), 1);

    // fucking라우팅
    //  Ipv6StaticRoutingHelper ipv6RoutingHelperRsu;
    Ptr<Ipv6> ipv6 = clientApp->GetNode()->GetObject<Ipv6>();
    // Ptr<Ipv6StaticRouting> staticRouting = ipv6RoutingHelperRsu.GetStaticRouting(ipv6);
    // uint32_t slInterfaceIndex = ipv6->GetInterfaceForDevice(ueSlNetDev.Get(0));
    // staticRouting->AddNetworkRouteTo(Ipv6Address::ConvertFrom(rsuServerIpv6),
    //                                  Ipv6Prefix(128),
    //                                  Ipv6Address::ConvertFrom(temp),
    //                                  slInterfaceIndex);

    // 인터페이스 바꾸는거 그냥 예시
    clientApp->setInterface(ueUuNetDev.Get(0), ueSlNetDev.Get(0));
    Simulator::Schedule(Seconds(41.0), &UdpClient::changeInterface, clientApp);

    // Ptr<Ipv6> ipv6 = ue->GetObject<Ipv6>();
    for (uint32_t ifIndex = 0; ifIndex < ipv6->GetNInterfaces(); ++ifIndex)
    {
        for (uint32_t addrIndex = 0; addrIndex < ipv6->GetNAddresses(ifIndex); ++addrIndex)
        {
            auto ifAddr = ipv6->GetAddress(ifIndex, addrIndex);
            std::cout << "Iface " << ifIndex << ", Addr " << addrIndex << ": "
                      << ifAddr.GetAddress() << "/" << ifAddr.GetPrefix() << std::endl;
        }
    }
    std::cout << "rsutoserver server ip : " << rsuServerIpv6 << std::endl;
    std::cout << "uetorsu rsu ip" << temp << std::endl;
    // main 함수 내부, 인터넷 스택 설치 이후

    // RSU 노드의 IPv6 L3 프로토콜의 "Rx" Trace Source에 콜백 함수를 연결합니다.
    // "Rx"는 IP 계층에서 패킷을 수신하는 이벤트입니다.
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(rsu->GetId()) +
                                      "/$ns3::Ipv6L3Protocol/Rx",
                                  MakeCallback(&Ipv6PacketTraceAtRsu));
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(pgw->GetId()) +
                                      "/$ns3::Ipv6L3Protocol/Rx",
                                  MakeCallback(&Ipv6PacketTraceAtPgw));
    //   Config::ConnectWithoutContext (
    // "/NodeList/*/DeviceList/*/"
    // "$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/"
    // "NrUePhy/SpectrumPhy/RxDataTrace",
    // MakeCallback(&RxDataCallback)); // 이건 전송 받을 때만 해서 안됨

    // Config::ConnectWithoutContext(
    //     "/NodeList/*/DeviceList/*/$ns3::NrNetDevice/"
    //     "$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/ReportPowerSpectralDensity",
    //     MakeCallback(&psdCallback));

    Simulator::Schedule(Seconds(0.0), &PrintUeInfo, ueNodeContainer.Get(0));
    Ptr<NetDevice> dev = ueUuNetDev.Get (0);
    Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice> (dev);
    Ptr<NrSpectrumPhy> spectrumPhy = ueDev->GetPhy (0)->GetSpectrumPhy ();
    Ptr<NrInterference> interference = spectrumPhy->GetNrInterferenceCtrl();
    interference->TraceConnectWithoutContext(
        "RssiPerProcessedChunk",
        MakeBoundCallback(&UeRssiPerProcessedChunk, spectrumPhy));

    Ptr<NetDevice> dev1 = ueSlNetDev.Get (0);
    Ptr<NrUeNetDevice> ueDev1 = DynamicCast<NrUeNetDevice> (dev1);
    Ptr<NrSpectrumPhy> spectrumPhy1 = ueDev1->GetPhy (0)->GetSpectrumPhy ();
    Ptr<NrSlInterference> interference1 = spectrumPhy1->GetSlInterference();
    interference1->TraceConnectWithoutContext(
        "SlRssiPerProcessedChunk",
        MakeBoundCallback(&UeSlRssiPerProcessedChunk, spectrumPhy1));

    // --- 시뮬레이션 실행 ---
    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();
}