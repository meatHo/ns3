// V2I Streaming with Adaptive Interface Selection - ns-3 Scenario

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-sl-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/udp-socket.h"

using namespace ns3;

class AdaptiveUdpClient : public Application
{
  public:
    AdaptiveUdpClient();
    virtual ~AdaptiveUdpClient();
    void Setup(Address pc5Address, Address uuAddress, Time interval);

  private:
    virtual void StartApplication();
    virtual void StopApplication();
    void ScheduleNextTx();
    void SendPacket();
    double EstimateThroughput(Ptr<Socket> socket);

    Ptr<Socket> m_pc5Socket;
    Ptr<Socket> m_uuSocket;
    Ptr<Socket> m_currentSocket;
    Address m_pc5Address;
    Address m_uuAddress;
    Time m_interval;
    EventId m_sendEvent;
    uint32_t m_packetSize;
};

AdaptiveUdpClient::AdaptiveUdpClient()
    : m_packetSize(1024)
{
}

AdaptiveUdpClient::~AdaptiveUdpClient()
{
}

void
AdaptiveUdpClient::Setup(Address pc5Address, Address uuAddress, Time interval)
{
    m_pc5Address = pc5Address;
    m_uuAddress = uuAddress;
    m_interval = interval;
}

void
AdaptiveUdpClient::StartApplication()
{
    m_pc5Socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_uuSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());

    m_pc5Socket->Connect(m_pc5Address);
    m_uuSocket->Connect(m_uuAddress);

    m_currentSocket = m_pc5Socket; // 초기 설정
    ScheduleNextTx();
}

void
AdaptiveUdpClient::StopApplication()
{
    if (m_pc5Socket)
        m_pc5Socket->Close();
    if (m_uuSocket)
        m_uuSocket->Close();
    Simulator::Cancel(m_sendEvent);
}

void
AdaptiveUdpClient::ScheduleNextTx()
{
    double pc5Rate = EstimateThroughput(m_pc5Socket);
    double uuRate = EstimateThroughput(m_uuSocket);

    m_currentSocket = (pc5Rate > uuRate) ? m_pc5Socket : m_uuSocket;
    SendPacket();
    m_sendEvent = Simulator::Schedule(m_interval, &AdaptiveUdpClient::ScheduleNextTx, this);
}

void
AdaptiveUdpClient::SendPacket()
{
    Ptr<Packet> packet = Create<Packet>(m_packetSize);
    m_currentSocket->Send(packet);
}

double
AdaptiveUdpClient::EstimateThroughput(Ptr<Socket> socket)
{
    // 간단한 거리 기반 모델 (가정)
    Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    double distance = pos.x; // RSU 기준으로 x축 거리
    if (socket == m_pc5Socket)
    {
        if (distance < 50)
            return 1000;
        else if (distance < 100)
            return 800;
        else if (distance < 150)
            return 400;
        else if (distance < 200)
            return 100;
        else
            return 0;
    }
    return 100; // gNB는 고정 100 Mbps
}

int
main(int argc, char* argv[])
{
    Time simTime = Seconds(20);
    CommandLine cmd;
    cmd.Parse(argc, argv);

    NodeContainer ueNode, rsuNode, gnbNode, cloudNode;
    ueNode.Create(1);
    rsuNode.Create(1);
    gnbNode.Create(1);
    cloudNode.Create(1);

    InternetStackHelper internet;
    internet.InstallAll();

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNode);
    ueNode.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(
        Vector(-300.0, 0.0, 0.0));
    ueNode.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(20.0, 0.0, 0.0));

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(rsuNode);
    rsuNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.0));
    mobility.Install(gnbNode);
    gnbNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(1000.0, 0.0, 0.0));
    mobility.Install(cloudNode);
    cloudNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(2000.0, 0.0, 0.0));

    PointToPointHelper p2pRsu, p2pGnb;
    p2pRsu.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    p2pRsu.SetChannelAttribute("Delay", StringValue("2ms"));
    NetDeviceContainer rsuToCloud = p2pRsu.Install(rsuNode.Get(0), cloudNode.Get(0));
    p2pGnb.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    p2pGnb.SetChannelAttribute("Delay", StringValue("5ms"));
    NetDeviceContainer gnbToCloud = p2pGnb.Install(gnbNode.Get(0), cloudNode.Get(0));

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer rsuIf = ipv4.Assign(rsuToCloud);
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer gnbIf = ipv4.Assign(gnbToCloud);

    Ptr<NrV2xHelper> v2xHelper = CreateObject<NrV2xHelper>();
    v2xHelper->SetPc5Mode(NrV2xHelper::Pc5Mode::MODE1);
    v2xHelper->InstallSidelink(ueNode, rsuNode);

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNode);
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNode);
    nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);

    UdpServerHelper server(5000);
    server.Install(cloudNode.Get(0));

    Ptr<AdaptiveUdpClient> app = CreateObject<AdaptiveUdpClient>();
    Address pc5Addr = InetSocketAddress(rsuIf.GetAddress(1), 5000);
    Address uuAddr = InetSocketAddress(gnbIf.GetAddress(1), 5000);
    app->Setup(pc5Addr, uuAddr, MilliSeconds(10));
    ueNode.Get(0)->AddApplication(app);
    app->SetStartTime(Seconds(2.0));
    app->SetStopTime(simTime);

    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
