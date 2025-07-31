#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VideoStreaming");

class VideoStreamServer : public Application
{
  public:
    VideoStreamServer()
    {
    }

    virtual ~VideoStreamServer()
    {
    }

    void Setup(Ptr<Socket> socket, Address address, uint32_t pktSize, uint32_t fps)
    {
        m_socket = socket;
        m_peer = address;
        m_pktSize = pktSize;
        m_interval = Seconds(1.0 / fps);
    }

  private:
    virtual void StartApplication() override
    {
        m_socket->Bind();
        m_sendEvent = Simulator::Schedule(Seconds(0.0), &VideoStreamServer::SendPacket, this);
    }

    virtual void StopApplication() override
    {
        Simulator::Cancel(m_sendEvent);
        m_socket->Close();
    }

    void SendPacket()
    {
        Ptr<Packet> packet = Create<Packet>(m_pktSize);
        m_socket->SendTo(packet, 0, m_peer);
        m_sendEvent = Simulator::Schedule(m_interval, &VideoStreamServer::SendPacket, this);
    }

    Ptr<Socket> m_socket;
    Address m_peer;
    uint32_t m_pktSize;
    Time m_interval;
    EventId m_sendEvent;
};

class VideoStreamClient : public Application
{
  public:
    VideoStreamClient()
    {
    }

  private:
    virtual void StartApplication() override
    {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9999));
        m_socket->SetRecvCallback(MakeCallback(&VideoStreamClient::ReceivePacket, this));
    }

    void ReceivePacket(Ptr<Socket> socket)
    {
        while (Ptr<Packet> packet = socket->Recv())
        {
            NS_LOG_INFO("Received " << packet->GetSize() << " bytes at "
                                    << Simulator::Now().GetSeconds() << "s");
        }
    }

    Ptr<Socket> m_socket;
};

int
main(int argc, char* argv[])
{
    LogComponentEnable("VideoStreaming", LOG_LEVEL_INFO);

    NodeContainer nodes;
    nodes.Create(2);

    InternetStackHelper internet;
    internet.Install(nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));

    NetDeviceContainer devices = p2p.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    Ptr<Socket> serverSocket = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    Ptr<VideoStreamServer> serverApp = CreateObject<VideoStreamServer>();
    serverApp->Setup(serverSocket,
                     InetSocketAddress(interfaces.GetAddress(1), 9999),
                     1024,
                     30); // 1KB 패킷, 30fps
    nodes.Get(0)->AddApplication(serverApp);
    serverApp->SetStartTime(Seconds(1.0));
    serverApp->SetStopTime(Seconds(10.0));

    Ptr<VideoStreamClient> clientApp = CreateObject<VideoStreamClient>();
    nodes.Get(1)->AddApplication(clientApp);
    clientApp->SetStartTime(Seconds(0.0));
    clientApp->SetStopTime(Seconds(11.0));

    Simulator::Stop(Seconds(12.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}