#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HighCpuLoadExample");

int
main(int argc, char* argv[])
{
    uint32_t numNodes = 100;
    double simTime = 20.0;

    CommandLine cmd;
    cmd.AddValue("numNodes", "Number of nodes", numNodes);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(numNodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    std::vector<NetDeviceContainer> devices;
    std::vector<Ipv4InterfaceContainer> interfaces;

    for (uint32_t i = 0; i < numNodes - 1; ++i)
    {
        NodeContainer pair(nodes.Get(i), nodes.Get(i + 1));
        NetDeviceContainer devs = p2p.Install(pair);
        devices.push_back(devs);

        std::ostringstream subnet;
        subnet << "10." << i + 1 << ".1.0";
        address.SetBase(Ipv4Address(subnet.str().c_str()), "255.255.255.0");
        Ipv4InterfaceContainer iface = address.Assign(devs);
        interfaces.push_back(iface);
    }

    for (uint32_t i = 0; i < numNodes - 1; ++i)
    {
        uint16_t port = 9000 + i;
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          Address(InetSocketAddress(interfaces[i].GetAddress(1), port)));
        onoff.SetAttribute("DataRate", StringValue("5Mbps"));
        onoff.SetAttribute("PacketSize", UintegerValue(1400));

        ApplicationContainer app = onoff.Install(nodes.Get(i));
        app.Start(Seconds(1.0));
        app.Stop(Seconds(simTime - 1));

        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              Address(InetSocketAddress(Ipv4Address::GetAny(), port)));
        ApplicationContainer sinkApp = sink.Install(nodes.Get(i + 1));
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(simTime));
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
