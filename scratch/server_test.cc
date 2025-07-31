#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-helper.h"

using namespace ns3;

int
main(int argc, char* argv[])
{
    NodeContainer nodes;
    nodes.Create(1);

    InternetStackHelper internet;
    internet.Install(nodes);
}