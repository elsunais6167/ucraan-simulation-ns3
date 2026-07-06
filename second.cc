/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Specifies the open-source license under which this code is distributed (GNU General Public License v2).
 */

// Include necessary ns-3 modules. 
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
// Includes models for Carrier Sense Multiple Access (CSMA), which simulates Ethernet-like LANs.
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
// Includes the helper to automatically build routing tables across multiple subnets.
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

// Default Network Topology
//
//       10.1.1.0
// n0 -------------- n1   n2   n3   n4
//    point-to-point  |    |    |    |
//                    ================
//                      LAN 10.1.2.0
//
// Note: n1 acts as a router. It has two network interfaces: one on the P2P link and one on the CSMA LAN.

using namespace ns3;

// Defines a logging component named "SecondScriptExample".
NS_LOG_COMPONENT_DEFINE("SecondScriptExample");

int
main(int argc, char* argv[])
{
    // Define default variables for the simulation.
    bool verbose = true; // Flag to enable/disable application logging.
    uint32_t nCsma = 3;  // The number of extra nodes on the CSMA LAN (default is 3, making 4 LAN nodes total including n1).

    // Initialize the CommandLine parser to allow changing variables at runtime.
    CommandLine cmd(__FILE__);
    // Expose the 'nCsma' variable to the command line so the user can change the number of LAN nodes via terminal.
    cmd.AddValue("nCsma", "Number of \"extra\" CSMA nodes/devices", nCsma);
    // Expose the 'verbose' variable to the command line.
    cmd.AddValue("verbose", "Tell echo applications to log if true", verbose);

    // Parse the command line arguments provided by the user.
    cmd.Parse(argc, argv);

    // If the verbose flag is true, enable logging for the client and server applications.
    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

    // A safety check: if the user passes 0 for nCsma, force it to 1. 
    // The CSMA network requires at least 2 nodes (n1 + at least 1 extra node) to function properly.
    nCsma = nCsma == 0 ? 1 : nCsma;

    // Create a container for the two nodes that will form the Point-to-Point (P2P) link.
    NodeContainer p2pNodes;
    p2pNodes.Create(2); // Creates n0 and n1.

    // Create a separate container for the nodes that will be on the CSMA LAN.
    NodeContainer csmaNodes;
    // Add n1 (index 1 of the p2pNodes container) to the CSMA container. 
    // This is crucial: n1 is now in BOTH containers and will bridge the two networks.
    csmaNodes.Add(p2pNodes.Get(1));
    // Create the additional 'nCsma' nodes (default 3: n2, n3, n4) and add them to the CSMA container.
    csmaNodes.Create(nCsma);

    // Configure the Point-to-Point link attributes.
    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps")); // 5 Megabits per second bandwidth.
    pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));     // 2 milliseconds propagation delay.

    // Install the P2P devices (NICs) onto the p2pNodes (n0 and n1) and connect them with a channel.
    NetDeviceContainer p2pDevices;
    p2pDevices = pointToPoint.Install(p2pNodes);

    // Configure the CSMA (Ethernet) local area network attributes.
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));      // 100 Megabits per second LAN bandwidth.
    csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));   // A very small delay typical of short Ethernet cables.

    // Install the CSMA devices onto all nodes in the csmaNodes container (n1, n2, n3, n4).
    NetDeviceContainer csmaDevices;
    csmaDevices = csma.Install(csmaNodes);

    // Create the helper to install the TCP/IP stack.
    InternetStackHelper stack;
    // Install the stack on n0.
    stack.Install(p2pNodes.Get(0));
    // Install the stack on all nodes in the CSMA container (n1, n2, n3, n4). 
    // Note: n1 gets the stack here, which is why we didn't install it on p2pNodes as a whole.
    stack.Install(csmaNodes);

    // Helper to assign IP addresses.
    Ipv4AddressHelper address;
    
    // Set the base IP address for the Point-to-Point network to 10.1.1.0/24.
    address.SetBase("10.1.1.0", "255.255.255.0");
    // Assign IPs to the P2P devices (n0 gets 10.1.1.1, n1 gets 10.1.1.2).
    Ipv4InterfaceContainer p2pInterfaces;
    p2pInterfaces = address.Assign(p2pDevices);

    // Change the base IP address for the CSMA LAN to a new subnet: 10.1.2.0/24.
    address.SetBase("10.1.2.0", "255.255.255.0");
    // Assign IPs to the CSMA devices (n1 gets 10.1.2.1, n2 gets 10.1.2.2, etc.).
    Ipv4InterfaceContainer csmaInterfaces;
    csmaInterfaces = address.Assign(csmaDevices);

    // Set up a UDP Echo Server on port 9.
    UdpEchoServerHelper echoServer(9);

    // Install the server on the LAST node of the CSMA network. 
    // Since index 0 is n1, index 'nCsma' (default 3) will be the last node added (n4).
    ApplicationContainer serverApps = echoServer.Install(csmaNodes.Get(nCsma));
    serverApps.Start(Seconds(1));  // Server starts listening at 1.0s