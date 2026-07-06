/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Specifies that this open-source code is licensed under version 2.0 of the General Public License.
 */

// Include header files for various ns-3 modules needed to build and run the simulation.
#include "ns3/applications-module.h"     // Includes application-layer tools (e.g., UdpEchoClient/Server).
#include "ns3/core-module.h"             // Includes core simulation features (logging, time, command-line arguments).
#include "ns3/internet-module.h"         // Includes IPv4/IPv6 stacks, routing protocols, and socket implementations.
#include "ns3/network-module.h"          // Includes fundamental networking concepts (Nodes, NetDevices, Packets).
#include "ns3/point-to-point-module.h"   // Includes dedicated helpers for point-to-point links.

// Default Network Topology
//
//       10.1.1.0
// n0 -------------- n1
//    point-to-point
//
// ASCII art visualizing the two nodes (n0 and n1) connected directly by a point-to-point link.

// Imports the entire ns3 namespace so we don't have to prefix objects with 'ns3::' (e.g., using 'Time' instead of 'ns3::Time').
using namespace ns3;

// Registers a logging component named "FirstScriptExample" so console log messages can be enabled for this file specifically.
NS_LOG_COMPONENT_DEFINE("FirstScriptExample");

// The main entry point of the C++ program where simulation execution begins.
int
main(int argc, char* argv[])
{
    // Creates a command-line argument parser linked to this source file name.
    CommandLine cmd(__FILE__);
    // Parses runtime terminal arguments passed to the simulation script (allows setting attributes via terminal flags).
    cmd.Parse(argc, argv);

    // Sets the underlying simulation time resolution to Nanoseconds (NS), the standard unit for timing accuracy.
    Time::SetResolution(Time::NS);

    // Enables informational logging output for the UDP Echo Client application so we can see packet transmission messages.
    LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    // Enables informational logging output for the UDP Echo Server application so we can see packet reception/echo messages.
    LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

    // Creates a container object designed to hold a collection of network nodes.
    NodeContainer nodes;
    // Instantiates 2 empty, unconfigured virtual network nodes inside the container.
    nodes.Create(2);

    // Creates a helper object that simplifies building point-to-point network links and interface cards.
    PointToPointHelper pointToPoint;
    // Configures the helper so every NetDevice it creates transmits data at a speed of 5 Megabits per second.
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    // Configures the helper so every point-to-point Channel it creates introduces a propagation delay of 2 milliseconds.
    pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));

    // Creates a container to store the network interface cards (NetDevices) created during installation.
    NetDeviceContainer devices;
    // Installs point-to-point NetDevices on both nodes and links them together via a newly created point-to-point Channel.
    devices = pointToPoint.Install(nodes);

    // Creates a helper object responsible for installing the TCP/IP networking stack.
    InternetStackHelper stack;
    // Installs protocol implementations (ARP, IPv4, UDP, TCP, etc.) onto all nodes inside the 'nodes' container.
    stack.Install(nodes);

    // Creates a helper object that assigns IP addresses to network devices.
    Ipv4AddressHelper address;
    // Sets the base network subnet address to "10.1.1.0" with a standard /24 subnet mask ("255.255.255.0").
    address.SetBase("10.1.1.0", "255.255.255.0");

    // Assigns sequential IPv4 addresses (10.1.1.1 and 10.1.1.2) to the NetDevices and stores the interface references.
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // Creates a helper to configure a UDP Echo Server configured to listen on port number 9.
    UdpEchoServerHelper echoServer(9);

    // Installs the UDP Echo Server application onto the second node (index 1, IP: 10.1.1.2).
    ApplicationContainer serverApps = echoServer.Install(nodes.Get(1));
    // Schedules the server application to start listening for traffic at exactly 1.0 second into the simulation.
    serverApps.Start(Seconds(1));
    // Schedules the server application to shut down and stop processing traffic at 10.0 seconds into the simulation.
    serverApps.Stop(Seconds(10));

    // Creates a helper for a UDP Echo Client targeting the server's IP address (index 1: 10.1.1.2) on port 9.
    UdpEchoClientHelper echoClient(interfaces.GetAddress(1), 9);
    // Configures the client to send a maximum total of 1 echo request packet.
    echoClient.SetAttribute("MaxPackets", UintegerValue(1));
    // Configures the client to wait 1 second between consecutive packets (relevant if MaxPackets > 1).
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1)));
    // Sets the payload data size of the transmitted packet to 1024 bytes.
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    // Installs the UDP Echo Client application onto the first node (index 0, IP: 10.1.1.1).
    ApplicationContainer clientApps = echoClient.Install(nodes.Get(0));
    // Schedules the client application to start initiating packet transmissions at exactly 2.0 seconds.
    clientApps.Start(Seconds(2));
    // Schedules the client application to cease activities at 10.0 seconds.
    clientApps.Stop(Seconds(10));

    // Executes the scheduled simulation events chronologically until no events remain or a stop time is reached.
    Simulator::Run();
    // Cleans up internal memory, destroys nodes, and tears down network structures at the end of the simulation.
    Simulator::Destroy();

    // Returns 0 to indicate the program executed and terminated successfully.
    return 0;
}