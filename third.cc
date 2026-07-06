/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Specifies the open-source license under which this code is distributed.
 */

// Include necessary ns-3 modules, including new ones for Wi-Fi and Mobility.
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"       // Includes models for how nodes move in the simulation space.
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ssid.h"                  // Includes the Service Set Identifier (SSID) class for Wi-Fi networks.
#include "ns3/yans-wifi-helper.h"      // Includes helpers for the "Yet Another Network Simulator" (YANS) Wi-Fi models.

// Default Network Topology
//
//   Wifi 10.1.3.0
//                 AP
//  * * * *
//  |    |    |    |    10.1.1.0
// n5   n6   n7   n0 -------------- n1   n2   n3   n4
//                   point-to-point  |    |    |    |
//                                   ================
//                                     LAN 10.1.2.0
//
// Note: n0 acts as the Wi-Fi Access Point (AP) and a router to the P2P link. 
// n1 acts as a router between the P2P link and the CSMA LAN.

using namespace ns3;

// Defines a logging component named "ThirdScriptExample".
NS_LOG_COMPONENT_DEFINE("ThirdScriptExample");

int
main(int argc, char* argv[])
{
    // Default variables for the simulation.
    bool verbose = true;
    uint32_t nCsma = 3;  // Number of extra CSMA nodes.
    uint32_t nWifi = 3;  // Number of Wi-Fi Station (STA) client nodes.
    bool tracing = false; // Flag to enable/disable PCAP tracing.

    // Initialize the CommandLine parser to allow modifying these variables via the terminal.
    CommandLine cmd(__FILE__);
    cmd.AddValue("nCsma", "Number of \"extra\" CSMA nodes/devices", nCsma);
    cmd.AddValue("nWifi", "Number of wifi STA devices", nWifi);
    cmd.AddValue("verbose", "Tell echo applications to log if true", verbose);
    cmd.AddValue("tracing", "Enable pcap tracing", tracing);

    // Parse terminal commands.
    cmd.Parse(argc, argv);

    // The underlying restriction of 18 is due to the grid position
    // allocator's configuration; the grid layout will exceed the
    // bounding box if more than 18 nodes are provided.
    if (nWifi > 18)
    {
        std::cout << "nWifi should be 18 or less; otherwise grid layout exceeds the bounding box"
                  << std::endl;
        return 1; // Exit the program with an error code.
    }

    // Enable application logging if verbose is true.
    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

    // --- 1. SET UP POINT-TO-POINT (P2P) LINK ---
    NodeContainer p2pNodes;
    p2pNodes.Create(2); // Creates n0 and n1.

    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));

    NetDeviceContainer p2pDevices;
    p2pDevices = pointToPoint.Install(p2pNodes);

    // --- 2. SET UP CSMA (LAN) NETWORK ---
    NodeContainer csmaNodes;
    csmaNodes.Add(p2pNodes.Get(1)); // Add n1 to bridge P2P and CSMA.
    csmaNodes.Create(nCsma);        // Create n2, n3, n4.

    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
    csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));

    NetDeviceContainer csmaDevices;
    csmaDevices = csma.Install(csmaNodes);

    // --- 3. SET UP WI-FI NETWORK ---
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nWifi); // Create the wireless client nodes (n5, n6, n7).
    
    // Assign n0 (the first node of the P2P link) to act as the Wi-Fi Access Point (AP).
    NodeContainer wifiApNode = p2pNodes.Get(0);

    // Create the physical channel (the "air") using the default YANS Wi-Fi settings.
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    // Create the PHY layer helper.
    YansWifiPhyHelper phy;
    // Attach the PHY helper to the channel we just created.
    phy.SetChannel(channel.Create());

    // Create a Wi-Fi MAC (Media Access Control) helper.
    WifiMacHelper mac;
    // Define the SSID (network name) for our Wi-Fi network.
    Ssid ssid = Ssid("ns-3-ssid");

    // Create a general Wi-Fi helper (uses 802.11a standard by default in this ns-3 version).
    WifiHelper wifi;

    // Configure the MAC for the Station (STA) nodes (the clients).
    // "ActiveProbing" is false, meaning they will passively listen for AP beacons instead of sending probe requests.
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    
    NetDeviceContainer staDevices;
    // Install the Wi-Fi devices (PHY and MAC) onto the Station nodes.
    staDevices = wifi.Install(phy, mac, wifiStaNodes);

    // Reconfigure the MAC for the Access Point (AP) node.
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    
    NetDeviceContainer apDevices;
    // Install the Wi-Fi device onto the AP node (n0).
    apDevices = wifi.Install(phy, mac, wifiApNode);

    // --- 4. SET UP MOBILITY (NODE POSITIONS) ---
    MobilityHelper mobility;

    // Set up a grid position allocator to initially place the Wi-Fi STA nodes.
    // This places nodes starting at (0,0), moving right by 5 meters (DeltaX) for 3 nodes (GridWidth), 
    // then dropping down 10 meters (DeltaY) for the next row.
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(5.0),
                                  "DeltaY", DoubleValue(10.0),
                                  "GridWidth", UintegerValue(3),
                                  "LayoutType", StringValue("RowFirst"));

    // Tell the Wi-Fi STA nodes to move randomly within a 100x100 meter bounding box (-50 to 50 on X and Y axes).
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Bounds", RectangleValue(Rectangle(-50, 50, -50, 50)));
    mobility.Install(wifiStaNodes);

    // Tell the AP node (n0) to stay perfectly still (Constant Position).
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiApNode);

    // --- 5. INSTALL INTERNET STACK ---
    InternetStackHelper stack;
    stack.Install(csmaNodes);    // Installs on n1, n2, n3, n4.
    stack.Install(wifiApNode);   // Installs on n0.
    stack.Install(wifiStaNodes); // Installs on n5, n6, n7.

    // --- 6. ASSIGN IP ADDRESSES ---
    Ipv4AddressHelper address;

    // Subnet 1: Point-to-Point (10.1.1.0/24)
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer p2pInterfaces;
    p2pInterfaces = address.Assign(p2pDevices);

    // Subnet 2: CSMA LAN (10.1.2.0/24)
    address.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer csmaInterfaces;
    csmaInterfaces = address.Assign(csmaDevices);

    // Subnet 3: Wi-Fi Network (10.1.3.0/24)
    address.SetBase("10.1.3.0", "255.255.255.0");
    address.Assign(staDevices); // Assigns IPs to clients.
    address.Assign(apDevices);  // Assigns an IP to the AP.

    // --- 7. SET UP APPLICATIONS ---
    UdpEchoServerHelper echoServer(9);

    // Install the Echo Server on the last node of the CSMA network (n4).
    ApplicationContainer serverApps = echoServer.Install(csmaNodes.Get(nCsma));
    serverApps.Start(Seconds(1));
    serverApps.Stop(Seconds(10));

    // Install the Echo Client on the last Wi-Fi STA node.
    // It targets the IP address of the Echo Server on the CSMA network.
    UdpEchoClientHelper echoClient(csmaInterfaces.GetAddress(nCsma), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(1));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(wifiStaNodes.Get(nWifi - 1));
    clientApps.Start(Seconds(2));
    clientApps.Stop(Seconds(10));

    // --- 8. ROUTING ---
    // Automatically populate routing tables so nodes know how to route traffic across the 3 different subnets (via n0 and n1).
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Because some mobility models (like random walk) never technically "finish", 
    // we must explicitly tell the simulator when to stop, otherwise it runs forever.
    Simulator::Stop(Seconds(10));

    // --- 9. TRACING ---
    if (tracing)
    {
        // Change the format of the Wi-Fi PCAP traces to include radiotap headers (shows signal strength, rate, etc.).
        phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        
        // Enable PCAP tracing on the P2P link.
        pointToPoint.EnablePcapAll("third");
        
        // Enable PCAP tracing on the Access Point. Note we trace the PHY layer here to see raw radio frames.
        phy.EnablePcap("third", apDevices.Get(0));
        
        // Enable promiscuous PCAP tracing on the first CSMA node.
        csma.EnablePcap("third", csmaDevices.Get(0), true);
    }

    // --- 10. RUN SIMULATION ---
    Simulator::Run();
    Simulator::Destroy();
    
    return 0;
}