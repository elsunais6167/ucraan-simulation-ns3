/* ============================================================================
 *  aanet-sim.cc  —  UCRAAN AANET simulation skeleton for ns-3.47
 *
 *  ONE CONVERGED 5G/6G NTN ENVIRONMENT (we LEVERAGE both, not compare them):
 *
 *      [ LEO SATELLITE ]   <- 5G NR NTN inter-tier link (latency ~ms)
 *            |  \
 *            |   \         (alternate relay when the mesh is partitioned)
 *      [ AIRCRAFT MESH ]   <- airborne tier: A2A multi-hop, ISAC-capable (6G)
 *            |
 *      [ GROUND STATION ]  <- bottom tier
 *
 *    5G is leveraged via the satellite/NTN inter-tier path.
 *    6G is leveraged via ISAC sensing -> per-link uncertainty (done in Python,
 *    fed back as UCRAAN routes; see the hook at the bottom).
 *
 *  WHAT THIS PROGRAM DOES
 *    1. Reads mobility.csv (real flight trajectories) -> moving aircraft.
 *    2. Builds the A2A airborne mesh AND the satellite/ground NTN tier.
 *    3. Sends a UDP flow between two aircraft; routing may use the mesh or
 *       fall back to the satellite relay.
 *    4. Measures Packet Delivery Ratio (PDR) and end-to-end delay.
 *
 *  ROUTING
 *    Default = OLSR (a real ad-hoc protocol + one of your baselines) running
 *    across BOTH tiers, so it naturally exploits the satellite when the mesh
 *    breaks. To inject UCRAAN's routes, see InstallStaticRoutesFromFile().
 *
 *  BUILD / RUN  (from the ns-3.47 root):
 *    cp aanet-sim.cc scratch/
 *    ./ns3 build
 *    ./ns3 run "scratch/aanet-sim --mobility=aanet/data/mobility.csv"
 * ==========================================================================*/

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AanetSim");

struct TrajPoint
{
    double t, x, y, z;
};

// Read a mobility CSV -> per-aircraft time-sorted trajectory points.
// Robust: finds columns by NAME (icao24,t,x,y,z) in any order, tolerates an
// index column or extra columns, and skips rows it can't parse instead of
// crashing. Returns aircraft IDs in first-seen order.
static std::string
trim(std::string s)
{
    while (!s.empty() && (s.back() == '\r' || s.back() == ' ' || s.back() == '\n'))
        s.pop_back();
    size_t i = 0;
    while (i < s.size() && (s[i] == ' '))
        ++i;
    return s.substr(i);
}

std::vector<std::string>
ReadMobility(const std::string& path,
             std::map<std::string, std::vector<TrajPoint>>& traj,
             double& maxTime)
{
    std::vector<std::string> order;
    std::ifstream in(path.c_str());
    NS_ABORT_MSG_IF(!in.is_open(), "Cannot open mobility file: " << path);

    auto split = [](const std::string& line) {
        std::vector<std::string> c;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ','))
            c.push_back(trim(cell));
        return c;
    };

    // header -> column index
    std::string header;
    std::getline(in, header);
    std::vector<std::string> cols = split(header);
    int ci = -1, ct = -1, cx = -1, cy = -1, cz = -1;
    for (size_t i = 0; i < cols.size(); ++i)
    {
        if (cols[i] == "icao24") ci = i;
        else if (cols[i] == "t" || cols[i] == "time") ct = i;
        else if (cols[i] == "x") cx = i;
        else if (cols[i] == "y") cy = i;
        else if (cols[i] == "z") cz = i;
    }
    NS_LOG_UNCOND("mobility columns: icao24@" << ci << " t@" << ct
                  << " x@" << cx << " y@" << cy << " z@" << cz);
    NS_ABORT_MSG_IF(ci < 0 || ct < 0 || cx < 0 || cy < 0 || cz < 0,
                    "mobility file must have columns icao24,t,x,y,z (got header: "
                    << header << "). Did you run build_mobility_cluster.py?");

    maxTime = 0.0;
    uint32_t skipped = 0;
    std::string line;
    int maxNeeded = std::max({ci, ct, cx, cy, cz});
    while (std::getline(in, line))
    {
        if (trim(line).empty())
            continue;
        std::vector<std::string> c = split(line);
        if ((int)c.size() <= maxNeeded)
        {
            ++skipped;
            continue;
        }
        TrajPoint p;
        try
        {
            p.t = std::stod(c[ct]);
            p.x = std::stod(c[cx]);
            p.y = std::stod(c[cy]);
            p.z = std::stod(c[cz]);
        }
        catch (...)
        {
            ++skipped; // non-numeric / empty cell -> skip this row
            continue;
        }
        std::string id = c[ci];
        if (traj.find(id) == traj.end())
            order.push_back(id);
        traj[id].push_back(p);
        if (p.t > maxTime)
            maxTime = p.t;
    }
    if (skipped)
        NS_LOG_UNCOND("ReadMobility: skipped " << skipped << " unparsable rows");
    for (auto& kv : traj)
        std::sort(kv.second.begin(), kv.second.end(),
                  [](const TrajPoint& a, const TrajPoint& b) { return a.t < b.t; });
    return order;
}

// ---- UCRAAN route injection (reads routes.csv from the Python solver) ------
struct RouteEntry
{
    uint32_t node;
    Ipv4Address dest, nexthop;
};
static std::map<double, std::vector<RouteEntry>> g_schedule;
static NodeContainer g_allNodes;

// Find the local interface on `node` that shares a subnet with `nexthop`,
// so we don't have to hard-code mesh vs NTN interface indices.
static int32_t
IfaceForNexthop(Ptr<Ipv4> ip, Ipv4Address nexthop)
{
    for (uint32_t i = 1; i < ip->GetNInterfaces(); ++i)
        for (uint32_t j = 0; j < ip->GetNAddresses(i); ++j)
        {
            Ipv4InterfaceAddress ia = ip->GetAddress(i, j);
            if (ia.GetLocal().CombineMask(ia.GetMask()) ==
                nexthop.CombineMask(ia.GetMask()))
                return (int32_t)i;
        }
    return -1;
}

// At each slot: wipe last slot's static routes, install this slot's UCRAAN routes.
static void
ApplyRoutes(double t)
{
    Ipv4StaticRoutingHelper h;
    for (uint32_t k = 0; k < g_allNodes.GetN(); ++k)
    {
        Ptr<Ipv4StaticRouting> sr = h.GetStaticRouting(g_allNodes.Get(k)->GetObject<Ipv4>());
        while (sr->GetNRoutes() > 0)
            sr->RemoveRoute(0);
    }
    for (const RouteEntry& e : g_schedule[t])
    {
        Ptr<Ipv4> ip = g_allNodes.Get(e.node)->GetObject<Ipv4>();
        int32_t iface = IfaceForNexthop(ip, e.nexthop);
        if (iface >= 0)
            h.GetStaticRouting(ip)->AddHostRouteTo(e.dest, e.nexthop, (uint32_t)iface);
    }
}

// Bring all of the satellite's links down (handover outage) or back up.
static void
SetSatLinks(Ptr<Node> sat, bool up)
{
    Ptr<Ipv4> ip = sat->GetObject<Ipv4>();
    for (uint32_t i = 1; i < ip->GetNInterfaces(); ++i) // skip loopback (0)
    {
        if (up)
            ip->SetUp(i);
        else
            ip->SetDown(i);
    }
}

// ---- trace-based route metrics: sample the source's next hop toward dst -----
// Works identically for OLSR (dynamic) and the static UCRAAN/delay routes, so
// every protocol gets route-lifetime / reroute numbers from the same mechanism.
static std::vector<std::string> g_nexthopSeq;
static Ptr<Node> g_srcNode;
static Ipv4Address g_dstAddr;
static Ipv4Address g_srcAddr;

static void
SampleNextHop()
{
    Ptr<Ipv4> ip = g_srcNode->GetObject<Ipv4>();
    Ptr<Ipv4RoutingProtocol> rp = ip->GetRoutingProtocol();
    Ipv4Header hdr;
    hdr.SetDestination(g_dstAddr);
    hdr.SetSource(g_srcAddr);
    hdr.SetProtocol(17); // UDP
    Socket::SocketErrno serr;
    Ptr<Ipv4Route> route = rp->RouteOutput(Ptr<Packet>(), hdr, nullptr, serr);
    std::ostringstream nh;
    if (route)
        nh << route->GetGateway();
    else
        nh << "none";
    g_nexthopSeq.push_back(nh.str());
}

int
main(int argc, char* argv[])
{
    // ---- STEP A: options ---------------------------------------------------
    std::string mobilityFile = "mobility.csv";
    std::string outFile = "results.csv";
    std::string routesFile = ""; // empty => OLSR baseline; set => run UCRAAN
    double meshRange = 120000.0;  // A2A link reach (m)
    double ntnRange = 1500000.0;  // aircraft<->satellite reach (m)
    double satAlt = 600000.0;     // LEO altitude (m) -> sets NTN latency
    double geoScale = 1.0;        // shrink world so ns-3 Wi-Fi can ACK (see note)
    std::string appRate = "10Mbps";
    uint32_t srcIdx = 0;
    uint32_t dstIdx = 0; // 0 => last aircraft
    double simTime = 0.0;

    CommandLine cmd;
    cmd.AddValue("mobility", "Path to mobility.csv", mobilityFile);
    cmd.AddValue("output", "Results CSV to append to", outFile);
    cmd.AddValue("routes", "UCRAAN routes.csv (empty=OLSR baseline)", routesFile);
    cmd.AddValue("meshRange", "A2A link range (m)", meshRange);
    cmd.AddValue("ntnRange", "Aircraft-satellite range (m)", ntnRange);
    cmd.AddValue("satAlt", "LEO satellite altitude (m)", satAlt);
    cmd.AddValue("geoScale", "Shrink positions for ns-3 Wi-Fi, e.g. 0.001", geoScale);
    double handoverPeriod = 0.0; // seconds between satellite outages (0=off)
    double handoverDur = 5.0;    // seconds the satellite is down each handover
    bool debug = false;          // print source routing table mid-run
    cmd.AddValue("handoverPeriod", "Seconds between satellite handover outages (0=off)", handoverPeriod);
    cmd.AddValue("handoverDur", "Seconds satellite is down per handover", handoverDur);
    cmd.AddValue("debug", "Print source routing table mid-run", debug);
    double sampleInt = 2.0; // how often to sample the src->dst next hop (s)
    cmd.AddValue("sampleInt", "Seconds between next-hop samples for route metrics", sampleInt);
    cmd.AddValue("appRate", "Offered traffic rate", appRate);
    cmd.AddValue("src", "Source aircraft index", srcIdx);
    cmd.AddValue("dst", "Destination aircraft index (0=last)", dstIdx);
    cmd.AddValue("simTime", "Seconds (0=auto)", simTime);
    cmd.Parse(argc, argv);

    // ns-3's Wi-Fi MAC only ACKs unicast over a few km (it assumes propagation
    // delay << slot time). Aircraft links are 10s-100s of km, so we shrink the
    // whole world by geoScale. Relative geometry -- and therefore the entire
    // topology and its dynamics -- is preserved exactly; only absolute
    // distances/delays change. Use --geoScale=0.001 to make Wi-Fi deliver.
    double satAltReal = satAlt;   // keep TRUE altitude for NTN latency (P2P needs no geoScale)
    meshRange *= geoScale;
    ntnRange *= geoScale;
    satAlt *= geoScale;
    NS_LOG_UNCOND("geoScale=" << geoScale << "  meshRange=" << meshRange
                  << "m  ntnRange=" << ntnRange << "m  satAlt=" << satAlt << "m");

    // ---- STEP B: load trajectories, create the three tiers -----------------
    std::map<std::string, std::vector<TrajPoint>> traj;
    double trajMax = 0.0;
    std::vector<std::string> ids = ReadMobility(mobilityFile, traj, trajMax);
    uint32_t nAir = ids.size();
    NS_ABORT_MSG_IF(nAir < 2, "Need >= 2 aircraft in mobility.csv");
    if (simTime <= 0.0)
        simTime = trajMax + 1.0;
    if (dstIdx == 0)
        dstIdx = nAir - 1;

    NodeContainer aircraft;
    aircraft.Create(nAir);     // middle tier: the AANET mesh
    NodeContainer infra;
    infra.Create(2);           // infra.Get(0)=satellite, infra.Get(1)=ground
    Ptr<Node> satNode = infra.Get(0);
    Ptr<Node> groundNode = infra.Get(1);

    NodeContainer allNodes;
    allNodes.Add(aircraft);
    allNodes.Add(infra);

    NS_LOG_UNCOND("5G/6G NTN environment: " << nAir << " aircraft + 1 LEO sat + 1 ground");
    NS_LOG_UNCOND("simTime=" << simTime << "s  src=" << srcIdx << "  dst=" << dstIdx);

    // ---- STEP C: mobility --------------------------------------------------
    // Aircraft fly the real trajectories; satellite & ground are fixed.
    MobilityHelper airMob;
    airMob.SetMobilityModel("ns3::WaypointMobilityModel");
    airMob.Install(aircraft);
    for (uint32_t i = 0; i < nAir; ++i)
    {
        Ptr<WaypointMobilityModel> wp = aircraft.Get(i)->GetObject<WaypointMobilityModel>();
        for (const TrajPoint& p : traj[ids[i]])
            wp->AddWaypoint(Waypoint(Seconds(p.t),
                                     Vector(p.x * geoScale, p.y * geoScale, p.z * geoScale)));
    }
    MobilityHelper fixedMob;
    Ptr<ListPositionAllocator> pa = CreateObject<ListPositionAllocator>();
    pa->Add(Vector(0, 0, satAlt)); // satellite overhead -> realistic LEO delay
    pa->Add(Vector(0, 0, 0));      // ground station
    fixedMob.SetPositionAllocator(pa);
    fixedMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    fixedMob.Install(infra);

    // ---- STEP D: radios ----------------------------------------------------
    // Two channels. Both use a range-based loss model so a link exists iff
    // nodes are within reach; the speed-of-light delay model turns the 600 km
    // satellite altitude into a realistic NTN latency automatically.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("OfdmRate54Mbps"),
                                 "ControlMode", StringValue("OfdmRate6Mbps"));
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    // (1) A2A airborne mesh
    YansWifiChannelHelper meshChan;
    meshChan.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    meshChan.AddPropagationLoss("ns3::RangePropagationLossModel",
                                "MaxRange", DoubleValue(meshRange));
    YansWifiPhyHelper meshPhy;
    meshPhy.SetChannel(meshChan.Create());
    meshPhy.Set("TxPowerStart", DoubleValue(46.0));
    meshPhy.Set("TxPowerEnd", DoubleValue(46.0));
    NetDeviceContainer meshDev = wifi.Install(meshPhy, mac, aircraft);

    // (2) NTN inter-tier (5G NR NTN) as a BENT-PIPE STAR: each aircraft gets a
    // dedicated point-to-point link to the satellite, and the satellite links
    // to the ground. Aircraft reach each other via the satellite ONLY through
    // the satellite -- so a handover that drops the satellite truly cuts the
    // inter-tier path. P2P also sidesteps Wi-Fi's long-range ACK limit, so this
    // tier needs no geoScale and uses a realistic LEO one-way delay.
    // Realistic LEO bent-pipe latency: derive the one-way aircraft<->satellite
    // delay from the TRUE altitude (P2P needs no geoScale) and add a transponder
    // processing delay. A two-hop aircraft->sat->aircraft relay therefore costs
    // ~2x this -- materially MORE than a healthy multi-hop mesh path. This is the
    // honest cost that makes a delay-aware/predictive router (UCRAAN) prefer the
    // mesh, while OLSR's hop-count metric still grabs the slower 2-hop relay.
    const double C_MPS = 3.0e8;
    double satOneWayMs = (satAltReal / C_MPS) * 1e3;   // nadir LEO propagation
    double satProcMs = 1.5;                            // transponder processing
    double satLinkMs = satOneWayMs + satProcMs;        // per aircraft<->sat hop
    std::ostringstream satDelayStr;
    satDelayStr << satLinkMs << "ms";
    NS_LOG_UNCOND("NTN per-link one-way delay = " << satLinkMs << " ms  (bent-pipe "
                  << "aircraft->sat->aircraft ~ " << 2 * satLinkMs << " ms)");
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("25Mbps")); // realistic NTN user-link rate
    p2p.SetChannelAttribute("Delay", StringValue(satDelayStr.str()));
    std::vector<NetDeviceContainer> satLinks; // satLinks[i] = aircraft_i <-> sat
    for (uint32_t i = 0; i < nAir; ++i)
        satLinks.push_back(p2p.Install(aircraft.Get(i), satNode));
    NetDeviceContainer groundLink = p2p.Install(satNode, groundNode);

    // ---- STEP E: IP + routing (OLSR over BOTH tiers) -----------------------
    OlsrHelper olsr;
    Ipv4StaticRoutingHelper staticRouting;
    Ipv4ListRoutingHelper listRouting;
    listRouting.Add(staticRouting, 0);     // UCRAAN routes go here (priority 0)
    if (routesFile.empty())
        listRouting.Add(olsr, 10);         // baseline: OLSR over both tiers
    InternetStackHelper internet;
    internet.SetRoutingHelper(listRouting);
    internet.Install(allNodes);

    Ipv4AddressHelper ip;
    ip.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer meshIf = ip.Assign(meshDev);
    // satellite star: link i uses 10.3.i.0/24 (aircraft=.1, satellite=.2)
    for (uint32_t i = 0; i < nAir; ++i)
    {
        std::ostringstream net;
        net << "10.3." << i << ".0";
        ip.SetBase(net.str().c_str(), "255.255.255.0");
        ip.Assign(satLinks[i]);
    }
    ip.SetBase("10.4.0.0", "255.255.255.0");
    ip.Assign(groundLink);

    // schedule satellite handover outages (cuts the whole inter-tier star)
    if (handoverPeriod > 0.0)
        for (double th = handoverPeriod; th < simTime; th += handoverPeriod)
        {
            Simulator::Schedule(Seconds(th), &SetSatLinks, satNode, false);
            Simulator::Schedule(Seconds(th + handoverDur), &SetSatLinks, satNode, true);
        }

    // ---- load UCRAAN routes (if provided) and schedule per-slot installs ---
    if (!routesFile.empty())
    {
        std::ifstream rin(routesFile.c_str());
        NS_ABORT_MSG_IF(!rin.is_open(), "Cannot open routes file: " << routesFile);
        std::string l;
        std::getline(rin, l); // header: time,node,dest_ip,nexthop_ip
        while (std::getline(rin, l))
        {
            if (l.empty())
                continue;
            std::stringstream ss(l);
            std::string c;
            std::vector<std::string> v;
            while (std::getline(ss, c, ','))
                v.push_back(c);
            if (v.size() < 4)
                continue;
            RouteEntry e{(uint32_t)std::stoul(v[1]),
                         Ipv4Address(v[2].c_str()),
                         Ipv4Address(v[3].c_str())};
            g_schedule[std::stod(v[0])].push_back(e);
        }
        g_allNodes = allNodes;
        for (auto& kv : g_schedule)
            Simulator::Schedule(Seconds(kv.first), &ApplyRoutes, kv.first);
        NS_LOG_UNCOND("UCRAAN mode: " << g_schedule.size() << " route slots loaded");
    }

    // ---- STEP F: UDP flow, source aircraft -> destination aircraft ---------
    uint16_t port = 9;
    Ipv4Address dstAddr = meshIf.GetAddress(dstIdx);
    NS_LOG_UNCOND("source -> " << dstAddr << " (dst mesh IP)");
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
    onoff.SetConstantRate(DataRate(appRate), 1024);
    onoff.SetAttribute("StartTime", TimeValue(Seconds(2.0)));
    onoff.SetAttribute("StopTime", TimeValue(Seconds(simTime - 1.0)));
    onoff.Install(aircraft.Get(srcIdx));

    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer dstApp = sink.Install(aircraft.Get(dstIdx));
    dstApp.Start(Seconds(0.0));
    dstApp.Stop(Seconds(simTime));

    // ---- STEP G: measurement ----------------------------------------------
    FlowMonitorHelper fm;
    Ptr<FlowMonitor> monitor = fm.InstallAll();

    // sample the source's next hop toward dst every sampleInt seconds, so we can
    // derive route lifetime + reroute count for ANY protocol from ns-3 itself.
    g_srcNode = aircraft.Get(srcIdx);
    g_dstAddr = dstAddr;
    g_srcAddr = meshIf.GetAddress(srcIdx);
    for (double ts = 2.0; ts < simTime; ts += sampleInt)
        Simulator::Schedule(Seconds(ts), &SampleNextHop);

    // DEBUG (only with --debug): dump the SOURCE node's routing table mid-run.
    if (debug)
    {
        double tDump = simTime * 0.3;
        if (tDump > 60.0)
            tDump = 60.0;
        Ptr<OutputStreamWrapper> dbg = Create<OutputStreamWrapper>(&std::cout);
        Ipv4RoutingHelper::PrintRoutingTableAt(Seconds(tDump), aircraft.Get(srcIdx), dbg);
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // ---- FlowMonitor: data delivery + on-air control overhead --------------
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(fm.GetClassifier());
    uint64_t tx = 0, r x = 0, lost = 0, rxBytes = 0, ctrlPkts = 0;
    double delaySum = 0.0, jitterSum = 0.0;
    for (const auto& kv : monitor->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple ft = classifier->FindFlow(kv.first);
        if (ft.destinationPort == 698) // OLSR control traffic
        {
            continue; // don't count control as data
        }
        tx += kv.second.txPackets;
        rx += kv.second.rxPackets;
        lost += kv.second.lostPackets;
        rxBytes += kv.second.rxBytes;
        delaySum += kv.second.delaySum.GetSeconds();
        jitterSum += kv.second.jitterSum.GetSeconds();
    }
    double pdr = tx ? (double)rx / tx : 0.0;
    double meanDelayMs = rx ? 1000.0 * delaySum / rx : 0.0;
    double jitterMs = rx > 1 ? 1000.0 * jitterSum / (rx - 1) : 0.0;
    double thrKbps = 8.0 * rxBytes / 1000.0 / simTime;
    double ctrlPps = ctrlPkts / simTime;

    // ---- route metrics from the next-hop samples ---------------------------
    uint32_t reroutes = 0;
    std::vector<uint32_t> runs;
    uint32_t run = 1;
    for (size_t i = 1; i < g_nexthopSeq.size(); ++i)
    {
        if (g_nexthopSeq[i] != g_nexthopSeq[i - 1])
        {
            ++reroutes;
            runs.push_back(run);
            run = 1;
        }
        else
            ++run;
    }
    if (!g_nexthopSeq.empty())
        runs.push_back(run);
    double meanRun = 0.0;
    for (uint32_t r : runs)
        meanRun += r;
    meanRun = runs.empty() ? 0.0 : meanRun / runs.size();
    double routeLifeS = meanRun * sampleInt;
    double reroutePerS = simTime ? reroutes / simTime : 0.0;

    NS_LOG_UNCOND("---- RESULT ----  tx=" << tx << "  rx=" << rx
                  << "  PDR=" << pdr << "  meanDelay=" << meanDelayMs
                  << "  jitter=" << jitterMs << "  throughput=" << thrKbps
                  << "  lost=" << lost << "  routeLife=" << routeLifeS
                  << "  reroutePerS=" << reroutePerS << "  ctrlPps=" << ctrlPps);

    Simulator::Destroy();
    return 0;
}

/* ============================================================================
 *  HOW TO RUN UCRAAN (now built in)
 *  --------------------------------------------------------------------------
 *  1. Generate routes with the Python solver (the ISAC/uncertainty brain):
 *       python3 ucraan_solver.py --mobility aanet/data/mobility.csv \
 *               --src 0 --dst -1 --sensing-error 1.0 --eps 0.1
 *  2. Run ns-3 with those routes (OLSR is disabled automatically):
 *       ./ns3 run "scratch/aanet-sim --mobility=aanet/data/mobility.csv \
 *                  --routes=routes.csv"
 *  3. Baseline run = same command WITHOUT --routes (OLSR over both tiers).
 *  Compare the results.csv rows from the two runs => your headline result.
 * ==========================================================================*/