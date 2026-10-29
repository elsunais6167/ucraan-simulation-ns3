#!/usr/bin/env python3
# ============================================================================
#  aanet_sim.py   (file 3 of 4) -- the ns-3 simulator. Called by run_experiments.py;
#  can also be run alone (see usage below).
#
#  aanet_sim.py  —  UCRAAN AANET simulation for ns-3.47 PYTHON (cppyy) bindings
#
#  Python port of aanet-sim.cc. Same model, same CLI flags, same RESULT line,
#  so run_experiments.py's parser reads it. Run it with the ns-3
#  python that has the cppyy bindings (the one your Sionna/ns-3 install uses):
#
#      python3 scratch/aanet/aanet_sim.py \
#          --mobility=scratch/aanet/data/cluster_mobility.csv \
#          --geoScale=0.0005 --simTime=120
#
#  With UCRAAN/delay routes (OLSR auto-disabled when --routes is given):
#      python3 scratch/aanet/aanet_sim.py --mobility=... --routes=routes.csv
#
#  ONE CONVERGED 5G/6G NTN ENVIRONMENT (leveraged together, not compared):
#      [ LEO SATELLITE ]  <- 5G NR NTN inter-tier link (bent-pipe star)
#      [ AIRCRAFT MESH ]  <- airborne tier: A2A multi-hop, ISAC-capable (6G)
#      [ GROUND STATION ] <- bottom tier
#  5G via the satellite/NTN path; 6G/ISAC via the Python solver's uncertainty
#  routes fed in through --routes.
# ============================================================================

import argparse
import csv
import sys

from ns import ns

# ---------------------------------------------------------------------------
# A tiny JIT-compiled C++ helper for the one call that is painful from Python:
# Ipv4RoutingProtocol::RouteOutput takes a Socket::SocketErrno by NON-CONST
# reference (an output arg), which cppyy cannot bind cleanly. We wrap it in
# C++ and just return the chosen next hop as a string. Works for OLSR (dynamic)
# AND the static UCRAAN/delay routes, so every protocol's route-lifetime /
# reroute numbers come from the same mechanism.
# ---------------------------------------------------------------------------
HAVE_SAMPLER = False
try:
    ns.cppyy.cppdef(r"""
        #include "ns3/core-module.h"
        #include "ns3/network-module.h"
        #include "ns3/internet-module.h"
        #include <sstream>
        #include "ns3/olsr-module.h"
        #include "ns3/wifi-module.h"
        #include "ns3/propagation-module.h"
        #include "ns3/mobility-module.h"
        namespace ns3 {
        static uint64_t g_aanetOlsrTx = 0;
        static void AanetOlsrTx(const olsr::PacketHeader&, const olsr::MessageList&) { g_aanetOlsrTx++; }
        void AanetHookOlsr() {
            Config::ConnectWithoutContext("/NodeList/*/$ns3::olsr::RoutingProtocol/Tx",
                                          MakeCallback(&AanetOlsrTx));
        }
        uint64_t AanetOlsrTxCount() { return g_aanetOlsrTx; }

        // ---- ISAC time-split. Frames of period T are synchronised network-wide;
        // aircraft i spends the first rho_i*T of every frame in a sensing dwell.
        //  * transmit side: the aircraft sets its own NAV for the dwell, so its
        //    MAC defers data exactly as a TDD ISAC frame would;
        //  * receive side: a propagation-layer gate drops any frame whose
        //    transmitter or receiver is dwelling (the radio is busy sensing).
        // A hop is therefore usable only while both ends are out of their dwell.
        static std::vector<Ptr<ChannelAccessManager>> g_isacCam;
        static std::vector<double> g_isacRho, g_isacEnd;
        static double g_isacT = 0.01;
        static double g_isacDwellSec = 0.0;
        static uint32_t g_isacN = 0;
        class AanetDwellGate : public PropagationLossModel {
          public:
            static TypeId GetTypeId() {
                static TypeId tid = TypeId("ns3::AanetDwellGate")
                    .SetParent<PropagationLossModel>().SetGroupName("Propagation");
                return tid;
            }
          private:
            static bool Dwelling(Ptr<MobilityModel> m) {
                Ptr<Node> n = m->GetObject<Node>();
                if (!n) return false;
                uint32_t id = n->GetId();
                return id < g_isacEnd.size() && Simulator::Now().GetSeconds() < g_isacEnd[id];
            }
            double DoCalcRxPower(double tx, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override {
                return (Dwelling(a) || Dwelling(b)) ? -1000.0 : tx;
            }
            int64_t DoAssignStreams(int64_t s) override { return 0; }
        };
        void AanetAttachGate(Ptr<YansWifiChannel> ch, double maxRange) {
            Ptr<RangePropagationLossModel> r = CreateObject<RangePropagationLossModel>();
            r->SetAttribute("MaxRange", DoubleValue(maxRange));
            r->SetNext(CreateObject<AanetDwellGate>());
            ch->SetPropagationLossModel(r);
        }
        void AanetIsacAdd(Ptr<NetDevice> dev) {
            Ptr<WifiNetDevice> w = DynamicCast<WifiNetDevice>(dev);
            g_isacCam.push_back(w->GetMac()->GetChannelAccessManager());
            g_isacRho.push_back(0.0); g_isacEnd.push_back(-1.0); g_isacN++;
        }
        void AanetIsacSetRho(uint32_t i, double r) { if (i < g_isacRho.size()) g_isacRho[i] = r; }
        double AanetIsacDwell() { return g_isacDwellSec; }
        static void AanetIsacFrame() {
            double now = Simulator::Now().GetSeconds();
            for (uint32_t i = 0; i < g_isacN; ++i) {
                double r = g_isacRho[i];
                if (r <= 1e-6) continue;
                g_isacEnd[i] = now + r * g_isacT;
                g_isacCam[i]->NotifyNavStartNow(Seconds(r * g_isacT));
                g_isacDwellSec += r * g_isacT;
            }
            Simulator::Schedule(Seconds(g_isacT), &AanetIsacFrame);
        }
        void AanetResetAll() {             // for persistent (multi-run) workers
            g_isacCam.clear(); g_isacRho.clear(); g_isacEnd.clear();
            g_isacN = 0; g_isacDwellSec = 0.0; g_aanetOlsrTx = 0;
        }
        void AanetIsacStart(double T, double t0) {
            g_isacT = T; Simulator::Schedule(Seconds(t0), &AanetIsacFrame);
        }
        std::string AanetNextHop(Ptr<Node> src, Ipv4Address dst, Ipv4Address srcAddr) {
            Ptr<Ipv4> ip = src->GetObject<Ipv4>();
            if (!ip) return "none";
            Ptr<Ipv4RoutingProtocol> rp = ip->GetRoutingProtocol();
            if (!rp) return "none";
            Ipv4Header hdr;
            hdr.SetDestination(dst);
            hdr.SetSource(srcAddr);
            hdr.SetProtocol(17); // UDP
            Socket::SocketErrno serr;
            Ptr<Ipv4Route> route = rp->RouteOutput(Ptr<Packet>(), hdr, nullptr, serr);
            std::ostringstream nh;
            if (route) nh << route->GetGateway();
            else       nh << "none";
            return nh.str();
        }
        }
    """)
    HAVE_SAMPLER = True
except Exception as ex:  # pragma: no cover - depends on local ns-3 build
    print(f"WARN: next-hop sampler unavailable ({ex}); "
          f"route-lifetime/reroute will report 0", file=sys.stderr)

# ---- globals shared with the scheduled callbacks --------------------------
g_schedule = {}       # time(float) -> list of (node_idx, dest_ip, nexthop_ip)
g_allNodes = None
g_srcNode = None
g_dstAddr = None
g_srcAddr = None
g_nexthopSeq = []     # sampled next-hop strings, in time order
g_pyEvents = []       # (time, seq, fn, args) python callbacks
g_rho = {}            # time -> list of (node, rho)


# ---------------------------------------------------------------------------
# PORTABILITY: some ns-3 python wheels (e.g. 3.44 cppyy) cannot pass a Python
# callable + args to Simulator.Schedule. Instead we advance the simulator to
# each callback time with Stop()+Run() and invoke the callback in Python.
# Semantics are identical: every callback fires at exactly its sim time.
# ---------------------------------------------------------------------------
def py_event(t, fn, *a):
    g_pyEvents.append((float(t), len(g_pyEvents), fn, a))


def run_with_py_events(sim_time):
    for t, _, fn, a in sorted(g_pyEvents, key=lambda e: (e[0], e[1])):
        if t >= sim_time:
            break
        now = ns.Simulator.Now().GetSeconds()
        if t > now:
            ns.Simulator.Stop(ns.Seconds(t - now))
            ns.Simulator.Run()
        fn(*a)
    now = ns.Simulator.Now().GetSeconds()
    if sim_time > now:
        ns.Simulator.Stop(ns.Seconds(sim_time - now))
        ns.Simulator.Run()


# ---------------------------------------------------------------------------
# Mobility CSV -> per-aircraft time-sorted trajectory. Finds columns by NAME
# (icao24,t,x,y,z) in any order, tolerates extra columns, skips unparsable rows.
# ---------------------------------------------------------------------------
def read_mobility(path):
    traj, order = {}, []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = [h.strip() for h in next(reader)]
        idx = {name: i for i, name in enumerate(header)}

        def col(*names):
            for nm in names:
                if nm in idx:
                    return idx[nm]
            return -1

        ci, ct = col("icao24"), col("t", "time")
        cx, cy, cz = col("x"), col("y"), col("z")
        print(f"mobility columns: icao24@{ci} t@{ct} x@{cx} y@{cy} z@{cz}")
        if min(ci, ct, cx, cy, cz) < 0:
            sys.exit("mobility file must have columns icao24,t,x,y,z "
                     f"(got header: {header}). Did you run build_mobility_cluster.py?")

        need = max(ci, ct, cx, cy, cz)
        skipped, maxt = 0, 0.0
        for row in reader:
            if not row or len(row) <= need:
                skipped += 1
                continue
            try:
                t, x, y, z = (float(row[ct]), float(row[cx]),
                              float(row[cy]), float(row[cz]))
            except ValueError:
                skipped += 1
                continue
            aid = row[ci].strip()
            if not aid:            # empty icao24 -> not a real aircraft, skip
                skipped += 1
                continue
            if aid not in traj:
                traj[aid] = []
                order.append(aid)
            traj[aid].append((t, x, y, z))
            maxt = max(maxt, t)

    if skipped:
        print(f"ReadMobility: skipped {skipped} unparsable rows")
    for aid in traj:
        traj[aid].sort(key=lambda p: p[0])
    return order, traj, maxt


# ---------------------------------------------------------------------------
# scheduled callbacks (run inside Simulator.Run)
# ---------------------------------------------------------------------------
def iface_for_nexthop(ip, nexthop):
    """Local interface on this node that shares a subnet with nexthop."""
    for i in range(1, ip.GetNInterfaces()):
        for j in range(ip.GetNAddresses(i)):
            ia = ip.GetAddress(i, j)
            m = ia.GetMask()
            if ia.GetLocal().CombineMask(m).Get() == nexthop.CombineMask(m).Get():
                return i
    return -1


def apply_routes(t):
    """Wipe last slot's static routes, install this slot's UCRAAN/delay routes."""
    h = ns.Ipv4StaticRoutingHelper()
    for k in range(g_allNodes.GetN()):
        sr = h.GetStaticRouting(g_allNodes.Get(k).GetObject[ns.Ipv4]())
        while sr.GetNRoutes() > 0:
            sr.RemoveRoute(0)
    for node_idx, dest_ip, nexthop_ip in g_schedule.get(t, []):
        ip = g_allNodes.Get(node_idx).GetObject[ns.Ipv4]()
        iface = iface_for_nexthop(ip, nexthop_ip)
        if iface >= 0:
            h.GetStaticRouting(ip).AddHostRouteTo(dest_ip, nexthop_ip, iface)


def apply_rho(t):
    for node, r in g_rho.get(t, []):
        ns.AanetIsacSetRho(node, r)


def set_sat_links(sat, up):
    """Bring all of the satellite's links down (handover outage) or back up."""
    ip = sat.GetObject[ns.Ipv4]()
    for i in range(1, ip.GetNInterfaces()):  # skip loopback (0)
        if up:
            ip.SetUp(i)
        else:
            ip.SetDown(i)


def sample_next_hop():
    """Sample the source's next hop toward dst (uniform across all protocols)."""
    if HAVE_SAMPLER:
        g_nexthopSeq.append(str(ns.AanetNextHop(g_srcNode, g_dstAddr, g_srcAddr)))


# ---------------------------------------------------------------------------
# pure-python route metrics from the sampled next-hop sequence (unit-testable)
# ---------------------------------------------------------------------------
def route_metrics(seq, sample_int, sim_time):
    if not seq:
        return 0.0, 0.0
    reroutes, runs, run = 0, [], 1
    for i in range(1, len(seq)):
        if seq[i] != seq[i - 1]:
            reroutes += 1
            runs.append(run)
            run = 1
        else:
            run += 1
    runs.append(run)
    mean_run = sum(runs) / len(runs)
    route_life_s = mean_run * sample_int
    reroute_per_s = reroutes / sim_time if sim_time else 0.0
    return route_life_s, reroute_per_s


# ---------------------------------------------------------------------------
def reset_state():
    """make a run in a reused process identical to a run in a fresh one"""
    global g_allNodes, g_srcNode, g_dstAddr, g_srcAddr
    g_schedule.clear(); g_nexthopSeq.clear(); g_pyEvents.clear(); g_rho.clear()
    g_allNodes = g_srcNode = g_dstAddr = g_srcAddr = None
    if HAVE_SAMPLER:
        ns.AanetResetAll()
    ns.RngSeedManager.ResetNextStreamIndex()
    ns.Ipv4AddressGenerator.Reset()


def run_once(argv):
    """run one simulation in this process; returns its stdout"""
    import io, contextlib
    reset_state()
    buf = io.StringIO()
    old = sys.argv
    sys.argv = ["aanet_sim.py"] + list(argv)
    try:
        with contextlib.redirect_stdout(buf):
            main()
    finally:
        sys.argv = old
    return buf.getvalue()


def main():
    global g_allNodes, g_srcNode, g_dstAddr, g_srcAddr

    ap = argparse.ArgumentParser(description="UCRAAN AANET ns-3 python simulator")
    ap.add_argument("--mobility", default="mobility.csv")
    ap.add_argument("--output", default="results.csv")   # accepted for parity
    ap.add_argument("--routes", default="")              # empty => OLSR baseline
    ap.add_argument("--meshRange", type=float, default=120000.0)
    ap.add_argument("--ntnRange", type=float, default=1500000.0)
    ap.add_argument("--satAlt", type=float, default=600000.0)
    ap.add_argument("--geoScale", type=float, default=1.0)
    ap.add_argument("--appRate", default="10Mbps")
    ap.add_argument("--src", type=int, default=0)
    ap.add_argument("--dst", type=int, default=0)        # 0 => last aircraft
    ap.add_argument("--simTime", type=float, default=0.0)  # 0 => auto
    ap.add_argument("--handoverPeriod", type=float, default=0.0)  # 0 => off
    ap.add_argument("--handoverDur", type=float, default=5.0)
    ap.add_argument("--sampleInt", type=float, default=2.0)
    ap.add_argument("--arpDeadTimeout", type=float, default=2.0,
                    help="ARP DEAD hold-down (s). ns-3 default is 100 s, which lets a "
                         "single failed resolution blackhole a neighbour for the rest "
                         "of a 120 s run; real stacks re-probe within ~1-3 s")
    ap.add_argument("--rhoUniform", type=float, default=0.0,
                    help="ISAC sensing share of every aircraft's frame (fixed split)")
    ap.add_argument("--rhoFile", default="",
                    help="per-slot per-aircraft sensing shares time,node,rho (joint split)")
    ap.add_argument("--isacFrame", type=float, default=0.01, help="ISAC frame period (s)")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    # ns-3 Wi-Fi MAC only ACKs unicast over a few km. Aircraft links are 10s-100s
    # of km, so shrink the whole world by geoScale: relative geometry (and thus
    # the topology + its dynamics) is preserved exactly; only absolute
    # distances/delays change.
    mesh_range = args.meshRange * args.geoScale
    ntn_range = args.ntnRange * args.geoScale
    sat_alt = args.satAlt * args.geoScale
    print(f"geoScale={args.geoScale}  meshRange={mesh_range}m  "
          f"ntnRange={ntn_range}m  satAlt={sat_alt}m")

    # ---- load trajectories, create the three tiers ------------------------
    ids, traj, traj_max = read_mobility(args.mobility)
    n_air = len(ids)
    if n_air < 2:
        sys.exit("Need >= 2 aircraft in mobility.csv")
    sim_time = args.simTime if args.simTime > 0.0 else traj_max + 1.0
    dst_idx = args.dst if args.dst != 0 else n_air - 1
    src_idx = args.src

    aircraft = ns.NodeContainer()
    aircraft.Create(n_air)          # middle tier: the AANET mesh
    infra = ns.NodeContainer()
    infra.Create(2)                 # infra.Get(0)=satellite, infra.Get(1)=ground
    sat_node = infra.Get(0)
    ground_node = infra.Get(1)

    all_nodes = ns.NodeContainer()
    all_nodes.Add(aircraft)
    all_nodes.Add(infra)

    print(f"5G/6G NTN environment: {n_air} aircraft + 1 LEO sat + 1 ground")
    print(f"simTime={sim_time}s  src={src_idx}  dst={dst_idx}")

    # ---- mobility: aircraft fly real trajectories; sat & ground fixed -----
    air_mob = ns.MobilityHelper()
    air_mob.SetMobilityModel("ns3::WaypointMobilityModel")
    air_mob.Install(aircraft)
    for i in range(n_air):
        wp = aircraft.Get(i).GetObject[ns.WaypointMobilityModel]()
        last_t = None
        for (t, x, y, z) in traj[ids[i]]:
            if last_t is not None and t <= last_t:
                continue  # WaypointMobilityModel needs strictly increasing times
            wp.AddWaypoint(ns.Waypoint(
                ns.Seconds(t),
                ns.Vector(x * args.geoScale, y * args.geoScale, z * args.geoScale)))
            last_t = t

    fixed_mob = ns.MobilityHelper()
    pa = ns.CreateObject[ns.ListPositionAllocator]()
    pa.Add(ns.Vector(0, 0, sat_alt))  # satellite overhead -> realistic LEO delay
    pa.Add(ns.Vector(0, 0, 0))        # ground station
    fixed_mob.SetPositionAllocator(pa)
    fixed_mob.SetMobilityModel("ns3::ConstantPositionMobilityModel")
    fixed_mob.Install(infra)

    # ---- radios -----------------------------------------------------------
    wifi = ns.WifiHelper()
    wifi.SetStandard(ns.WIFI_STANDARD_80211a)
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", ns.StringValue("OfdmRate54Mbps"),
                                 "ControlMode", ns.StringValue("OfdmRate6Mbps"))
    mac = ns.WifiMacHelper()
    mac.SetType("ns3::AdhocWifiMac")

    # (1) A2A airborne mesh: link exists iff within mesh_range
    mesh_chan = ns.YansWifiChannelHelper()
    mesh_chan.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel")
    mesh_chan.AddPropagationLoss("ns3::RangePropagationLossModel",
                                 "MaxRange", ns.DoubleValue(mesh_range))
    mesh_phy = ns.YansWifiPhyHelper()
    mesh_channel = mesh_chan.Create()
    if HAVE_SAMPLER:
        ns.AanetAttachGate(mesh_channel, mesh_range)   # range cut-off + ISAC dwell gate
    mesh_phy.SetChannel(mesh_channel)
    mesh_phy.Set("TxPowerStart", ns.DoubleValue(46.0))
    mesh_phy.Set("TxPowerEnd", ns.DoubleValue(46.0))
    mesh_dev = wifi.Install(mesh_phy, mac, aircraft)

    # (2) NTN inter-tier (5G NR NTN) as a BENT-PIPE STAR: each aircraft <-> sat
    # is a dedicated P2P link; sat <-> ground is another. Aircraft reach each
    # other via the satellite ONLY through the satellite, so a handover that
    # drops the satellite truly cuts the inter-tier path. P2P sidesteps Wi-Fi's
    # long-range ACK limit, so this tier needs no geoScale (real 2ms delay).
    p2p = ns.PointToPointHelper()
    p2p.SetDeviceAttribute("DataRate", ns.StringValue("50Mbps"))
    p2p.SetChannelAttribute("Delay", ns.StringValue("2ms"))
    sat_links = []
    for i in range(n_air):
        sat_links.append(p2p.Install(aircraft.Get(i), sat_node))
    ground_link = p2p.Install(sat_node, ground_node)

    ns.Config.SetDefault("ns3::ArpCache::DeadTimeout",
                         ns.TimeValue(ns.Seconds(args.arpDeadTimeout)))

    # ---- IP + routing (OLSR over BOTH tiers unless --routes) --------------
    olsr = ns.OlsrHelper()
    static_routing = ns.Ipv4StaticRoutingHelper()
    list_routing = ns.Ipv4ListRoutingHelper()
    list_routing.Add(static_routing, 0)     # UCRAAN routes -> priority 0
    if not args.routes:
        list_routing.Add(olsr, 10)          # baseline: OLSR over both tiers
    internet = ns.InternetStackHelper()
    internet.SetRoutingHelper(list_routing)
    internet.Install(all_nodes)
    if not args.routes and HAVE_SAMPLER:
        ns.AanetHookOlsr()   # count OLSR control packets at the protocol Tx trace

    ip = ns.Ipv4AddressHelper()
    ip.SetBase(ns.Ipv4Address("10.1.0.0"), ns.Ipv4Mask("255.255.0.0"))
    mesh_if = ip.Assign(mesh_dev)
    if HAVE_SAMPLER:
        for i in range(n_air):
            ns.AanetIsacAdd(mesh_dev.Get(i))
            ns.AanetIsacSetRho(i, args.rhoUniform)
    # satellite star: link i uses 10.3.i.0/24 (aircraft=.1, satellite=.2)
    for i in range(n_air):
        ip.SetBase(ns.Ipv4Address(f"10.3.{i}.0"), ns.Ipv4Mask("255.255.255.0"))
        ip.Assign(sat_links[i])
    ip.SetBase(ns.Ipv4Address("10.4.0.0"), ns.Ipv4Mask("255.255.255.0"))
    ip.Assign(ground_link)

    # ---- satellite handover outages (cuts the whole inter-tier star) ------
    if args.handoverPeriod > 0.0:
        th = args.handoverPeriod
        while th < sim_time:
            py_event(th, set_sat_links, sat_node, False)
            py_event(th + args.handoverDur, set_sat_links, sat_node, True)
            th += args.handoverPeriod

    # ---- load UCRAAN/delay routes and schedule per-slot installs ----------
    if args.routes:
        with open(args.routes, newline="") as rf:
            reader = csv.reader(rf)
            next(reader, None)  # header: time,node,dest_ip,nexthop_ip
            for row in reader:
                if len(row) < 4:
                    continue
                slot = float(row[0])
                g_schedule.setdefault(slot, []).append(
                    (int(row[1]), ns.Ipv4Address(row[2].strip()),
                     ns.Ipv4Address(row[3].strip())))
        g_allNodes = all_nodes
        for slot in g_schedule:
            py_event(slot, apply_routes, slot)
        print(f"UCRAAN mode: {len(g_schedule)} route slots loaded")

    # ---- ISAC sensing dwell (time split) ----------------------------------
    if args.rhoFile:
        with open(args.rhoFile, newline="") as rf:
            rd = csv.reader(rf); next(rd, None)
            for row in rd:
                g_rho.setdefault(float(row[0]), []).append((int(row[1]), float(row[2])))
        for slot in g_rho:
            py_event(slot, apply_rho, slot)
    if HAVE_SAMPLER and (args.rhoFile or args.rhoUniform > 0):
        ns.AanetIsacStart(args.isacFrame, 0.0)

    # ---- UDP flow, source aircraft -> destination aircraft ----------------
    port = 9
    dst_addr = mesh_if.GetAddress(dst_idx)
    print(f"source -> {dst_addr} (dst mesh IP)")
    onoff = ns.OnOffHelper("ns3::UdpSocketFactory",
                           ns.InetSocketAddress(dst_addr, port).ConvertTo())
    onoff.SetConstantRate(ns.DataRate(args.appRate), 1024)
    onoff.SetAttribute("StartTime", ns.TimeValue(ns.Seconds(2.0)))
    onoff.SetAttribute("StopTime", ns.TimeValue(ns.Seconds(sim_time - 1.0)))
    onoff.Install(aircraft.Get(src_idx))

    sink = ns.PacketSinkHelper("ns3::UdpSocketFactory",
                               ns.InetSocketAddress(ns.Ipv4Address.GetAny(),
                                                    port).ConvertTo())
    dst_app = sink.Install(aircraft.Get(dst_idx))
    dst_app.Start(ns.Seconds(0.0))
    dst_app.Stop(ns.Seconds(sim_time))

    # ---- measurement ------------------------------------------------------
    fm = ns.FlowMonitorHelper()
    monitor = fm.InstallAll()

    # sample source's next hop toward dst every sampleInt seconds
    g_srcNode = aircraft.Get(src_idx)
    g_dstAddr = dst_addr
    g_srcAddr = mesh_if.GetAddress(src_idx)
    ts = 2.0
    while ts < sim_time:
        py_event(ts, sample_next_hop)
        ts += args.sampleInt

    # DEBUG (best-effort): dump the source node's routing table mid-run
    if args.debug:
        t_dump = min(sim_time * 0.3, 60.0)
        try:
            osw = ns.CreateObject[ns.OutputStreamWrapper](
                "aanet-route-dump.txt", ns.cppyy.gbl.std.ios.out)
            ns.Ipv4RoutingHelper.PrintRoutingTableAt(
                ns.Seconds(t_dump), aircraft.Get(src_idx), osw)
            print(f"debug: routing table at t={t_dump}s -> aanet-route-dump.txt")
        except Exception as ex:
            print(f"debug route dump unavailable: {ex}", file=sys.stderr)

    run_with_py_events(sim_time)

    # ---- FlowMonitor: data delivery + on-air control overhead -------------
    monitor.CheckForLostPackets()
    classifier = fm.GetClassifier()
    tx = rx = lost = rx_bytes = ctrl_pkts = 0
    delay_sum = jitter_sum = 0.0
    for flow_id, st in monitor.GetFlowStats():
        is_ctrl = False
        try:
            ft = classifier.FindFlow(flow_id)
            is_ctrl = (ft.destinationPort == 698)  # OLSR control traffic
        except Exception:
            is_ctrl = False
        if is_ctrl:
            ctrl_pkts += st.txPackets
            continue  # don't count control as data
        tx += st.txPackets
        rx += st.rxPackets
        lost += st.lostPackets
        rx_bytes += st.rxBytes
        delay_sum += st.delaySum.GetSeconds()
        jitter_sum += st.jitterSum.GetSeconds()

    pdr = rx / tx if tx else 0.0
    mean_delay_ms = 1000.0 * delay_sum / rx if rx else 0.0
    jitter_ms = 1000.0 * jitter_sum / (rx - 1) if rx > 1 else 0.0
    thr_kbps = 8.0 * rx_bytes / 1000.0 / sim_time if sim_time else 0.0
    if not args.routes and HAVE_SAMPLER:
        ctrl_pkts = int(ns.AanetOlsrTxCount())
    elif args.routes:
        # solver-based variants: injected-route update volume (changed entries)
        prev, ctrl_pkts = {}, 0
        for slot in sorted(g_schedule):
            if slot >= sim_time:
                break
            cur = {(nd, str(dip)): str(nh) for nd, dip, nh in g_schedule[slot]}
            ctrl_pkts += sum(1 for k, v in cur.items() if prev.get(k) != v)
            ctrl_pkts += sum(1 for k in prev if k not in cur)
            prev = cur
    ctrl_pps = ctrl_pkts / sim_time if sim_time else 0.0

    route_life_s, reroute_per_s = route_metrics(
        g_nexthopSeq, args.sampleInt, sim_time)

    # exact same RESULT line as the C++ so run_experiments.py's regex matches
    print(f"---- RESULT ----  tx={int(tx)}  rx={int(rx)}  PDR={pdr}"
          f"  meanDelay={mean_delay_ms}  jitter={jitter_ms}"
          f"  throughput={thr_kbps}  lost={int(lost)}"
          f"  routeLife={route_life_s}  reroutePerS={reroute_per_s}"
          f"  ctrlPps={ctrl_pps}")
    if HAVE_SAMPLER:
        print(f"ISAC mean sensing share per aircraft = "
              f"{float(ns.AanetIsacDwell()) / (n_air * sim_time):.4f}")

    ns.Simulator.Destroy()


if __name__ == "__main__":
    main()
