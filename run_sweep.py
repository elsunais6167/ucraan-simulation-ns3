#!/usr/bin/env python3
# ============================================================================
#  run_sweep.py  -- the headline UCRAAN experiment
#
#  1. Auto-finds a source/destination pair that is MULTI-HOP in the mesh (so
#     routing choices actually matter).
#  2. Sweeps two stress axes: sensing error x satellite-handover frequency.
#  3. For each cell runs three protocols in ns-3:
#        olsr      - baseline ad-hoc routing
#        ucraan_eo - UCRAAN ignoring uncertainty (expectation only)
#        ucraan    - full uncertainty-aware UCRAAN
#  4. Writes one tidy results CSV (no plot).
#
#  Run from your ns-3 root:
#    python3 run_sweep.py \
#       --mobility scratch/aanet/data/cluster_mobility.csv \
#       --binary scratch/aanet/aanet-sim \
#       --solver ucraan_solver.py \
#       --geoScale 0.0005 --simTime 120 --mesh-range 200000 \
#       --sensing 0.2 0.5 1.0 2.0 4.0 \
#       --ho-period 1000 60 30 15 \
#       --out sweep_results.csv
# ============================================================================

import argparse
import collections
import os
import re
import subprocess
import tempfile

import numpy as np
import pandas as pd
import networkx as nx

RESULT_RE = re.compile(
    r"tx=(\d+)\s+rx=(\d+)\s+PDR=([\d.eE+-]+)\s+meanDelay=([\d.eE+-]+)\s+"
    r"jitter=([\d.eE+-]+)\s+throughput=([\d.eE+-]+)\s+lost=(\d+)\s+"
    r"routeLife=([\d.eE+-]+)\s+reroutePerS=([\d.eE+-]+)\s+ctrlPps=([\d.eE+-]+)")


# ---- geometry helpers ------------------------------------------------------
def load_positions(mobility):
    df = pd.read_csv(mobility)
    ids = list(dict.fromkeys(df["icao24"]))
    idx = {a: i for i, a in enumerate(ids)}
    tr = {}
    for a, g in df.groupby("icao24", sort=False):
        g = g.sort_values("t")
        tr[idx[a]] = (g["t"].values, g["x"].values, g["y"].values, g["z"].values)
    return len(ids), tr, df["t"].min(), df["t"].max()


def pos(tr, i, t):
    ts, xs, ys, zs = tr[i]
    if t < ts[0] or t > ts[-1]:
        return None
    return np.array([np.interp(t, ts, xs), np.interp(t, ts, ys), np.interp(t, ts, zs)])


def find_pair(n, tr, tmin, tmax, mesh_range, slot=5.0):
    """Pick the pair whose ACTUAL routed path has its worst (bottleneck) link
    near the range edge most often. That bottleneck is the link a noisy sensing
    estimate can flip in/out of range -- where delay-optimal gets fooled into a
    phantom link and UCRAAN hedges. Scoring the path bottleneck (not the direct
    distance) is what makes the two protocols separate across the whole sweep.
    """
    slots = np.arange(tmin, tmax, slot)
    R = mesh_range
    lo, hi = 0.85 * R, 1.30 * R         # near-edge / just-past-edge band
    edge = collections.Counter()        # slots the path bottleneck is in-band
    seen = collections.Counter()        # slots both planes are airborne
    hopsum = collections.defaultdict(list)
    for t in slots:
        P = {i: pos(tr, i, t) for i in range(n)}
        present = [i for i in range(n) if P[i] is not None]
        # graph over links up to the sensing-consideration range (1.3R), so a
        # bottleneck that drifts just past R is still on a candidate path
        G = nx.Graph()
        G.add_nodes_from(present)
        for a in present:
            for b in present:
                if a < b:
                    d = np.linalg.norm(P[a] - P[b])
                    if d <= hi:
                        G.add_edge(a, b, d=d)
        for a in present:
            for b in present:
                if a < b:
                    seen[(a, b)] += 1
                    if nx.has_path(G, a, b):
                        # the min-delay path, exactly what delay_opt would take
                        pth = nx.shortest_path(G, a, b, weight="d")
                        bott = max(G[pth[i]][pth[i + 1]]["d"]
                                   for i in range(len(pth) - 1))
                        hopsum[(a, b)].append(len(pth) - 1)
                        if lo <= bott <= hi:
                            edge[(a, b)] += 1
    if not seen:
        raise SystemExit("No co-present pairs found; check mobility / --mesh-range.")
    # Rank pairs by (multi-hop?, edge fraction, median hops). A 1-hop pair gives
    # the protocols nothing to decide, so multi-hop is a hard preference; among
    # multi-hop pairs prefer the one whose bottleneck hugs the range edge most
    # (that's where sensing noise separates the protocols), tie-broken by hops.
    cand = []
    for pair, s in seen.items():
        if s < 0.5 * len(slots):
            continue
        ef = edge[pair] / s
        mh = float(np.median(hopsum[pair])) if hopsum[pair] else 1.0
        cand.append((mh >= 2, ef, mh, pair))
    if not cand:  # nothing co-present half the time -> take the most co-present
        pair = max(seen, key=lambda p: seen[p])
        mh = float(np.median(hopsum[pair])) if hopsum[pair] else 1.0
        return pair[0], pair[1], 0.0, mh
    multi, ef, mh, pair = max(cand)
    return pair[0], pair[1], ef, mh


# ---- subprocess wrappers ---------------------------------------------------
def gen_routes(solver, mobility, src, dst, se, ho, ho_dur, mode, mesh_range,
               t_max, out):
    cmd = (f"python3 {solver} --mobility {mobility} --src {src} --dst {dst} "
           f"--sensing-error {se} --ho-period {ho} --ho-width {ho_dur} "
           f"--mode {mode} --mesh-range {mesh_range} --t-max {t_max} --out {out}")
    p = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if p.returncode != 0:
        print(f"  !! solver failed ({mode} se={se} ho={ho}):\n{p.stderr[-2000:]}")
        return False
    return True


def run_sim(ns3, binary, args_str, timeout):
    cmd = f'{ns3} run "{binary} {args_str}"'
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"  !! sim timed out after {timeout}s: {cmd}")
        return None
    m = RESULT_RE.search(p.stdout + p.stderr)
    if not m:
        tail = (p.stdout + p.stderr)[-2000:]
        print(f"  !! no RESULT line from sim (rc={p.returncode}). Output tail:\n{tail}")
        return None
    return dict(tx=int(m.group(1)), rx=int(m.group(2)), pdr=float(m.group(3)),
                delay=float(m.group(4)), jitter=float(m.group(5)),
                throughput=float(m.group(6)), lost=int(m.group(7)),
                route_life=float(m.group(8)), reroute_per_s=float(m.group(9)),
                ctrl_pps=float(m.group(10)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mobility", required=True)
    ap.add_argument("--binary", default="scratch/aanet/aanet-sim")
    ap.add_argument("--solver", default="ucraan_solver.py")
    ap.add_argument("--ns3", default="./ns3")
    ap.add_argument("--geoScale", type=float, default=0.0005)
    ap.add_argument("--simTime", type=float, default=120)
    ap.add_argument("--mesh-range", type=float, default=200000)
    ap.add_argument("--sensing", type=float, nargs="+",
                    default=[0.2, 0.5, 1.0, 2.0, 4.0])
    ap.add_argument("--ho-period", type=float, nargs="+",
                    default=[1000.0, 60.0, 30.0, 15.0])
    ap.add_argument("--ho-dur", type=float, default=5.0,
                    help="seconds the satellite is down per handover; passed to "
                         "BOTH the sim (--handoverDur) and the solver (--ho-width) "
                         "so the solver's risk model matches reality")
    ap.add_argument("--app-rate", default="2Mbps",
                    help="offered UDP load; keep well under multi-hop 802.11a "
                         "capacity so delay/PDR reflect routing, not congestion")
    ap.add_argument("--src", type=int, default=-1, help="-1 = auto-find")
    ap.add_argument("--dst", type=int, default=-1)
    ap.add_argument("--timeout", type=int, default=1800)
    ap.add_argument("--out", default="sweep_results.csv")
    args = ap.parse_args()

    # 1) choose the src/dst pair
    n, tr, tmin, tmax = load_positions(args.mobility)
    if args.src < 0 or args.dst < 0:
        src, dst, ef, mh = find_pair(n, tr, tmin, tmax, args.mesh_range)
        print(f"auto pair: src={src} dst={dst}  routed path's bottleneck link is "
              f"near the edge {ef*100:.0f}% of the time, median {mh:.0f} hops")
        if ef < 0.05:
            print("  note: no pair's route really hugs the range edge in this "
                  "fleet, so the sensing-error axis may stay flat. Try a different "
                  "cluster or adjust --mesh-range toward the typical hop distance.")
    else:
        src, dst = args.src, args.dst
        print(f"using given pair src={src} dst={dst}")

    common = (f"--mobility={args.mobility} --geoScale={args.geoScale} "
              f"--simTime={args.simTime} --meshRange={args.mesh_range} "
              f"--handoverDur={args.ho_dur} --appRate={args.app_rate} "
              f"--src={src} --dst={dst} --output=/dev/null")
    tmp = tempfile.mkdtemp()
    rows = []

    def record(protocol, se, ho, r):
        if r is None:
            print(f"  !! no result for {protocol} se={se} ho={ho}")
            return
        rows.append(dict(protocol=protocol, sensing_error=se, ho_period=ho,
                         handovers=int(args.simTime // ho) if ho < args.simTime else 0,
                         tx=r["tx"], rx=r["rx"], pdr=r["pdr"], delay_ms=r["delay"],
                         jitter_ms=r["jitter"], throughput_kbps=r["throughput"],
                         lost=r["lost"], route_lifetime_s=r["route_life"],
                         reroute_per_s=r["reroute_per_s"], ctrl_pkts_per_s=r["ctrl_pps"]))
        pd.DataFrame(rows).to_csv(args.out, index=False)  # checkpoint each run
        print(f"  {protocol:10s} se={se:<4} ho={ho:<5} PDR={r['pdr']:.4f} "
              f"delay={r['delay']:.2f}ms life={r['route_life']:.0f}s "
              f"reroute/s={r['reroute_per_s']:.3f} ctrl_pps={r['ctrl_pps']:.2f}")

    for ho in args.ho_period:
        print(f"== handover period {ho}s ==")
        # OLSR is sensing-invariant: run once per handover, replicate across se
        r_olsr = run_sim(args.ns3, args.binary, f"{common} --handoverPeriod={ho}",
                         args.timeout)
        for se in args.sensing:
            record("olsr", se, ho, r_olsr)
            for mode, name in [("delay", "delay_opt"), ("ucraan", "ucraan")]:
                rf = os.path.join(tmp, f"r_{name}_{se}_{ho}.csv")
                if not gen_routes(args.solver, args.mobility, src, dst, se, ho,
                                  args.ho_dur, mode, args.mesh_range,
                                  args.simTime, rf):
                    continue
                r = run_sim(args.ns3, args.binary,
                            f"{common} --handoverPeriod={ho} --routes={rf}", args.timeout)
                record(name, se, ho, r)

    print(f"\ndone -> {args.out}  ({len(rows)} runs)")


if __name__ == "__main__":
    main()