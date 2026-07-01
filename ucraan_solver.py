#!/usr/bin/env python3
# ============================================================================
#  ucraan_solver.py
#
#  The "brain" of UCRAAN. ns-3 moves the planes and forwards packets; THIS
#  script decides the routes, using uncertainty the way your paper describes.
#
#  It runs ONCE before the ns-3 run and produces routes.csv. The four blocks
#  map directly to your methodology:
#     7.2  ISAC sensing + EKF tracking  -> per-link covariance
#     7.3  uncertainty -> link-survival probability
#     7.4  cross-layer, risk-adjusted edge cost
#     7.5  chance-constrained shortest path  -> the chosen route
#
#  USAGE (defaults match aanet-sim.cc):
#     python3 ucraan_solver.py --mobility aanet/data/mobility.csv \
#             --src 0 --dst -1 --sensing-error 1.0 --eps 0.1
#
#  Sweep the central hypothesis by changing --sensing-error (0.2 .. 4.0).
#  Run --mode eo for the "expectation-only" ablation (ignores variance).
# ============================================================================

import argparse
import math
import numpy as np
import pandas as pd
import networkx as nx
from scipy.stats import norm

C = 3.0e8  # speed of light (m/s)


# ----------------------------------------------------------------------------
# IP addressing — MUST match the scheme in aanet-sim.cc
#   mesh:  aircraft i -> 10.1.0.(i+1)
#   ntn :  aircraft i -> 10.2.0.(i+1);  satellite -> 10.2.0.(nAir+1)
# ----------------------------------------------------------------------------
def ip(base_octets, offset):
    n = (base_octets[0] << 24) + (base_octets[1] << 16) + \
        (base_octets[2] << 8) + base_octets[3] + offset
    return f"{(n >> 24) & 255}.{(n >> 16) & 255}.{(n >> 8) & 255}.{n & 255}"

def mesh_ip(i):       return ip([10, 1, 0, 0], i + 1)
# satellite bent-pipe star: link i is 10.3.i.0/24 (aircraft=.1, satellite=.2)
def sat_air_ip(i):    return f"10.3.{i}.1"   # aircraft i's side of its sat link
def sat_sat_ip(i):    return f"10.3.{i}.2"   # satellite's side of link to aircraft i


# ----------------------------------------------------------------------------
# Link budget (Friis): distance <-> SNR.  Used to turn geometry into link
# quality, and to find the "threshold distance" beyond which a link drops.
# ----------------------------------------------------------------------------
class LinkBudget:
    def __init__(self, fc_hz, ptx_dbm, gain_dbi, noise_dbm):
        self.lam = C / fc_hz
        self.const = ptx_dbm + 2 * gain_dbi - noise_dbm  # tx+rx gains
    def snr_db(self, d):
        d = max(d, 1.0)
        fspl = 20 * math.log10(4 * math.pi * d / self.lam)
        return self.const - fspl
    def threshold_distance(self, snr_thr_db):
        # distance where snr_db(d) == snr_thr_db
        fspl = self.const - snr_thr_db
        return (self.lam / (4 * math.pi)) * (10 ** (fspl / 20.0))


# ----------------------------------------------------------------------------
# 7.2  Per-link EKF-style tracker.  State = [distance, closing-rate].
# Produces a predicted distance and its variance tau seconds ahead.
# This is the "sensing tracker covariance" your paper relies on.
# ----------------------------------------------------------------------------
class LinkTracker:
    # State = [range, closing-rate]. Constant-velocity model with a proper
    # KINEMATIC (white-acceleration) process noise. This keeps the filter
    # responsive: under noisy ranging it still tracks the closing motion instead
    # of lagging behind and confidently keeping a link that has already broken.
    # The accel-noise level sigma_a sets how aggressively the filter adapts its
    # rate estimate (and how fast its predictive variance grows with the horizon).
    def __init__(self, d0, sigma_a):
        self.x = np.array([d0, 0.0])          # [distance, rate]
        self.P = np.diag([1e4, 1e4])          # init: pos std 100 m, rate std 100 m/s
        self.H = np.array([[1.0, 0.0]])       # we measure distance
        self.qa = sigma_a ** 2                # acceleration-noise PSD, (m/s^2)^2

    def _Q(self, dt):
        # discrete white-acceleration process noise for a [pos, vel] state
        return self.qa * np.array([[dt**4 / 4, dt**3 / 2],
                                   [dt**3 / 2, dt**2]])

    def step(self, d_meas, r_meas, dt):
        F = np.array([[1, dt], [0, 1]])       # constant-velocity transition
        # predict
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)
        # update with the noisy ISAC range
        S = self.H @ self.P @ self.H.T + r_meas
        K = (self.P @ self.H.T) / S
        self.x = self.x + (K.flatten() * (d_meas - (self.H @ self.x)[0]))
        self.P = self.P - np.outer(K, self.H @ self.P)

    def predict_ahead(self, tau):
        F = np.array([[1, tau], [0, 1]])
        x = F @ self.x
        P = F @ self.P @ F.T + self._Q(tau)
        return x[0], math.sqrt(max(P[0, 0], 1e-6))  # predicted mean range, predictive std


# ----------------------------------------------------------------------------
# Load mobility.csv and give interpolated positions at any time t.
# ----------------------------------------------------------------------------
class Fleet:
    def __init__(self, csv):
        df = pd.read_csv(csv)
        self.ids = list(dict.fromkeys(df["icao24"]))  # first-seen order
        self.idx = {a: i for i, a in enumerate(self.ids)}
        self.n = len(self.ids)
        self.tr = {}
        for a, g in df.groupby("icao24", sort=False):
            g = g.sort_values("t")
            self.tr[a] = (g["t"].values, g["x"].values, g["y"].values, g["z"].values)
        self.t_min = df["t"].min()
        self.t_max = df["t"].max()

    def pos(self, i, t):
        a = self.ids[i]
        ts, xs, ys, zs = self.tr[a]
        if t < ts[0] or t > ts[-1]:
            return None  # aircraft not present at this time
        return np.array([np.interp(t, ts, xs),
                         np.interp(t, ts, ys),
                         np.interp(t, ts, zs)])


# ----------------------------------------------------------------------------
# Satellite accessibility (the 5G NTN inter-tier link).  A bent-pipe relay
# that can bridge ANY two aircraft, but with a quantified accessibility/
# handover risk that dips during periodic handover windows (7.2, satellite).
# ----------------------------------------------------------------------------
def sat_access_prob(t, base=0.97, period=30.0, dip=0.25, width=5.0):
    phase = t % period
    if phase < width:                      # handover window
        return base - dip
    return base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mobility", default="mobility.csv")
    ap.add_argument("--out", default="routes.csv")
    ap.add_argument("--src", type=int, default=0)
    ap.add_argument("--dst", type=int, default=-1)      # -1 => last aircraft
    ap.add_argument("--slot", type=float, default=1.0,  # route RECOMPUTE interval
                    help="route recompute/refresh interval (s); small => fresh "
                         "routes, little staleness. Independent of --tau.")
    ap.add_argument("--tau", type=float, default=2.0,   # survival LOOKAHEAD horizon
                    help="survival lookahead horizon (s): how far ahead a link must "
                         "stay up to be trusted (~ commitment window + margin).")
    ap.add_argument("--mesh-range", type=float, default=200000.0)
    ap.add_argument("--carrier", type=float, default=5e9)   # A2A carrier (Hz)
    ap.add_argument("--snr-thr", type=float, default=3.0)   # min usable SNR dB
    ap.add_argument("--eps", type=float, default=0.1)       # 1-eps reliability
    ap.add_argument("--risk-weight", type=float, default=8.0)
    ap.add_argument("--sensing-error", type=float, default=1.0,  # SWEPT axis
                    help="scales ISAC ranging uncertainty (1=nominal, higher=worse)")
    ap.add_argument("--range-sigma0", type=float, default=180.0,
                    help="baseline 1-sigma ISAC ranging error (m) at sensing-error=1, "
                         "scaled by range and by --sensing-error")
    ap.add_argument("--sigma-a", type=float, default=40.0,
                    help="EKF acceleration-noise level (m/s^2); higher = more "
                         "responsive tracker / faster-growing predictive variance")
    ap.add_argument("--seed", type=int, default=0,
                    help="RNG seed for sensing noise; share across modes for a "
                         "controlled comparison")
    ap.add_argument("--ho-period", type=float, default=1000.0,  # SWEPT axis
                    help="seconds between satellite handover windows (large=calm)")
    ap.add_argument("--ho-dip", type=float, default=0.25,
                    help="satellite accessibility drop during a handover window")
    ap.add_argument("--mode", choices=["ucraan", "delay"], default="ucraan",
                    help="ucraan=uncertainty-aware chance-constrained; "
                         "delay=delay-optimal shortest path on point-estimate state")
    ap.add_argument("--sat-alt", type=float, default=600000.0,
                    help="LEO altitude (m); sets NTN bent-pipe latency. MUST match "
                         "satAlt in aanet-sim.cc")
    ap.add_argument("--sat-proc-ms", type=float, default=1.5,
                    help="satellite transponder processing delay (ms). MUST match "
                         "aanet-sim.cc")
    ap.add_argument("--mesh-hop-ms", type=float, default=0.4,
                    help="per-hop mesh delay ns-3 actually measures (802.11a MAC+tx); "
                         "dominates mesh latency in the geoScaled world")
    args = ap.parse_args()

    np.random.seed(args.seed)   # reproducible sensing noise (share across modes)

    fleet = Fleet(args.mobility)
    n = fleet.n
    dst = args.dst if args.dst >= 0 else n - 1
    src = args.src
    lb = LinkBudget(fc_hz=args.carrier, ptx_dbm=46, gain_dbi=10, noise_dbm=-90)

    # A mesh link "survives" exactly when the two planes stay within mesh range
    # -- which is precisely when ns-3 keeps the link up. Calibrating to ns-3's
    # own cutoff is what makes uncertainty-awareness meaningful.
    d_thr = args.mesh_range

    # NTN bent-pipe one-way delay (aircraft -> sat -> aircraft), kept consistent
    # with aanet-sim.cc: two aircraft<->sat hops, each = prop(altitude) +
    # transponder. This is deliberately MORE than a healthy multi-hop mesh path,
    # so a delay-aware router prefers the mesh while OLSR's hop-count metric grabs
    # the slower 2-hop relay.
    sat_one_way_ms = (args.sat_alt / C) * 1e3 + args.sat_proc_ms
    sat_delay_ms = 2.0 * sat_one_way_ms
    
    trackers = {}        # (a,b) -> LinkTracker, persistent across slots
    last_seen = {}       # (a,b) -> float (timestamp of last update)

    rows = []
    slots = np.arange(fleet.t_min, fleet.t_max, args.slot)

    for t in slots:
        # geometry at this slot
        P = {i: fleet.pos(i, t) for i in range(n)}
        present = [i for i in range(n) if P[i] is not None]

        G = nx.DiGraph()
        G.add_nodes_from(present)
        est = {}   # (a,b) -> noisy estimated distance (the "point estimate")

        # ---- build candidate edges -----------------------------------------
        for a in present:
            for b in present:
                if a == b:
                    continue
                true_d = np.linalg.norm(P[a] - P[b])

                # mesh edge: consider links near the range edge too, so the
                # sensing estimate can disagree with reality there
                if true_d <= args.mesh_range * 1.4:
                    # 7.2 ISAC ranging: a sane CRLB-style model. The 1-sigma error
                    # grows with range (weaker echo at distance) and with the swept
                    # sensing-error factor. At the range edge it is
                    # ~range_sigma0 * sensing_error * 2 -- tens of m (good sensing)
                    # to ~km (degraded). That is the regime where a single noisy
                    # point estimate flips a link in/out of range while a filter +
                    # hedge can still tell the difference.
                    range_sigma = args.range_sigma0 * args.sensing_error * \
                                  (1.0 + true_d / args.mesh_range)
                    r_meas = max(range_sigma ** 2, 1.0)
                    d_meas = true_d + np.random.normal(0, range_sigma)
                    est[(a, b)] = d_meas

                    key = (a, b)
                    if key not in trackers:
                        trackers[key] = LinkTracker(d_meas, args.sigma_a)
                        dt = args.slot
                    else:
                        dt = t - last_seen[key] # Calculate true elapsed time
                    
                    trackers[key].step(d_meas, r_meas, dt)
                    last_seen[key] = t
                    
                    mu_d, sd_d = trackers[key].predict_ahead(args.tau)
                    # Delay ns-3 actually measures on a mesh hop is MAC/tx-dominated
                    # (geoScale makes propagation negligible). Model it as a per-hop
                    # constant plus a tiny distance term for tie-breaking, so the
                    # solver's notion of "fast" matches the simulator's stopwatch.
                    delay_ms = args.mesh_hop_ms + true_d / C * 1e3

                    if args.mode == "ucraan":
                        # 7.3 link survival over the lookahead horizon tau. Use the
                        # EKF *predictive* std (measurement noise + process/velocity
                        # uncertainty propagated tau ahead) -- NOT the raw measurement
                        # noise. This is what makes the sensing-error axis actually
                        # drive hedging: noisier sensing -> larger predictive variance
                        # -> lower survival probability for near-edge links.
                        s = max(sd_d, 1e-3)
                        p_surv = float(norm.cdf((d_thr - mu_d) / s))

                        # liveness gate on the FILTERED distance (robust to noise),
                        # plus a coin-flip floor on predicted survival.
                        d_filt = trackers[key].x[0]
                        if d_filt > d_thr or p_surv < 0.50:
                            continue

                        cost = delay_ms + args.risk_weight * (-math.log(max(p_surv, 1e-6)))
                        G.add_edge(a, b, tier="mesh", p=p_surv, delay=delay_ms, weight=cost)
                    else:
                        # delay: naive point estimate, no uncertainty model.
                        # Include the link iff the (noisy) estimate says in-range.
                        if d_meas > d_thr:
                            continue
                        G.add_edge(a, b, tier="mesh", p=1.0, delay=delay_ms, weight=delay_ms)

                # satellite bridge edge (5G NTN), available to all pairs
                p_sat = sat_access_prob(t, period=args.ho_period, dip=args.ho_dip)
                if args.mode == "ucraan":
                    cost_sat = sat_delay_ms + args.risk_weight * (-math.log(max(p_sat, 1e-6)))
                else:
                    cost_sat = sat_delay_ms            # baselines ignore sat risk
                
                if (not G.has_edge(a, b)) or cost_sat < G[a][b]["weight"]:
                    G.add_edge(a, b, tier="sat", p=p_sat, delay=sat_delay_ms, weight=cost_sat)

        # ---- choose the path per protocol ----------------------------------
        if src not in G or dst not in G:
            continue
        path, tiers = None, None

        if args.mode == "delay":
            if not nx.has_path(G, src, dst):
                continue
            path = nx.shortest_path(G, src, dst, weight="weight")  # delay-optimal
            tiers = [G[path[i]][path[i + 1]]["tier"] for i in range(len(path) - 1)]

        else:  # ucraan: 7.5 chance-constrained shortest path
            if not nx.has_path(G, src, dst):
                continue
            rw = args.risk_weight
            cand = nx.shortest_path(G, src, dst, weight="weight")
            for _ in range(6):
                psurv = np.prod([G[cand[i]][cand[i + 1]]["p"] for i in range(len(cand) - 1)])
                if psurv >= 1 - args.eps:
                    path = cand
                    break
                rw *= 2.0
                for u, v, dta in G.edges(data=True):
                    dta["weight"] = dta["delay"] + rw * (-math.log(max(dta["p"], 1e-6)))
                cand = nx.shortest_path(G, src, dst, weight="weight")
            if path is None:
                path = cand
            tiers = [G[path[i]][path[i + 1]]["tier"] for i in range(len(path) - 1)]

        # ---- turn the path into ns-3 host routes ---------------------------
        dst_ip = mesh_ip(dst)
        for i in range(len(path) - 1):
            a, b = path[i], path[i + 1]
            if tiers[i] == "mesh":
                rows.append([round(t, 3), a, dst_ip, mesh_ip(b)])
            else:  # satellite bridge a -> sat -> b (bent-pipe star)
                rows.append([round(t, 3), a, dst_ip, sat_sat_ip(a)])
                rows.append([round(t, 3), n, dst_ip, sat_air_ip(b)])

    out = pd.DataFrame(rows, columns=["time", "node", "dest_ip", "nexthop_ip"])
    out.to_csv(args.out, index=False)
    print(f"wrote {len(out)} route entries across {out['time'].nunique()} slots "
          f"to {args.out}  (mode={args.mode}, sensing_error={args.sensing_error})")

if __name__ == "__main__":
    main()