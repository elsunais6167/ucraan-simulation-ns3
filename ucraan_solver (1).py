#!/usr/bin/env python3
# ============================================================================
#  ucraan_solver.py   (file 2 of 4) -- the UCRAAN algorithm. Called by run_experiments.py;
#  can also be run alone to write routes.csv + routes.csv.rho for one src-dst pair.
#
#  ucraan_solver.py  (v2 -- tuned)
#
#  The "brain" of UCRAAN. ns-3 moves the planes and forwards packets; THIS
#  script decides the routes. Runs once before the ns-3 run, writes routes.csv.
#
#     S3.3.2  ISAC sensing (CRLB-scaled) + per-link Kalman tracking
#     S3.3.3  predictive distribution -> window link-survival probability
#     S3.3.4  cross-layer risk-adjusted edge cost (mesh AND satellite)
#     S3.3.5  chance-constrained shortest path (+ route-commitment hysteresis)
#
#  Modes
#     ucraan : full uncertainty-aware model
#     eo     : expectation-only ablation (same tracker, variance ignored)
#     delay  : delay-optimal shortest path on the raw point estimate (Cui 2021)
#     rl     : value-network greedy forwarding on the raw point estimate
#
#  v2 changes vs v1 (see Chapter 3 / diagnosis notes)
#   [B1] satellite edge was silently dropped for every pair near the range
#        edge (a `continue` in the mesh block skipped it)  -> fixed, ALL modes
#   [B2] satellite is now an explicit graph vertex, so a path can traverse the
#        bent-pipe at most once (two traversals installed two conflicting host
#        routes on the satellite -> routing loop in ns-3)  -> fixed, ALL modes
#   [B3] noise is seeded per (seed, slot, link) -> identical realisation in
#        every mode (true paired design)                   -> fixed, ALL modes
#   [U1] tracker: white-acceleration Q(sigma_a), rate initialised with a large
#        prior, actual inter-measurement dt, re-initialised after gaps
#   [U2] survival uses the tracker's PREDICTIVE sigma_d(tau) (Eq. 3.7/3.11),
#        and must hold over the whole commitment window [t, t+tau]
#   [U3] satellite accessibility uses the (ephemeris-predictable) handover
#        schedule: p_sat = availability fraction over the window
#   [U4] route-commitment hysteresis: keep the current path while it still
#        meets the chance constraint and is within (1+h) of the best cost
#   [U5] chance-constraint fallback = most-reliable path when infeasible
#  Every U-change can be switched off for ablation (--legacy-*, --hyst 0).
#  Defaults = values tuned on training traces 1-3 (tune.py); evaluated on 4-6.
#
#  v3: JOINT SENSING-COMMUNICATION RESOURCE ALLOCATION (6G ISAC time split)
#   Every aircraft i spends a share rho_i of each ISAC frame on dedicated sensing
#   and 1-rho_i on data. Both endpoints of a link range each other, so the fused
#   ranging variance scales as rho_ref/(rho_a+rho_b); a hop can carry data only
#   while both ends are out of their dwell, so its airtime share is 1-max(rho_a,rho_b).
#   --rho-mode uniform : every aircraft uses the budget rho_bar (fixed split)
#   --rho-mode joint   : UCRAAN distributes the same total budget n*rho_bar each
#                        slot by value of sensing, keeping traffic-carrying
#                        aircraft free for data, and re-routes with the new split.
# ============================================================================

import argparse
import hashlib
import math
import numpy as np
import pandas as pd
import networkx as nx
from scipy.stats import norm

C = 3.0e8
SAT = "S"          # satellite vertex id in the routing graph


# ---------------------------------------------------------------------------
# IP addressing -- MUST match aanet_sim
# ---------------------------------------------------------------------------
def mesh_ip(i):    return f"10.1.0.{i + 1}"
def sat_air_ip(i): return f"10.3.{i}.1"
def sat_sat_ip(i): return f"10.3.{i}.2"


class LinkBudget:
    def __init__(self, fc_hz, ptx_dbm, gain_dbi, noise_dbm):
        self.lam = C / fc_hz
        self.const = ptx_dbm + 2 * gain_dbi - noise_dbm

    def snr_db(self, d):
        d = max(d, 1.0)
        return self.const - 20 * math.log10(4 * math.pi * d / self.lam)


# ---------------------------------------------------------------------------
# [U1] per-link Kalman tracker, state [range, range-rate]
# ---------------------------------------------------------------------------
class LinkTracker:
    def __init__(self, z, r, t, sigma_a, rate_prior=300.0):
        self.x = np.array([z, 0.0])
        self.P = np.diag([r, rate_prior ** 2])
        self.t = t
        self.qa = sigma_a ** 2

    def Q(self, dt):
        return self.qa * np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]])

    def step(self, z, r, t):
        dt = t - self.t
        F = np.array([[1.0, dt], [0.0, 1.0]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q(dt)
        S = self.P[0, 0] + r
        K = self.P[:, 0] / S
        self.x = self.x + K * (z - self.x[0])
        self.P = self.P - np.outer(K, self.P[0, :])
        self.t = t

    def predict(self, tau):
        F = np.array([[1.0, tau], [0.0, 1.0]])
        x = F @ self.x
        P = F @ self.P @ F.T + self.Q(tau)
        return x[0], math.sqrt(max(P[0, 0], 1e-6))


# v1 tracker kept verbatim for the ablation (--legacy-tracker)
class LegacyTracker:
    def __init__(self, z, r, t, *_):
        self.x = np.array([z, 0.0]); self.P = np.diag([1e4, 1e2]); self.dt = None; self.t = t

    def step(self, z, r, t):
        dt = self.dt or max(t - self.t, 1e-3); self.dt = dt
        F = np.array([[1, dt], [0, 1]]); Q = np.diag([1.0, 0.5]) * dt
        self.x = F @ self.x; self.P = F @ self.P @ F.T + Q
        S = self.P[0, 0] + r; K = self.P[:, 0] / S
        self.x = self.x + K * (z - self.x[0]); self.P = self.P - np.outer(K, self.P[0, :])
        self.t = t

    def predict(self, tau):
        dt = self.dt or 1.0
        F = np.array([[1, tau], [0, 1]]); x = F @ self.x
        P = F @ self.P @ F.T + np.diag([1.0, 0.5]) * dt * (tau / dt)
        return x[0], math.sqrt(max(P[0, 0], 1e-6))


class Fleet:
    def __init__(self, csv):
        df = pd.read_csv(csv)
        self.ids = list(dict.fromkeys(df["icao24"]))
        self.n = len(self.ids)
        self.tr = {}
        for i, a in enumerate(self.ids):
            g = df[df.icao24 == a].sort_values("t")
            self.tr[i] = (g["t"].values, g["x"].values, g["y"].values, g["z"].values)
        self.t_min, self.t_max = df["t"].min(), df["t"].max()

    def pos(self, i, t):
        ts, xs, ys, zs = self.tr[i]
        if t < ts[0] or t > ts[-1]:
            return None
        return np.array([np.interp(t, ts, xs), np.interp(t, ts, ys), np.interp(t, ts, zs)])


# ---------------------------------------------------------------------------
# satellite accessibility
# ---------------------------------------------------------------------------
def sat_outage_fraction(t0, t1, period, dur):
    """fraction of [t0,t1] covered by scheduled outages [kT, kT+dur), k>=1"""
    if period <= 0 or t1 <= t0:
        return 0.0
    cov = 0.0
    k = max(1, int(math.floor(t0 / period)))
    while k * period < t1:
        a, b = k * period, k * period + dur
        cov += max(0.0, min(b, t1) - max(a, t0))
        k += 1
    return cov / (t1 - t0)


def sat_access_prob(t, args):
    if args.legacy_sat:   # v1: instantaneous phase, 25% dip (includes t<T)
        return args.p0 - (args.ho_dip if (t % args.ho_period) < 3.0 else 0.0)
    # [U3] availability over the commitment window, from the known schedule
    frac = sat_outage_fraction(t, t + args.tau, args.ho_period, args.ho_dur)
    return max(args.p0 * (1.0 - frac), 1e-4)


def noise(seed, slot_idx, a, b):
    h = hashlib.blake2b(f"{seed}|{slot_idx}|{a}|{b}".encode(), digest_size=8).digest()
    return np.random.default_rng(int.from_bytes(h, "little")).standard_normal()


# ---------------------------------------------------------------------------
def _Phi(x):
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


def solve(args, fleet=None, log_calib=None):
    """single src->dst pair (CLI path)"""
    fleet = fleet or Fleet(args.mobility)
    dst = args.dst if args.dst >= 0 else fleet.n - 1
    res = solve_multi(args, fleet, [(args.src, dst)], log_calib)
    return res[(args.src, dst)], fleet.n, dst


def solve_multi(args, fleet, pairs, log_calib=None):
    """paths only (backward-compatible API)"""
    return solve_multi2(args, fleet, pairs, log_calib)[0]


def solve_multi2(args, fleet, pairs, log_calib=None):
    """Returns (paths, rho). With a uniform split the sensing graph is
    pair-independent and is built once for all pairs; with the joint split the
    allocation depends on each pair's route, so each pair is solved on its own."""
    if args.rho_mode == "joint" and args.mode in ("ucraan", "eo") and len(pairs) > 1:
        P, Rh = {}, {}
        for pr in pairs:
            p, r = _solve_core(args, fleet, [pr], log_calib)
            P.update(p); Rh.update(r)
        return P, Rh
    return _solve_core(args, fleet, pairs, log_calib)


def _solve_core(args, fleet, pairs, log_calib=None):
    n = fleet.n
    lb = LinkBudget(args.carrier, 46, 10, -90)
    snr_edge = 10 ** (lb.snr_db(args.mesh_range) / 10)
    R = args.mesh_range
    trackers, last_seen = {}, {}
    TrackerCls = LegacyTracker if args.legacy_tracker else LinkTracker
    t_end = min(fleet.t_max, args.t_end) if args.t_end > 0 else fleet.t_max
    slots = np.arange(fleet.t_min, t_end, args.slot)
    out = {pr: {} for pr in pairs}
    rho_out = {pr: {} for pr in pairs}
    prev = {pr: None for pr in pairs}
    lim = -math.log(1 - args.eps)
    joint = args.rho_mode == "joint" and args.mode in ("ucraan", "eo")
    rho_bar = args.rho_budget
    if args.mode == "genie":
        rho_bar = args.rho_min      # perfect information needs no sensing airtime
    rho = np.full(n, rho_bar)       # split in force during the interval before this slot

    for si, t in enumerate(slots):
        P = {i: fleet.pos(i, t) for i in range(n)}
        present = [i for i in range(n) if P[i] is not None]
        G = nx.DiGraph()
        G.add_nodes_from(present)
        G.add_node(SAT)
        est = {}
        p_sat = sat_access_prob(t, args)
        for a in present:
            # ---- [B1][B2] satellite legs exist for EVERY aircraft ----------
            if args.mode == "genie":   # evaluation-only upper bound: perfect knowledge
                frac = sat_outage_fraction(t, t + args.slot, args.ho_period, args.ho_dur)
                s_risk, s_d = (0.0, 0.0) if frac == 0 else (1e3, 0.0)
            elif args.mode in ("ucraan", "eo"):
                s_risk = -math.log(p_sat)
            else:
                s_risk = 0.0                     # baselines ignore satellite risk
            G.add_edge(a, SAT, tier="up", p=p_sat, delay=args.sat_delay / 2, risk=s_risk)
            G.add_edge(SAT, a, tier="down", p=1.0, delay=args.sat_delay / 2, risk=0.0)

            for b in present:
                if a == b:
                    continue
                true_d = float(np.linalg.norm(P[a] - P[b]))
                if true_d > R * args.sense_range:
                    continue
                # ---- ISAC ranging, CRLB-scaled (sigma grows as SNR falls) --
                snr = 10 ** (lb.snr_db(true_d) / 10)
                sig_r = args.sigma0 * args.sensing_error * math.sqrt(snr_edge / snr) \
                    * math.sqrt(args.rho_ref / max(rho[a] + rho[b], 1e-3))
                r_meas = max(sig_r ** 2, 1.0)
                z = true_d + sig_r * noise(args.seed, si, a, b)
                est[(a, b)] = z
                delay = args.hop_delay + true_d / C * 1e3

                if args.mode == "genie":
                    ok = all(fleet.pos(a, t + u) is not None and fleet.pos(b, t + u) is not None and
                             np.linalg.norm(fleet.pos(a, t + u) - fleet.pos(b, t + u)) <= R
                             for u in np.linspace(0, args.slot, 6))
                    if ok:
                        G.add_edge(a, b, tier="mesh", p=1.0, delay=delay, risk=0.0)
                    continue
                if args.mode in ("delay", "rl"):
                    if z <= R:
                        G.add_edge(a, b, tier="mesh", p=1.0, delay=delay, risk=0.0)
                    continue

                # ---- tracking (ucraan / eo) --------------------------------
                key = (a, b)
                gap = t - last_seen.get(key, -1e9)
                if key not in trackers or gap > args.max_gap:
                    trackers[key] = TrackerCls(z, r_meas, t, args.sigma_a)
                else:
                    trackers[key].step(z, r_meas, t)
                last_seen[key] = t
                trk = trackers[key]
                if args.legacy_survival:          # v1: filtered tau-mean, raw meas sigma
                    mu_w, _ = trk.predict(args.tau)
                    sd = math.sqrt(r_meas) + 1.0
                else:                             # [U2] window survival, predictive sigma
                    mu_t, s_t = trk.predict(args.tau)
                    mu_w = max(trk.x[0], mu_t)
                    sd = math.sqrt(s_t ** 2 + args.sigma_floor ** 2) * args.sigma_infl
                p = (1.0 if mu_w <= R else 0.0) if args.mode == "eo" else _Phi((R - mu_w) / sd)
                if log_calib is not None and args.mode == "ucraan":
                    alive = all(fleet.pos(a, t + u) is not None and fleet.pos(b, t + u) is not None and
                                np.linalg.norm(fleet.pos(a, t + u) - fleet.pos(b, t + u)) <= R
                                for u in np.linspace(0, args.tau, 5))
                    log_calib.append((p, alive, t, a, b))
                if p < args.p_min:
                    continue
                G.add_edge(a, b, tier="mesh", p=p, delay=delay, risk=-math.log(max(p, 1e-9)))

        for pr in pairs:
            src, dst = pr
            if joint:
                # block-coordinate "joint" step: route under the split in force,
                # allocate the budget for that route, re-route under the new split
                _set_air(args, G, rho)
                path = _select(args, G, P, present, est, src, dst, prev[pr], lim, R)
                new_rho = allocate_sensing(args, G, trackers, t, present, path, n)
                _set_air(args, G, new_rho)
                path = _select(args, G, P, present, est, src, dst, prev[pr], lim, R)
                new_rho = allocate_sensing(args, G, trackers, t, present, path, n)
            else:
                _set_air(args, G, rho)
                path = _select(args, G, P, present, est, src, dst, prev[pr], lim, R)
                new_rho = np.full(n, rho_bar)
            out[pr][float(t)] = path
            rho_out[pr][float(t)] = new_rho.copy()
            prev[pr] = path
        if len(pairs) == 1:
            rho = rho_out[pairs[0]][float(t)]   # measurements at the next slot use this split
    return out, rho_out


# ---------------------------------------------------------------------------
# communication cost of the sensing dwell
# ---------------------------------------------------------------------------
def _set_air(args, G, rho):
    """annotate mesh edges with airtime share, dwell wait and air cost"""
    T_ms = args.isac_frame * 1e3
    for u, v, d in G.edges(data=True):
        if d["tier"] != "mesh":
            continue
        m = max(rho[u], rho[v])
        d["share"] = 1.0 - m
        d["dwell_ms"] = m * m * T_ms / 2.0          # mean wait for an arrival during a dwell
        d["air"] = args.load_mbps / (args.cap_mbps * max(1.0 - m, 1e-3))


def path_utilisation(args, G, path, P):
    """max over hops of the shared-channel utilisation (clique bound):
    hops whose endpoints are within range of each other share airtime"""
    hops = [(path[i], path[i + 1]) for i in range(len(path) - 1)
            if SAT not in (path[i], path[i + 1]) and G.has_edge(path[i], path[i + 1])]
    if not hops:
        return 0.0
    R = args.mesh_range

    def near(h, k):
        return any(np.linalg.norm(P[x] - P[y]) <= R for x in h for y in k)
    return max(sum(G[k[0]][k[1]]["air"] for k in hops if k == h or near(h, k)) for h in hops)


# ---------------------------------------------------------------------------
# JOINT SENSING-COMMUNICATION ALLOCATION
# ---------------------------------------------------------------------------
def allocate_sensing(args, G, trackers, t, present, path, n):
    """Distribute the network sensing budget n*rho_bar across aircraft.
    maximise   sum_e V_e * log(rho_a + rho_b)            (information gain)
               - w_c * sum_{hops h in route} air_h(rho)   (airtime taken from data)
    s.t.       sum_i rho_i = n*rho_bar, rho_min <= rho_i <= rho_max,
               route utilisation <= eta
    V_e = relevance(e) x uncertainty(e): relevance is 1 for hops of the route,
    r_alt for links touching route aircraft or the destination (detours),
    r_bg for every other tracked link; uncertainty is 4p(1-p) of the survival
    probability predicted over a longer look-ahead (links about to become
    critical are sensed before they are critical), plus a floor.
    Greedy water-filling in steps of 0.01 (the objective is concave per node)."""
    rho = np.full(n, args.rho_min)
    budget = n * args.rho_budget - rho.sum()
    on = set()
    hops = []
    if path is not None:
        hops = [(path[i], path[i + 1]) for i in range(len(path) - 1)
                if SAT not in (path[i], path[i + 1])]
        on = {x for h in hops for x in h}
    dst = path[-1] if path else None
    # value of sensing per undirected link
    V = {}
    for (a, b), trk in trackers.items():
        if a not in present or b not in present or trk.t < t - 1e-9:
            continue
        key = (min(a, b), max(a, b))
        mu0 = trk.x[0]
        mu_l, s_l = trk.predict(args.value_horizon)
        p = _Phi((args.mesh_range - max(mu0, mu_l)) / (s_l * args.sigma_infl))
        unc = 4.0 * p * (1.0 - p) + args.value_floor
        if (a, b) in hops or (b, a) in hops:
            rel = 1.0
        elif a in on or b in on or dst in (a, b):
            rel = args.r_alt
        else:
            rel = args.r_bg
        V[key] = max(V.get(key, 0.0), rel * unc)
    inc = {i: [] for i in range(n)}
    for (a, b), v in V.items():
        inc[a].append((b, v)); inc[b].append((a, v))
    step = 0.01
    L, C = args.load_mbps, args.cap_mbps

    def air_cost(i, r_i):
        c = 0.0
        for (a, b) in hops:
            if i in (a, b):
                m = max(r_i, rho[b if i == a else a])
                c += L / (C * max(1.0 - m, 1e-3))
        return c

    while budget > 1e-9:
        best, bi = -1e18, None
        for i in present:
            if rho[i] + step > args.rho_max + 1e-12:
                continue
            g = sum(v * (math.log(rho[i] + rho[j] + step) - math.log(rho[i] + rho[j]))
                    for j, v in inc[i])
            c = args.w_comm * (air_cost(i, rho[i] + step) - air_cost(i, rho[i]))
            if hops and i in on:
                # hard guard: never push the route's utilisation past eta
                trial = rho.copy(); trial[i] += step
                u = sum(L / (C * max(1.0 - max(trial[a], trial[b]), 1e-3)) for a, b in hops)
                if u > args.eta and u > sum(
                        L / (C * max(1.0 - max(rho[a], rho[b]), 1e-3)) for a, b in hops):
                    c += 1e9
            if g - c > best:
                best, bi = g - c, i
        if bi is None:
            break
        if best <= 0:
            # remaining budget has no positive use: park it on aircraft that carry
            # no traffic (it costs the route nothing), same total as the fixed split
            idle = [i for i in present if i not in on and rho[i] + step <= args.rho_max + 1e-12]
            if not idle:
                break
            bi = min(idle, key=lambda i: rho[i])
        rho[bi] += step
        budget -= step
    return rho


def _select(args, G, P, present, est, src, dst, prev_path, lim, R):
    if src not in G or dst not in G:
        return None
    if args.mode == "rl":
        def value(j):
            return np.linalg.norm(P[j] - P[dst]) / C * 1e3
        cur, pth, seen = src, [src], {src}
        for _ in range(len(present)):
            if cur == dst:
                break
            nbrs = [j for j in present if j != cur and (cur, j) in est and est[(cur, j)] <= R
                    and j not in seen]
            prog = [j for j in nbrs if value(j) < value(cur)]
            choices = prog if prog else nbrs
            if not choices:
                break
            nxt = min(choices, key=lambda j: est[(cur, j)] / C * 1e3 + value(j))
            pth.append(nxt); seen.add(nxt); cur = nxt
        return pth if cur == dst else [src, SAT, dst]

    if not nx.has_path(G, src, dst):
        return None
    if args.mode in ("delay", "genie"):
        return nx.shortest_path(G, src, dst, weight=lambda u, v, d: d["delay"] + d["risk"])

    # ucraan / eo : chance-constrained shortest path (Lagrangian weight doubling)
    risk_of = lambda pth: sum(G[pth[i]][pth[i + 1]]["risk"] for i in range(len(pth) - 1))
    w = args.risk_weight
    wa = args.w_air
    wf = lambda ww: (lambda u, v, d: d["delay"] + d.get("dwell_ms", 0.0) + wa * d.get("air", 0.0)
                     + ww * d["risk"])
    cost_of = lambda pth, ww: sum(wf(ww)(pth[i], pth[i + 1], G[pth[i]][pth[i + 1]])
                                  for i in range(len(pth) - 1))
    path = None
    cand = nx.shortest_path(G, src, dst, weight=wf(w))
    for _ in range(args.iters):
        if risk_of(cand) <= lim:
            path = cand; break
        w *= 2.0
        cand = nx.shortest_path(G, src, dst, weight=wf(w))
    if path is None:
        # [U5] infeasible -> most reliable path (w -> inf)
        path = cand if args.legacy_fallback else \
            nx.shortest_path(G, src, dst, weight=lambda u, v, d: d["risk"] + 1e-6 * d["delay"])
    # [S2] capacity awareness: if the route would saturate the shared channel,
    # choose among {route, satellite, least-airtime route} by expected goodput
    if args.load_mbps > 0:
        def goodput(pth):
            pr = math.exp(-risk_of(pth))
            u = path_utilisation(args, G, pth, P)
            return pr * min(1.0, 1.0 / u) if u > 0 else pr
        if path_utilisation(args, G, path, P) > args.eta:
            cands = [path]
            if G.has_edge(src, SAT) and G.has_edge(SAT, dst):
                cands.append([src, SAT, dst])
            try:
                cands.append(nx.shortest_path(G, src, dst, weight=lambda u, v, d: d.get("air", 0.0)
                                              + 5.0 * d["risk"] + 1e-3 * d["delay"]))
            except nx.NetworkXNoPath:
                pass
            path = max(cands, key=goodput)
    # [U4] hysteresis: keep previous path if still admissible & near-optimal
    if args.hyst > 0 and prev_path is not None and \
            all(G.has_edge(prev_path[i], prev_path[i + 1]) for i in range(len(prev_path) - 1)):
        if risk_of(prev_path) <= max(lim, risk_of(path)) and \
                cost_of(prev_path, w) <= (1 + args.hyst) * cost_of(path, w):
            path = prev_path
    return path


def to_rows(out_paths, n, dst):
    rows = []
    dip = mesh_ip(dst)
    for t, path in out_paths.items():
        if path is None:
            continue
        for i in range(len(path) - 1):
            a, b = path[i], path[i + 1]
            if a == SAT:
                rows.append([round(t, 3), n, dip, sat_air_ip(b)])
            elif b == SAT:
                rows.append([round(t, 3), a, dip, sat_sat_ip(a)])
            else:
                rows.append([round(t, 3), a, dip, mesh_ip(b)])
    return pd.DataFrame(rows, columns=["time", "node", "dest_ip", "nexthop_ip"])


def as_oracle_paths(out_paths):
    """convert to (nodes, tiers) format used by the oracle: (nodes, tiers) with 'sat' hops"""
    res = {}
    for t, p in out_paths.items():
        if p is None:
            res[t] = None; continue
        nodes, tiers, i = [p[0]], [], 0
        while i < len(p) - 1:
            if p[i + 1] == SAT:
                nodes.append(p[i + 2]); tiers.append("sat"); i += 2
            else:
                nodes.append(p[i + 1]); tiers.append("mesh"); i += 1
        res[t] = (nodes, tiers)
    return res


def build_parser():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mobility", default="mobility.csv")
    ap.add_argument("--out", default="routes.csv")
    ap.add_argument("--src", type=int, default=0)
    ap.add_argument("--dst", type=int, default=-1)
    ap.add_argument("--mode", choices=["ucraan", "eo", "delay", "rl", "genie"], default="ucraan")
    ap.add_argument("--seed", type=int, default=1, help="sensing-noise seed (shared across modes)")
    ap.add_argument("--slot", type=float, default=1.0)       # route recompute interval (s)
    ap.add_argument("--tau", type=float, default=1.0)        # commitment / survival horizon (s)
    ap.add_argument("--t-end", type=float, default=0.0)      # stop solving after this time (0=all)
    ap.add_argument("--mesh-range", type=float, default=150000.0)
    ap.add_argument("--sense-range", type=float, default=1.4)  # ISAC tracks up to 1.4 R
    ap.add_argument("--carrier", type=float, default=5e9)
    ap.add_argument("--sensing-error", type=float, default=1.0)   # kappa (swept)
    ap.add_argument("--sigma0", type=float, default=2500.0,  # 1-sigma ranging error at range edge, kappa=1
                    help="ranging std (m) at the range edge for kappa=1; scales with 1/sqrt(SNR)")
    ap.add_argument("--sigma-a", type=float, default=10.0)    # tracker accel noise (m/s^2)
    ap.add_argument("--sigma-floor", type=float, default=0.0)
    ap.add_argument("--sigma-infl", type=float, default=1.5)
    ap.add_argument("--max-gap", type=float, default=10.0)
    ap.add_argument("--eps", type=float, default=0.05)
    ap.add_argument("--risk-weight", type=float, default=8.0)
    ap.add_argument("--iters", type=int, default=6)
    ap.add_argument("--p-min", type=float, default=0.2)
    ap.add_argument("--hyst", type=float, default=0.1)
    ap.add_argument("--hop-delay", type=float, default=0.5)  # ms MAC/processing per mesh hop
    ap.add_argument("--sat-delay", type=float, default=4.3)  # ms bent-pipe (2 legs)
    ap.add_argument("--ho-period", type=float, default=1000.0)
    ap.add_argument("--ho-dur", type=float, default=5.0)
    ap.add_argument("--ho-dip", type=float, default=0.25)    # legacy model only
    ap.add_argument("--p0", type=float, default=0.97)
    # ---- sensing-communication resource (ISAC time split) ----
    ap.add_argument("--rho-mode", choices=["uniform", "joint"], default="uniform")
    ap.add_argument("--rho-budget", type=float, default=0.1, help="mean sensing share per aircraft")
    ap.add_argument("--rho-min", type=float, default=0.01)
    ap.add_argument("--rho-max", type=float, default=0.6)
    ap.add_argument("--rho-ref", type=float, default=0.2,
                    help="combined endpoint dwell at which sigma0 is specified (0.1 + 0.1)")
    ap.add_argument("--isac-frame", type=float, default=0.01)
    ap.add_argument("--load-mbps", type=float, default=6.0)
    ap.add_argument("--cap-mbps", type=float, default=25.0, help="single-hop data capacity")
    ap.add_argument("--eta", type=float, default=0.9, help="max route utilisation")
    ap.add_argument("--w-air", type=float, default=1.0, help="ms-equivalent per unit airtime")
    ap.add_argument("--w-comm", type=float, default=10.0, help="allocator weight on airtime taken (tuned)")
    ap.add_argument("--r-alt", type=float, default=0.5)
    ap.add_argument("--r-bg", type=float, default=0.05)
    ap.add_argument("--value-horizon", type=float, default=3.0)
    ap.add_argument("--value-floor", type=float, default=0.05)
    ap.add_argument("--legacy-tracker", action="store_true")
    ap.add_argument("--legacy-survival", action="store_true")
    ap.add_argument("--legacy-sat", action="store_true")
    ap.add_argument("--legacy-fallback", action="store_true")
    return ap


def main():
    args = build_parser().parse_args()
    fleet = Fleet(args.mobility)
    dst = args.dst if args.dst >= 0 else fleet.n - 1
    P, Rh = solve_multi2(args, fleet, [(args.src, dst)])
    paths, n = P[(args.src, dst)], fleet.n
    out = to_rows(paths, n, dst)
    out.to_csv(args.out, index=False)
    rows = [(round(t, 3), i, round(float(r[i]), 4)) for t, r in Rh[(args.src, dst)].items() for i in range(n)]
    pd.DataFrame(rows, columns=["time", "node", "rho"]).to_csv(args.out + ".rho", index=False)
    print(f"wrote {len(out)} route entries across {out['time'].nunique()} slots to {args.out} "
          f"(mode={args.mode}, kappa={args.sensing_error}, seed={args.seed})")


if __name__ == "__main__":
    main()
