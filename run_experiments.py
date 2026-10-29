#!/usr/bin/env python3
# ============================================================================
#  run_experiments.py  (file 4 of 4)  --  step 2: experiments, statistics, tables, figures
#
#  THE FOUR FILES
#    prepare_data.py    flight data (OpenSky ADS-B, or --synthetic) -> data/cluster_mobility_<k>.csv
#    ucraan_solver.py   UCRAAN route solver + joint sensing-communication allocation
#                       (also delay-optimal, value-network, expectation-only, genie modes)
#    aanet_sim.py       ns-3 three-tier AANET simulator (Wi-Fi A2A mesh with ISAC dwell,
#                       LEO bent-pipe with handover outages, OLSR baseline)
#    run_experiments.py this file
#
#  QUICK START
#    pip install numpy pandas scipy networkx matplotlib ns3      # ns3 = official ns-3 python wheel
#    python3 prepare_data.py --input states_2022-06-27-12.csv.tar --bbox 40 62 -60 -5 --n-traces 6
#    python3 run_experiments.py all
#  or step by step:
#    python3 run_experiments.py pairs                 # which src-dst pairs will be simulated
#    python3 run_experiments.py oracle                # fast packet-free evaluation, all pairs (+ genie bound)
#    python3 run_experiments.py headline              # ns-3: kappa x T_ho at budget 0.1   (Table 3.5, Fig 3.6)
#    python3 run_experiments.py budget                # ns-3: sensing-budget sweep         (Table 3.6, Fig 3.8)
#    python3 run_experiments.py calibration           # ECE of survival probabilities      (Eq. 3.21)
#    python3 run_experiments.py analyse               # tables (csv + md), Wilcoxon tests, figures
#  Runs are checkpointed and RESUMED: re-running a command skips finished runs.
#  Outputs: results/*.csv   results/tables/*.md|csv   results/figures/*.png
#
#  PROTOCOLS (labels used in every output)
#    olsr        OLSR over both tiers                       (uniform sensing split, unused)
#    delay_opt   delay-optimal shortest path, point estimate (Cui et al. 2021 model)
#    rl_value    value-network greedy forwarding, point estimate
#    ucraan      UCRAAN with a fixed (uniform) sensing split   [ablation]
#    ucraan_jsc  UCRAAN with joint sensing-communication allocation   [proposed]
#    genie       perfect information, oracle only            [upper bound]
# ============================================================================
import argparse
import glob
import itertools
import json
import os
import re
import subprocess
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

NS3_PROTOCOLS = ["olsr", "delay_opt", "rl_value", "ucraan", "ucraan_jsc"]
SOLVER_MODE = {"delay_opt": ("delay", []), "rl_value": ("rl", []), "ucraan": ("ucraan", []),
               "ucraan_jsc": ("ucraan", ["--rho-mode", "joint"]), "ucraan_eo": ("eo", []),
               "genie": ("genie", [])}
LABEL = {"ucraan_jsc": "UCRAAN-JSC", "ucraan": "UCRAAN (fixed split)", "delay_opt": "Delay-optimal",
         "rl_value": "Value-network", "olsr": "OLSR", "genie": "Genie (upper bound)", "ucraan_eo": "UCRAAN-EO"}
RESULT_RE = re.compile(
    r"tx=(\d+)\s+rx=(\d+)\s+PDR=([\d.eE+-]+)\s+meanDelay=([\d.eE+-]+)\s+jitter=([\d.eE+-]+)\s+"
    r"throughput=([\d.eE+-]+)\s+lost=(\d+)\s+routeLife=([\d.eE+-]+)\s+reroutePerS=([\d.eE+-]+)\s+ctrlPps=([\d.eE+-]+)")
RESULT_KEYS = ["tx", "rx", "pdr", "delay_ms", "jitter_ms", "throughput_kbps", "lost", "route_lifetime_s",
               "reroute_per_s", "ctrl_pkts_per_s"]
KEY = ["trace", "src", "dst", "protocol", "sensing_error", "ho_period", "rho_budget"]


# =============================================================== pair choice
def load_positions(mobility):
    df = pd.read_csv(mobility)
    ids = list(dict.fromkeys(df["icao24"]))
    tr = {}
    for i, a in enumerate(ids):
        g = df[df.icao24 == a].sort_values("t")
        tr[i] = (g["t"].values, g["x"].values, g["y"].values, g["z"].values)
    return len(ids), tr


def pos(tr, i, t):
    ts, xs, ys, zs = tr[i]
    if t < ts[0] or t > ts[-1]:
        return None
    return np.array([np.interp(t, ts, xs), np.interp(t, ts, ys), np.interp(t, ts, zs)])


def find_pairs(mobility, R, t1, k=2, band=0.15, step=2.0):
    """Chapter 3, Section 3.7: rank pairs by the share of the SIMULATED window in
    which the min-delay route needs a link within +-15 % of the range edge (bonus
    for partitioned time); exclude always-adjacent and mostly satellite-only
    pairs; choose k pairs that share no endpoint. The range is never rescaled."""
    import networkx as nx
    n, tr = load_positions(mobility)
    score = {p: [0, 0, 0, 0] for p in itertools.combinations(range(n), 2)}
    for t in np.arange(0.0, t1, step):
        P = {i: pos(tr, i, t) for i in range(n)}
        G = nx.Graph(); G.add_nodes_from(range(n))
        for i, j in itertools.combinations(range(n), 2):
            if P[i] is None or P[j] is None:
                continue
            d = np.linalg.norm(P[i] - P[j])
            if d <= (1 + band) * R:
                G.add_edge(i, j, d=d, w=0.5 + d / 3e5)
        for (a, b), s in score.items():
            s[1] += 1
            if P[a] is not None and P[b] is not None and np.linalg.norm(P[a] - P[b]) <= (1 - band) * R:
                s[2] += 1; continue
            if not nx.has_path(G, a, b):
                s[3] += 1; continue
            pth = nx.shortest_path(G, a, b, weight="w")
            if max(G[pth[i]][pth[i + 1]]["d"] for i in range(len(pth) - 1)) >= (1 - band) * R:
                s[0] += 1
    ranked = sorted(((s[0] / s[1] + 0.25 * s[3] / s[1], p) for p, s in score.items()
                     if s[2] < s[1] and s[3] <= 0.5 * s[1]), reverse=True)
    chosen = []
    for _, p in ranked:
        if all(not (set(p) & set(q)) for q in chosen):
            chosen.append(p)
        if len(chosen) == k:
            break
    return chosen


# ============================================================ ns-3 workers
_W = {}


def _worker_init():
    import aanet_sim            # loads the ns-3 bindings once per worker process
    import ucraan_solver
    _W["sim"], _W["us"], _W["fleets"] = aanet_sim, ucraan_solver, {}


def _solve_to_files(us, fleets, spec, outdir):
    mode, extra = SOLVER_MODE[spec["protocol"]]
    mob = spec["trace_path"]
    if mob not in fleets:
        fleets[mob] = us.Fleet(mob)
    args = us.build_parser().parse_args(
        ["--mobility", mob, "--src", str(spec["src"]), "--dst", str(spec["dst"]), "--mode", mode,
         "--sensing-error", str(spec["sensing_error"]), "--ho-period", str(spec["ho_period"]),
         "--ho-dur", str(spec["ho_dur"]), "--mesh-range", str(spec["mesh_range"]), "--seed", str(spec["seed"]),
         "--slot", str(spec["slot"]), "--tau", str(spec["tau"]), "--t-end", str(spec["sim_time"] + spec["slot"]),
         "--rho-budget", str(spec["rho_budget"]), "--load-mbps", str(spec["load_mbps"])] + extra + spec["solver_args"])
    P, Rh = us.solve_multi2(args, fleets[mob], [(spec["src"], spec["dst"])])
    rf = os.path.join(outdir, f"routes_{os.getpid()}_{abs(hash(json.dumps(spec, sort_keys=True)))}.csv")
    us.to_rows(P[(spec["src"], spec["dst"])], fleets[mob].n, spec["dst"]).to_csv(rf, index=False)
    rows = [(round(t, 3), i, round(float(r[i]), 4)) for t, r in Rh[(spec["src"], spec["dst"])].items()
            for i in range(fleets[mob].n)]
    pd.DataFrame(rows, columns=["time", "node", "rho"]).to_csv(rf + ".rho", index=False)
    return rf


def _sim_argv(spec, rf):
    a = [f"--mobility={spec['trace_path']}", f"--geoScale={spec['geo_scale']}", f"--simTime={spec['sim_time']}",
         f"--meshRange={spec['mesh_range']}", f"--src={spec['src']}", f"--dst={spec['dst']}",
         f"--appRate={spec['load_mbps']:g}Mbps", f"--handoverDur={spec['ho_dur']}",
         f"--handoverPeriod={spec['ho_period']}", "--output=/dev/null"]
    if rf:
        a += [f"--routes={rf}", f"--rhoFile={rf}.rho"]
    else:
        a += [f"--rhoUniform={spec['rho_budget']}"]
    return a


def _parse(out):
    m = RESULT_RE.search(out)
    if not m:
        return None
    r = {k: float(m.group(i + 1)) for i, k in enumerate(RESULT_KEYS)}
    s = re.search(r"sensing share per aircraft = ([\d.]+)", out)
    r["sensing_share"] = float(s.group(1)) if s else np.nan
    return r


def run_job(spec):
    """one ns-3 run (solver + simulator) inside a persistent worker"""
    tmp = tempfile.mkdtemp(prefix="ucraan_")
    rf = None if spec["protocol"] == "olsr" else _solve_to_files(_W["us"], _W["fleets"], spec, tmp)
    out = _W["sim"].run_once(_sim_argv(spec, rf))
    return _parse(out)


def run_job_fresh(spec):
    """fallback: same job in a brand-new process (used if a worker crashes)"""
    p = subprocess.run([sys.executable, os.path.abspath(__file__), "_job", json.dumps(spec)],
                       capture_output=True, text=True, cwd=os.getcwd())
    try:
        return json.loads(p.stdout.strip().splitlines()[-1])
    except Exception:
        return None


def execute(specs, out_csv, jobs):
    """run specs in parallel persistent workers, checkpoint + resume"""
    done = pd.read_csv(out_csv) if os.path.exists(out_csv) else pd.DataFrame()
    have = set(map(tuple, done[KEY].astype(str).values)) if len(done) else set()
    todo = [s for s in specs if tuple(str(s[k]) for k in KEY) not in have]
    print(f"{len(specs)} runs in plan, {len(specs) - len(todo)} already done, {len(todo)} to run", flush=True)
    rows = done.to_dict("records") if len(done) else []
    if not todo:
        return pd.DataFrame(rows)
    import multiprocessing as mp
    ex = ProcessPoolExecutor(jobs, mp_context=mp.get_context("spawn"), initializer=_worker_init,
                             max_tasks_per_child=40)
    futs = {ex.submit(run_job, s): s for s in todo}
    for i, f in enumerate(as_completed(futs), 1):
        s = futs[f]
        try:
            r = f.result()
        except Exception as e:
            print(f"  worker failed ({type(e).__name__}); rerunning in a fresh process", flush=True)
            r = run_job_fresh(s)
        if r is None:
            print(f"  !! no result for {s['protocol']} {s['trace']} {s['src']}-{s['dst']}", flush=True)
            continue
        row = {k: s[k] for k in KEY}
        row.update(r)
        rows.append(row)
        pd.DataFrame(rows).to_csv(out_csv, index=False)
        print(f"  [{i}/{len(todo)}] {s['protocol']:10s} {s['trace']} {s['src']}-{s['dst']} "
              f"kappa={s['sensing_error']:<4} T_ho={s['ho_period']:<6g} rho={s['rho_budget']:<5g} "
              f"PDR={r['pdr']:.4f} delay={r['delay_ms']:.2f} ms", flush=True)
    ex.shutdown()
    return pd.DataFrame(rows)


def plan(a, kappas, hos, budgets):
    specs = []
    for mob in a.traces:
        for (src, dst) in find_pairs(mob, a.mesh_range, a.sim_time, k=a.pairs):
            for b in budgets:
                for ho in hos:
                    for p in a.protocols:
                        for kp in (kappas if p != "olsr" else kappas[:1]):
                            specs.append(dict(
                                trace=os.path.basename(mob), trace_path=os.path.abspath(mob), src=src, dst=dst,
                                protocol=p, sensing_error=kp, ho_period=ho, rho_budget=b, ho_dur=a.ho_dur,
                                mesh_range=a.mesh_range, seed=a.seed, slot=a.slot, tau=a.tau,
                                sim_time=a.sim_time, load_mbps=a.load, geo_scale=a.geo_scale,
                                solver_args=a.solver_args.split() if a.solver_args else []))
    return specs


def expand_olsr(df, kappas):
    """OLSR does not use sensing, so one run serves every sensing-error level"""
    o = df[df.protocol == "olsr"].drop_duplicates(["trace", "src", "dst", "ho_period", "rho_budget"])
    rest = df[df.protocol != "olsr"]
    reps = [o.assign(sensing_error=k) for k in kappas]
    return pd.concat([rest] + reps, ignore_index=True)


# =================================================================== oracle
_O = {}
DT = 0.2


def _oracle_init(mobs, sim_time):
    import ucraan_solver as us
    _O["us"] = us
    _O["F"], _O["D"] = {}, {}
    for m in mobs:
        f = us.Fleet(m); _O["F"][m] = f
        g = np.arange(0.0, sim_time + 10, DT)
        X = np.stack([[f.pos(i, t) if f.pos(i, t) is not None else np.full(3, 1e12) for i in range(f.n)] for t in g])
        _O["D"][m] = np.linalg.norm(X[:, :, None, :] - X[:, None, :, :], axis=-1)


def _util(p, rho, D, gi, R, load, cap, SAT):
    hops = [(p[i], p[i + 1]) for i in range(len(p) - 1) if SAT not in (p[i], p[i + 1])]
    if not hops or load <= 0:
        return 0.0
    air = {h: load / (cap * max(1.0 - max(rho[h[0]], rho[h[1]]), 1e-3)) for h in hops}
    near = lambda h, k: any(D[gi, x, y] <= R for x in h for y in k)
    return max(sum(air[k] for k in hops if k == h or near(h, k)) for h in hops)


def oracle_evaluate(paths, rho, D, R, ho, slot, ho_dur, t1, load, cap, frame_ms, SAT, t0=2.0):
    """packet-free replay of the installed routes against the true geometry and
    outage schedule (Chapter 3, Sections 3.7-3.8). delivered = validity x min(1, 1/U)"""
    slots = np.array(sorted(paths))
    grid = np.arange(int(round(t0 / DT)), int(round(t1 / DT)))
    ok = deliv = 0.0
    for gi in grid:
        s = gi * DT
        k = np.searchsorted(slots, s + 1e-9, side="right") - 1
        p = paths[slots[k]] if k >= 0 else None
        if p is None:
            continue
        good = True
        for i in range(len(p) - 1):
            a, b = p[i], p[i + 1]
            if a == SAT:
                continue
            if b == SAT:
                if not (s < ho or (s % ho) >= ho_dur):
                    good = False; break
            elif D[gi, a, b] > R:
                good = False; break
        if good:
            ok += 1
            u = _util(p, rho[slots[k]], D, gi, R, load, cap, SAT)
            deliv += min(1.0, 1.0 / u) if u > 0 else 1.0
    commits = phantom = changes = 0; dsum = 0.0; dn = 0; prev = None
    for t in slots:
        if not (t0 - slot <= t < t1) or paths[t] is None:
            prev = None; continue
        p = paths[t]; gi0 = int(round(t / DT)); r_ = rho[t]
        dsum += sum(2.15 if SAT in (p[i], p[i + 1]) else
                    0.5 + D[gi0, p[i], p[i + 1]] / 3e5 + max(r_[p[i]], r_[p[i + 1]]) ** 2 * frame_ms / 2
                    for i in range(len(p) - 1))
        if _util(p, r_, D, gi0, R, load, cap, SAT) >= 1.0:
            dsum += 500.0
        dn += 1
        changes += prev is not None and prev != tuple(p)
        prev = tuple(p)
        for i in range(len(p) - 1):
            if SAT not in (p[i], p[i + 1]):
                commits += 1; phantom += D[gi0, p[i], p[i + 1]] > R
    n = len(grid)
    return dict(path_validity=ok / n, delivered=deliv / n, phantom_rate=phantom / max(commits, 1),
                reroute_per_s=changes / (t1 - t0), path_delay_ms=dsum / max(dn, 1))


def _oracle_job(spec):
    us = _O["us"]; mob = spec["mob"]; f = _O["F"][mob]; R = spec["R"]
    pairs = [(a, b) for a, b in itertools.combinations(range(f.n), 2)
             if not all(f.pos(a, t) is not None and f.pos(b, t) is not None and
                        np.linalg.norm(f.pos(a, t) - f.pos(b, t)) <= R for t in np.arange(0, spec["T"], 5))]
    mode, extra = SOLVER_MODE[spec["protocol"]]
    args = us.build_parser().parse_args(
        ["--mobility", mob, "--mode", mode, "--sensing-error", str(spec["kappa"]), "--ho-period", str(spec["ho"]),
         "--seed", str(spec["seed"]), "--mesh-range", str(R), "--t-end", str(spec["T"] + 1),
         "--tau", str(spec["tau"]), "--rho-budget", str(spec["budget"]), "--load-mbps", str(spec["load"])]
        + extra + spec["solver_args"])
    P, Rh = us.solve_multi2(args, f, pairs)
    rows = []
    for (a, b), paths in P.items():
        m = oracle_evaluate(paths, Rh[(a, b)], _O["D"][mob], R, spec["ho"], args.slot, args.ho_dur, spec["T"] - 1,
                            args.load_mbps, args.cap_mbps, args.isac_frame * 1e3, us.SAT)
        rows.append(dict(trace=os.path.basename(mob), src=a, dst=b, protocol=spec["protocol"],
                         sensing_error=spec["kappa"], ho_period=spec["ho"], rho_budget=spec["budget"],
                         noise_seed=spec["seed"], **m))
    return rows


def cmd_oracle(a):
    mobs = [os.path.abspath(m) for m in a.traces]
    specs = [dict(mob=m, protocol=p, kappa=k, ho=h, budget=b, seed=s, R=a.mesh_range, T=a.sim_time,
                  tau=a.tau, load=a.load, solver_args=a.solver_args.split() if a.solver_args else [])
             for m in mobs for p in a.oracle_protocols for k in a.sensing for h in a.ho_period
             for b in a.budgets for s in range(1, a.noise_seeds + 1)]
    print(f"oracle: {len(specs)} solver jobs over all decision-relevant pairs", flush=True)
    rows = []
    with ProcessPoolExecutor(a.jobs, initializer=_oracle_init, initargs=(mobs, a.sim_time)) as ex:
        for r in ex.map(_oracle_job, specs, chunksize=1):
            rows.extend(r)
    df = pd.DataFrame(rows)
    df.to_csv(f"{a.results}/oracle.csv", index=False)
    print(df.groupby("protocol")[["delivered", "path_validity", "phantom_rate", "path_delay_ms",
                                  "reroute_per_s"]].mean().round(4).to_string())


# ============================================================= calibration
def cmd_calibration(a):
    import ucraan_solver as us
    rec = []
    for mob in a.traces:
        f = us.Fleet(mob)
        for k in a.sensing:
            args = us.build_parser().parse_args(["--mobility", mob, "--sensing-error", str(k), "--t-end",
                                                 str(a.sim_time + 1), "--p-min", "0.0", "--src", "0", "--dst", "1",
                                                 "--mesh-range", str(a.mesh_range), "--tau", str(a.tau)])
            log = []
            us.solve_multi2(args, f, [(0, 1)], log_calib=log)
            rec += [(os.path.basename(mob), k, p, alive) for (p, alive, *_r) in log]
    df = pd.DataFrame(rec, columns=["trace", "sensing_error", "p", "alive"])
    bins = np.clip((df.p.values * 10).astype(int), 0, 9)
    tab = []
    ece = 0.0
    for b in range(10):
        m = bins == b
        if m.any():
            conf, acc = df.p.values[m].mean(), df.alive.values[m].mean()
            ece += m.mean() * abs(acc - conf)
            tab.append(dict(bin=f"{b/10:.1f}-{(b+1)/10:.1f}", n=int(m.sum()), predicted=conf, observed=acc))
    t = pd.DataFrame(tab)
    os.makedirs(f"{a.results}/tables", exist_ok=True)
    t.to_csv(f"{a.results}/tables/calibration.csv", index=False)
    print(t.round(3).to_string(index=False)); print(f"ECE = {ece:.4f}  over {len(df):,} link-slot predictions")
    json.dump(dict(ece=ece, n=len(df)), open(f"{a.results}/calibration.json", "w"))


# ================================================================ analysis
def wilcoxon_vs(df, ref, other):
    from scipy.stats import wilcoxon
    k = ["trace", "src", "dst", "sensing_error", "ho_period", "rho_budget"]
    U = df[df.protocol == ref].set_index(k).pdr
    B = df[df.protocol == other].set_index(k).pdr
    j = U.index.intersection(B.index)
    d = (U.loc[j] - B.loc[j]).values
    nz = d[np.abs(d) > 1e-4]
    p = wilcoxon(nz).pvalue if len(nz) > 5 else float("nan")
    return dict(n=len(j), win=int((d > 1e-4).sum()), tie=int((np.abs(d) <= 1e-4).sum()),
                loss=int((d < -1e-4).sum()), p=p, mean_gain=float(d.mean()) if len(d) else np.nan)


def fmt_p(p):
    return "n/a" if not np.isfinite(p) else ("< 0.001" if p < 1e-3 else f"{p:.3f}")


def md_table(df):
    cols = list(df.columns)
    out = ["| " + " | ".join(map(str, cols)) + " |", "|" + "---|" * len(cols)]
    out += ["| " + " | ".join(map(str, r)) + " |" for r in df.values]
    return "\n".join(out)


def _style():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    matplotlib.rcParams.update({"font.family": "serif", "mathtext.fontset": "stix", "font.size": 10.5})
    return plt


COL = {"ucraan_jsc": "#2a78d6", "ucraan": "#eda100", "delay_opt": "#eb6834", "rl_value": "#1baf7a", "olsr": "#e87ba4"}
MK = {"ucraan_jsc": "o", "ucraan": "D", "delay_opt": "s", "rl_value": "^", "olsr": "v"}
LS = {"ucraan_jsc": "-", "ucraan": ":", "delay_opt": "--", "rl_value": "-.", "olsr": (0, (5, 1, 1, 1, 1, 1))}


def cmd_analyse(a):
    R = a.results
    os.makedirs(f"{R}/tables", exist_ok=True); os.makedirs(f"{R}/figures", exist_ok=True)
    report = []
    order = [p for p in ["ucraan_jsc", "ucraan", "delay_opt", "rl_value", "olsr"]]
    if os.path.exists(f"{R}/headline.csv"):
        H = pd.read_csv(f"{R}/headline.csv")
        H = expand_olsr(H, sorted(H[H.protocol != "olsr"].sensing_error.unique()))
        g = H.groupby("protocol")
        hard = H[(H.ho_period == H.ho_period.min()) & (H.sensing_error == H.sensing_error.max())].groupby("protocol").pdr.mean()
        rows = []
        for p in [q for q in order if q in g.groups]:
            t = wilcoxon_vs(H, "ucraan_jsc", p) if p != "ucraan_jsc" else None
            x = g.get_group(p)
            rows.append([LABEL[p], f"{x.pdr.mean():.3f}", f"{x.pdr.quantile(0.05):.3f}", f"{hard.get(p, np.nan):.3f}",
                         f"{x.delay_ms.mean():.2f} / {x.delay_ms.median():.2f}", f"{x.route_lifetime_s.mean():.1f}",
                         f"{x.reroute_per_s.mean():.3f}", f"{x.ctrl_pkts_per_s.mean():.2f}",
                         "—" if t is None else f"{t['win']}/{t['tie']}/{t['loss']}; p {fmt_p(t['p'])}"])
        T = pd.DataFrame(rows, columns=["Protocol", "PDR", "PDR 5th pct.",
                                        f"PDR (T_ho {H.ho_period.min():g} s, kappa {H.sensing_error.max():g})",
                                        "Delay mean / median (ms)", "Route life (s)", "Re-routes/s", "Control/s",
                                        "UCRAAN-JSC vs protocol (win/tie/loss; Wilcoxon)"])
        T.to_csv(f"{R}/tables/table_headline.csv", index=False)
        report += [f"## Headline campaign (rho = {H.rho_budget.iloc[0]:g}, {int((H.protocol == 'ucraan_jsc').sum())} cells)",
                   md_table(T), "",
                   "PDR by sensing error:", md_table(H.pivot_table(index="sensing_error", columns="protocol", values="pdr").round(4).reset_index()), "",
                   "PDR by handover period:", md_table(H.pivot_table(index="ho_period", columns="protocol", values="pdr").round(4).reset_index()), ""]
        plt = _style()
        kap = sorted(H.sensing_error.unique()); hos = sorted(H.ho_period.unique())
        fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.1), dpi=150, sharey=True)
        lo = max(0.0, np.floor(H.groupby(["protocol", "ho_period", "sensing_error"]).pdr.mean().min() * 20) / 20 - 0.02)
        for ax, ho in [(axes[0], max(hos)), (axes[1], min(hos))]:
            sub = H[H.ho_period == ho]
            for p in [q for q in order[::-1] if q in g.groups]:
                m = sub[sub.protocol == p].groupby("sensing_error").pdr.mean().reindex(kap)
                ax.plot(range(len(kap)), m.values, color=COL[p], marker=MK[p], ms=6, lw=2, ls=LS[p], label=LABEL[p],
                        markeredgecolor="white", markeredgewidth=0.8, zorder=3 if p == "ucraan_jsc" else 2)
            ax.set_xticks(range(len(kap))); ax.set_xticklabels([f"{k:g}" for k in kap])
            ax.set_xlabel(r"sensing-error factor $\kappa$"); ax.set_title(rf"$T_{{ho}}$ = {ho:g} s", fontsize=10.5)
            ax.grid(axis="y", color="#e6e6e6", lw=0.8); ax.set_axisbelow(True)
            for s in ("top", "right"):
                ax.spines[s].set_visible(False)
        axes[0].set_ylabel("packet delivery ratio (ns-3)"); axes[0].set_ylim(lo, 1.01)
        h, l = axes[1].get_legend_handles_labels()
        fig.legend(h[::-1], l[::-1], loc="lower center", ncol=5, frameon=False, fontsize=9)
        fig.tight_layout(rect=(0, 0.07, 1, 1)); fig.savefig(f"{R}/figures/fig_pdr_vs_kappa.png", facecolor="white")
    if os.path.exists(f"{R}/budget.csv"):
        B = pd.read_csv(f"{R}/budget.csv")
        B = expand_olsr(B, sorted(B[B.protocol != "olsr"].sensing_error.unique()))
        pb = B.pivot_table(index="rho_budget", columns="protocol", values="pdr").sort_index()
        mb = B.groupby(["rho_budget", "protocol"]).delay_ms.median().unstack().sort_index()
        T = pd.DataFrame([[LABEL[p]] + [f"{pb[p][b]:.3f} / {mb[p][b]:.1f}" for b in pb.index]
                          for p in order if p in pb], columns=["Protocol"] + [f"rho={b:g}" for b in pb.index])
        T.to_csv(f"{R}/tables/table_budget.csv", index=False)
        report += ["## Sensing-budget campaign (PDR / median delay ms)", md_table(T), ""]
        plt = _style()
        fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.1), dpi=150)
        xs = list(pb.index)
        for p in [q for q in order[::-1] if q in pb]:
            for ax, v in [(axes[0], pb[p]), (axes[1], mb[p])]:
                ax.plot(xs, v.values, color=COL[p], marker=MK[p], ms=6, lw=2, ls=LS[p], label=LABEL[p],
                        markeredgecolor="white", markeredgewidth=0.8, zorder=3 if p == "ucraan_jsc" else 2)
        for ax in axes:
            ax.set_xscale("log"); ax.set_xticks(xs); ax.set_xticklabels([f"{x:g}" for x in xs]); ax.minorticks_off()
            ax.set_xlabel(r"network sensing budget $\bar{\rho}$"); ax.grid(axis="y", color="#e6e6e6", lw=0.8)
            for s in ("top", "right"):
                ax.spines[s].set_visible(False)
        axes[0].set_ylabel("packet delivery ratio (ns-3)"); axes[1].set_ylabel("median delay (ms, ns-3)")
        axes[1].set_yscale("log")
        h, l = axes[0].get_legend_handles_labels()
        fig.legend(h[::-1], l[::-1], loc="lower center", ncol=5, frameon=False, fontsize=9)
        fig.tight_layout(rect=(0, 0.07, 1, 1)); fig.savefig(f"{R}/figures/fig_sensing_tradeoff.png", facecolor="white")
    if os.path.exists(f"{R}/oracle.csv"):
        O = pd.read_csv(f"{R}/oracle.csv")
        T = O.groupby("protocol")[["delivered", "path_validity", "phantom_rate", "path_delay_ms", "reroute_per_s"]].mean().round(4)
        T.to_csv(f"{R}/tables/table_oracle.csv")
        cells = O.pivot_table(index=["rho_budget", "ho_period", "sensing_error"], columns="protocol", values="delivered")
        eq = int(((cells.get("ucraan_jsc") - cells.get("genie")).abs() < 5e-4).sum()) if {"ucraan_jsc", "genie"} <= set(cells) else None
        report += [f"## Oracle ({O.groupby(['trace','src','dst']).ngroups} decision-relevant pairs)",
                   md_table(T.reset_index()), "" if eq is None else f"UCRAAN-JSC equals the genie in {eq} of {len(cells)} cells", ""]
    if os.path.exists(f"{R}/calibration.json"):
        c = json.load(open(f"{R}/calibration.json"))
        report += [f"## Calibration: ECE = {c['ece']:.4f} over {c['n']:,} predictions", ""]
    open(f"{R}/tables/REPORT.md", "w").write("\n".join(report))
    print("\n".join(report))
    print(f"\ntables -> {R}/tables   figures -> {R}/figures")


# ===================================================================== main
def main():
    if len(sys.argv) > 2 and sys.argv[1] == "_job":           # internal: fresh-process fallback
        _worker_init(); print(json.dumps(run_job(json.loads(sys.argv[2])))); return
    ap = argparse.ArgumentParser(description="UCRAAN experiments (see header of this file)",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["pairs", "headline", "budget", "oracle", "calibration", "analyse", "all"])
    ap.add_argument("--traces", nargs="+", default=None, help="default: data/cluster_mobility_*.csv")
    ap.add_argument("--results", default="results")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2)))
    ap.add_argument("--pairs", type=int, default=2, help="src-dst pairs per trace (ns-3)")
    ap.add_argument("--protocols", nargs="+", default=NS3_PROTOCOLS)
    ap.add_argument("--oracle-protocols", nargs="+", default=["delay_opt", "rl_value", "ucraan", "ucraan_jsc", "genie"])
    ap.add_argument("--sensing", type=float, nargs="+", default=None, help="kappa values")
    ap.add_argument("--ho-period", type=float, nargs="+", default=None, help="satellite handover periods (s)")
    ap.add_argument("--budgets", type=float, nargs="+", default=None, help="sensing budgets rho-bar")
    ap.add_argument("--mesh-range", type=float, default=150000.0)
    ap.add_argument("--load", type=float, default=6.0, help="traffic (Mbps)")
    ap.add_argument("--sim-time", type=float, default=120.0)
    ap.add_argument("--slot", type=float, default=1.0)
    ap.add_argument("--tau", type=float, default=2.0, help="commitment horizon used in the reported ns-3 runs")
    ap.add_argument("--ho-dur", type=float, default=5.0)
    ap.add_argument("--geo-scale", type=float, default=0.0005)
    ap.add_argument("--seed", type=int, default=1, help="sensing-noise seed (ns-3)")
    ap.add_argument("--noise-seeds", type=int, default=3, help="noise seeds (oracle)")
    ap.add_argument("--solver-args", default="", help="extra solver flags, e.g. for ablations")
    a = ap.parse_args()
    a.traces = a.traces or sorted(glob.glob("data/cluster_mobility_*.csv"),
                                  key=lambda s: int(re.findall(r"\d+", os.path.basename(s))[-1]))
    if not a.traces and a.command != "analyse":
        raise SystemExit("no traces: run prepare_data.py first (or pass --traces)")
    os.makedirs(a.results, exist_ok=True)

    if a.command == "pairs":
        for m in a.traces:
            print(os.path.basename(m), find_pairs(m, a.mesh_range, a.sim_time, k=a.pairs))
    if a.command in ("headline", "all"):
        k = a.sensing or [0.5, 2.0, 4.0]; h = a.ho_period or [1000.0, 30.0, 15.0]; b = a.budgets or [0.1]
        execute(plan(a, k, h, b), f"{a.results}/headline.csv", a.jobs)
    if a.command in ("budget", "all"):
        k = a.sensing or [2.0]; h = a.ho_period or [15.0]; b = a.budgets or [0.02, 0.05, 0.1, 0.2, 0.3]
        execute(plan(a, k, h, b), f"{a.results}/budget.csv", a.jobs)
    if a.command in ("oracle", "all"):
        a.sensing = a.sensing or [0.2, 0.5, 1.0, 2.0, 4.0]; a.ho_period = a.ho_period or [1000.0, 60.0, 30.0, 15.0]
        a.budgets = a.budgets or [0.1]
        cmd_oracle(a)
    if a.command in ("calibration", "all"):
        a.sensing = a.sensing or [0.2, 0.5, 1.0, 2.0, 4.0]
        cmd_calibration(a)
    if a.command in ("analyse", "all"):
        cmd_analyse(a)


if __name__ == "__main__":
    main()
