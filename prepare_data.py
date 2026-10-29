#!/usr/bin/env python3
# ============================================================================
#  prepare_data.py  (file 1 of 4)  --  step 1:  flight data  ->  simulator-ready traces
#
#  Implements the six-stage preprocessing pipeline of Chapter 3 (Section 3.5):
#    1 column normalisation   (OpenSky / ADS-B Exchange / generic CSV headers)
#    2 airborne filter        (altitude > 1,000 m, not on ground)
#    3 busiest-window search  (window relative to the dataset start)
#    4 spatial clustering     (seed = aircraft with most neighbours within
#                              --radius; keep the --n nearest to the seed)
#    5 projection             WGS-84 -> ECEF -> local east-north-up frame
#                              centred on the cluster (distances preserved)
#    6 resampling             1-s grid, time rebased to 0
#  Output: data/cluster_mobility_<k>.csv  with columns
#          icao24,t,x,y,z,velocity,heading      (metres, seconds, m/s, degrees)
#
#  REAL DATA (OpenSky state-vector dumps, e.g. states_2022-06-27-12.csv[.tar]):
#     python3 prepare_data.py --input states_2022-06-27-12.csv.tar \
#            --bbox 40 62 -60 -5 --n-traces 6
#  Several input files may be given (they are concatenated).
#  Accepted: .csv  .csv.gz  .csv.tar  .parquet
#
#  SYNTHETIC DATA (NAT Organised-Track-style traffic, for testing the pipeline):
#     python3 prepare_data.py --synthetic --n-traces 6
#
#  Requires: numpy, pandas
# ============================================================================
import argparse
import io
import math
import os
import tarfile

import numpy as np
import pandas as pd

# WGS-84
A_WGS = 6378137.0
E2 = 6.69437999014e-3

COLS = {  # canonical name -> accepted header names (case-insensitive)
    "icao24": ["icao24", "icao", "hex", "icao_address", "aircraft_id"],
    "time": ["time", "timestamp", "ts", "epoch", "lastposupdate", "datetime"],
    "lat": ["lat", "latitude"],
    "lon": ["lon", "lng", "long", "longitude"],
    "alt": ["geoaltitude", "baroaltitude", "altitude", "alt", "alt_geom", "alt_baro", "height"],
    "velocity": ["velocity", "groundspeed", "ground_speed", "gs", "speed"],
    "heading": ["heading", "true_track", "track", "course"],
    "onground": ["onground", "on_ground", "ground"],
}


# ---------------------------------------------------------------- 1 reading
def _read_one(path, bbox, chunksize=2_000_000):
    """read one file in chunks, keep only the needed columns and the bbox"""
    def frames(reader_src, compression=None):
        head = pd.read_csv(reader_src, nrows=0, compression=compression)
        low = {c.lower().strip(): c for c in head.columns}
        pick = {}
        for canon, names in COLS.items():
            for nm in names:
                if nm in low:
                    pick[canon] = low[nm]
                    break
        missing = [c for c in ("icao24", "time", "lat", "lon", "alt") if c not in pick]
        if missing:
            raise SystemExit(f"{path}: cannot find columns {missing}; header = {list(head.columns)}")
        return pick

    if path.endswith(".parquet"):
        df = pd.read_parquet(path)
        yield _normalise(df, bbox)
        return
    if path.endswith(".tar"):
        with tarfile.open(path) as tf:
            for m in tf.getmembers():
                if m.name.endswith((".csv", ".csv.gz")):
                    raw = tf.extractfile(m).read()
                    comp = "gzip" if m.name.endswith(".gz") else None
                    pick = frames(io.BytesIO(raw), comp)
                    for ch in pd.read_csv(io.BytesIO(raw), compression=comp, usecols=list(pick.values()),
                                          chunksize=chunksize, low_memory=False):
                        yield _normalise(ch.rename(columns={v: k for k, v in pick.items()}), bbox, renamed=True)
        return
    pick = frames(path)
    for ch in pd.read_csv(path, usecols=list(pick.values()), chunksize=chunksize, low_memory=False):
        yield _normalise(ch.rename(columns={v: k for k, v in pick.items()}), bbox, renamed=True)


def _normalise(df, bbox, renamed=False):
    if not renamed:
        low = {c.lower().strip(): c for c in df.columns}
        ren = {}
        for canon, names in COLS.items():
            for nm in names:
                if nm in low:
                    ren[low[nm]] = canon
                    break
        df = df.rename(columns=ren)
    for c in ("velocity", "heading", "onground"):
        if c not in df:
            df[c] = np.nan
    df = df[["icao24", "time", "lat", "lon", "alt", "velocity", "heading", "onground"]].copy()
    t = df["time"]
    if not np.issubdtype(t.dtype, np.number):
        df["time"] = pd.to_datetime(t, utc=True, errors="coerce").astype("int64") / 1e9
    for c in ("time", "lat", "lon", "alt", "velocity", "heading"):
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=["icao24", "time", "lat", "lon", "alt"])
    if bbox:
        la0, la1, lo0, lo1 = bbox
        df = df[(df.lat >= la0) & (df.lat <= la1) & (df.lon >= lo0) & (df.lon <= lo1)]
    df["icao24"] = df["icao24"].astype(str).str.strip().str.lower()
    return df


def load(paths, bbox, alt_unit, speed_unit):
    df = pd.concat([f for p in paths for f in _read_one(p, bbox)], ignore_index=True)
    if df.empty:
        raise SystemExit("no records left after reading (check --bbox and the input files)")
    # units: OpenSky is metres and m/s; ADS-B Exchange style is feet and knots
    if alt_unit == "auto":
        alt_unit = "ft" if df["alt"].quantile(0.9) > 15000 else "m"
    if alt_unit == "ft":
        df["alt"] *= 0.3048
    if speed_unit == "auto":
        speed_unit = "kt" if df["velocity"].quantile(0.9) > 330 else "ms"
    if speed_unit == "kt":
        df["velocity"] *= 0.514444
    print(f"read {len(df):,} records, {df.icao24.nunique():,} aircraft (altitude in {alt_unit}, speed in {speed_unit})")
    return df


# ------------------------------------------------------------ 2..4 selection
def airborne(df, min_alt):
    og = df["onground"].astype(str).str.lower().isin(["true", "1", "t", "yes"])
    df = df[(df["alt"] > min_alt) & ~og].drop_duplicates(["icao24", "time"])
    print(f"airborne filter: {len(df):,} records, {df.icao24.nunique():,} aircraft")
    return df


def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = np.radians(lat1), np.radians(lat2)
    dp, dl = p2 - p1, np.radians(lon2 - lon1)
    a = np.sin(dp / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dl / 2) ** 2
    return 2 * 6371000.0 * np.arcsin(np.sqrt(a))


def segments(df, window, duration, max_gap, step=30.0):
    """rank D-second segments (inside the busiest windows) by the number of
    aircraft that are continuously tracked through them"""
    t0 = df.time.min()
    rel = df.time - t0
    # busiest windows relative to the dataset start (stage 3)
    wb = (rel // window).astype(int)
    busy = df.groupby(wb).icao24.nunique().sort_values(ascending=False)
    out = []
    for w in busy.index:
        sub = df[wb == w]
        tr = {a: np.sort(g.time.values) for a, g in sub.groupby("icao24")}
        ws, we = t0 + w * window, t0 + (w + 1) * window
        for s in np.arange(ws, we - duration + 1e-9, step):
            e = s + duration
            ok = []
            for a, ts in tr.items():
                if ts[0] > s or ts[-1] < e:        # observed through the whole segment: no extrapolation
                    continue
                inside = ts[(ts >= s - max_gap) & (ts <= e + max_gap)]
                if len(inside) >= 2 and np.max(np.diff(inside)) <= max_gap:
                    ok.append(a)
            out.append((len(ok), s, e, ok))
    out.sort(key=lambda x: -x[0])
    return out


def cluster(df, s, e, ok, n, radius):
    """stage 4: seed = aircraft with most neighbours within radius; n nearest"""
    sub = df[(df.time >= s) & (df.time <= e) & df.icao24.isin(ok)]
    med = sub.groupby("icao24")[["lat", "lon"]].median()
    ids = list(med.index)
    if len(ids) < 2:
        return None
    la, lo = med.lat.values, med.lon.values
    D = haversine(la[:, None], lo[:, None], la[None, :], lo[None, :])
    seed = int(np.argmax((D <= radius).sum(axis=1)))
    order = np.argsort(D[seed])[:n]
    return [ids[i] for i in order]


# ------------------------------------------------------------- 5 projection
def geodetic_to_ecef(lat, lon, h):
    la, lo = np.radians(lat), np.radians(lon)
    N = A_WGS / np.sqrt(1 - E2 * np.sin(la) ** 2)
    return ((N + h) * np.cos(la) * np.cos(lo), (N + h) * np.cos(la) * np.sin(lo), (N * (1 - E2) + h) * np.sin(la))


def ecef_to_enu(x, y, z, lat0, lon0):
    x0, y0, z0 = geodetic_to_ecef(lat0, lon0, 0.0)
    dx, dy, dz = x - x0, y - y0, z - z0
    la, lo = math.radians(lat0), math.radians(lon0)
    e = -math.sin(lo) * dx + math.cos(lo) * dy
    nn = -math.sin(la) * math.cos(lo) * dx - math.sin(la) * math.sin(lo) * dy + math.cos(la) * dz
    u = math.cos(la) * math.cos(lo) * dx + math.cos(la) * math.sin(lo) * dy + math.sin(la) * dz
    return e, nn, u


# ------------------------------------------------------------- 6 resampling
def build_trace(df, s, e, ids, dt=1.0):
    sub = df[(df.time >= s - 60) & (df.time <= e + 60) & df.icao24.isin(ids)]
    lat0, lon0 = sub.lat.median(), sub.lon.median()
    grid = np.arange(s, e + 1e-9, dt)
    rows = []
    for a in ids:
        g = sub[sub.icao24 == a].sort_values("time")
        ts = g.time.values
        lat = np.interp(grid, ts, g.lat.values)
        lon = np.interp(grid, ts, g.lon.values)
        alt = np.interp(grid, ts, g.alt.values)
        X, Y, Z = geodetic_to_ecef(lat, lon, alt)
        E, Nn, U = ecef_to_enu(X, Y, Z, lat0, lon0)
        v = g.velocity.values
        vel = np.interp(grid, ts, v) if np.isfinite(v).sum() >= 2 else np.full(len(grid), np.nan)
        h = g.heading.values
        if np.isfinite(h).sum() >= 2:
            hh = np.degrees(np.unwrap(np.radians(np.nan_to_num(h, nan=np.nanmedian(h)))))
            hdg = np.mod(np.interp(grid, ts, hh), 360)
        else:
            hdg = np.full(len(grid), np.nan)
        for k in range(len(grid)):
            rows.append((a, round(grid[k] - s, 3), E[k], Nn[k], U[k], vel[k], hdg[k]))
    return pd.DataFrame(rows, columns=["icao24", "t", "x", "y", "z", "velocity", "heading"])


def describe(tr, mesh_range):
    ids = list(dict.fromkeys(tr.icao24))
    P = tr[tr.t == 0].set_index("icao24").loc[ids][["x", "y", "z"]].values
    D = np.linalg.norm(P[:, None] - P[None], axis=-1)[np.triu_indices(len(ids), 1)]
    return (f"{len(ids)} aircraft; pair distance at t=0: min {D.min()/1e3:.0f} km, median "
            f"{np.median(D)/1e3:.0f} km; {int((D <= mesh_range).sum())}/{len(D)} pairs within "
            f"{mesh_range/1e3:.0f} km")


# ------------------------------------------------------------- synthetic
def synthetic(seed, n_air=8, duration=300.0, dt=1.0):
    """NAT Organised-Track-style traffic (Section 3.5): 3 parallel eastbound
    tracks, one opposite-direction and one crossing flight, OU heading/speed wander"""
    rng = np.random.default_rng(seed)
    specs = []
    track_y = np.array([-1, 0, 1]) * rng.uniform(95e3, 125e3)
    for k in range(n_air - 2):
        y = track_y[k % 3] + rng.normal(0, 6e3)
        x = (k // 3) * rng.uniform(140e3, 190e3) + rng.uniform(-60e3, 60e3) - 80e3
        specs.append((x, y, rng.uniform(232, 258), 90 + rng.normal(0, 2.5)))
    specs.append((rng.uniform(150e3, 260e3), rng.uniform(-60e3, 60e3), rng.uniform(235, 255), 270 + rng.normal(0, 4)))
    specs.append((rng.uniform(-120e3, 120e3), rng.uniform(-260e3, -180e3), rng.uniform(235, 255),
                  rng.choice([20, 35, 45]) + rng.normal(0, 3)))
    ts = np.arange(0.0, duration + dt, dt)
    rows = []
    for (x0, y0, spd0, hdg0) in specs:
        icao = f"{rng.integers(0x400000, 0x4FFFFF):06x}"
        alt = rng.choice([10363, 10668, 10973, 11278, 11582])
        x, y, dh, ds = x0, y0, 0.0, 0.0
        for t in ts:
            dh += -0.05 * dh * dt + rng.normal(0, 0.08) * math.sqrt(dt)
            ds += -0.05 * ds * dt + rng.normal(0, 0.15) * math.sqrt(dt)
            h = math.radians(hdg0 + dh); v = spd0 + ds
            rows.append((icao, t, x, y, float(alt) + rng.normal(0, 3), v, (hdg0 + dh) % 360))
            x += v * math.sin(h) * dt; y += v * math.cos(h) * dt
    df = pd.DataFrame(rows, columns=["icao24", "t", "x", "y", "z", "velocity", "heading"])
    c = df[df.t == 0][["x", "y"]].mean()
    df["x"] -= c["x"]; df["y"] -= c["y"]
    return df


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", nargs="*", default=[], help="OpenSky/ADS-B files (.csv .csv.gz .csv.tar .parquet)")
    ap.add_argument("--synthetic", action="store_true", help="generate synthetic NAT traces instead")
    ap.add_argument("--outdir", default="data")
    ap.add_argument("--n-traces", type=int, default=6, help="independent traces to extract")
    ap.add_argument("--n", type=int, default=8, help="aircraft per trace")
    ap.add_argument("--duration", type=float, default=300.0, help="trace length (s); sim uses the first 120 s")
    ap.add_argument("--window", type=float, default=1800.0, help="busiest-window length (s)")
    ap.add_argument("--radius", type=float, default=400e3, help="clustering radius (m)")
    ap.add_argument("--min-alt", type=float, default=1000.0, help="airborne threshold (m)")
    ap.add_argument("--max-gap", type=float, default=30.0, help="max gap between reports inside a trace (s)")
    ap.add_argument("--bbox", type=float, nargs=4, metavar=("LAT_MIN", "LAT_MAX", "LON_MIN", "LON_MAX"),
                    help="e.g. North Atlantic: 40 62 -60 -5")
    ap.add_argument("--alt-unit", choices=["auto", "m", "ft"], default="auto")
    ap.add_argument("--speed-unit", choices=["auto", "ms", "kt"], default="auto")
    ap.add_argument("--mesh-range", type=float, default=150e3, help="only used for the printed summary")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    if a.synthetic:
        for k in range(1, a.n_traces + 1):
            tr = synthetic(k, a.n, a.duration)
            f = f"{a.outdir}/cluster_mobility_{k}.csv"; tr.to_csv(f, index=False)
            print(f"{f}: {describe(tr, a.mesh_range)}")
        return
    if not a.input:
        raise SystemExit("give --input <files> or --synthetic")

    df = airborne(load(a.input, a.bbox, a.alt_unit, a.speed_unit), a.min_alt)
    segs = segments(df, a.window, a.duration, a.max_gap)
    used, k = [], 0
    for cnt, s, e, ok in segs:
        if k >= a.n_traces or cnt < a.n:
            break
        if any(not (e <= us or s >= ue) for us, ue in used):   # non-overlapping in time
            continue
        ids = cluster(df, s, e, ok, a.n, a.radius)
        if not ids or len(ids) < a.n:
            continue
        k += 1
        used.append((s, e))
        tr = build_trace(df, s, e, ids)
        f = f"{a.outdir}/cluster_mobility_{k}.csv"; tr.to_csv(f, index=False)
        print(f"{f}: start {pd.to_datetime(s, unit='s', utc=True)}  {describe(tr, a.mesh_range)}")
    if k < a.n_traces:
        print(f"only {k} traces with {a.n} continuously tracked aircraft were found "
              f"(try a larger --bbox, more input hours, a larger --max-gap or a smaller --n)")


if __name__ == "__main__":
    main()
