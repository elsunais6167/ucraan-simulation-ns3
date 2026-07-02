#!/usr/bin/env python3
# ============================================================================
#  build_mobility_cluster.py
#
#  Picks a CLUSTER of aircraft that are actually near each other and airborne
#  at the same time -- a real AANET -- instead of "the busiest 8 planes in the
#  whole dataset" (which are scattered thousands of km apart and never connect).
#
#  Output: mobility.csv with columns icao24,t,x,y,z,velocity,heading
#  (x,y,z are local metres, ready for aanet-sim.cc and ucraan_solver.py).
#
#  USAGE:
#    python3 build_mobility_cluster.py --raw opensky_aviation_dataset.csv \
#            --n 8 --window 1800 --radius 400
# ============================================================================

import argparse
import numpy as np
import pandas as pd
from pyproj import Transformer


def pick_col(df, names, required=True, default=None):
    for n in names:
        if n in df.columns:
            return n
    if required:
        raise SystemExit(f"None of these columns found: {names}\nHave: {list(df.columns)}")
    return default


def haversine_km(lat1, lon1, lat2, lon2):
    R = 6371.0
    p1, p2 = np.radians(lat1), np.radians(lat2)
    dlat = np.radians(lat2 - lat1)
    dlon = np.radians(lon2 - lon1)
    a = np.sin(dlat / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dlon / 2) ** 2
    return 2 * R * np.arcsin(np.sqrt(a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", required=True, help="raw OpenSky-style CSV")
    ap.add_argument("--out", default="mobility.csv")
    ap.add_argument("--n", type=int, default=8, help="aircraft to keep")
    ap.add_argument("--window", type=float, default=1800, help="time window (s)")
    ap.add_argument("--radius", type=float, default=400, help="cluster radius (km)")
    args = ap.parse_args()

    df = pd.read_csv(args.raw)
    c_id = pick_col(df, ["icao24", "callsign", "id"])
    c_t = pick_col(df, ["time", "timestamp", "lastposupdate", "t"])
    c_lat = pick_col(df, ["lat", "latitude"])
    c_lon = pick_col(df, ["lon", "longitude"])
    c_alt = pick_col(df, ["baroaltitude", "geoaltitude", "altitude", "alt"],
                     required=False)
    c_vel = pick_col(df, ["velocity", "groundspeed", "speed"], required=False)
    c_hdg = pick_col(df, ["heading", "track"], required=False)

    df = df.rename(columns={c_id: "icao24", c_t: "time", c_lat: "lat", c_lon: "lon"})
    df["z"] = df[c_alt] if c_alt else 10000.0
    df["velocity"] = df[c_vel] if c_vel else 250.0
    df["heading"] = df[c_hdg] if c_hdg else 0.0
    df = df.dropna(subset=["icao24", "time", "lat", "lon"])
    df = df[df["z"].fillna(10000) > 1000]  # airborne only
    df["time"] = pd.to_numeric(df["time"], errors="coerce")
    df = df.dropna(subset=["time"])

    # 1) busiest time window: bucket time RELATIVE TO THE DATASET START (an
    #    epoch-aligned bucket can straddle the data boundary and yield a
    #    partial window), take the bucket with most aircraft
    t0 = df["time"].min()
    df["bucket"] = ((df["time"] - t0) // args.window) * args.window + t0
    best = df.groupby("bucket")["icao24"].nunique().idxmax()
    win = df[df["bucket"] == best].copy()
    print(f"window @ t={best:.0f}  aircraft in window: {win['icao24'].nunique()}")

    # 2) each aircraft's median position in the window
    med = win.groupby("icao24")[["lat", "lon"]].median()

    # 3) seed = aircraft with the most neighbours within radius
    best_count, seed = -1, None
    for a, row in med.iterrows():
        d = haversine_km(row["lat"], row["lon"], med["lat"].values, med["lon"].values)
        cnt = int((d <= args.radius).sum())
        if cnt > best_count:
            best_count, seed = cnt, a
    if best_count < 2:
        raise SystemExit("No cluster found; try a bigger --radius or --window.")

    # 4) keep the n aircraft closest to the seed
    d_seed = haversine_km(med.loc[seed, "lat"], med.loc[seed, "lon"],
                          med["lat"].values, med["lon"].values)
    med = med.assign(dist=d_seed).sort_values("dist")
    keep = list(med.index[:args.n])
    spread = haversine_km(med.loc[keep, "lat"].min(), med.loc[keep, "lon"].min(),
                          med.loc[keep, "lat"].max(), med.loc[keep, "lon"].max())
    print(f"kept {len(keep)} aircraft, max spread ~{spread:.0f} km")

    sel = win[win["icao24"].isin(keep)].copy()

    # 5) lat/lon -> local metres, centred on the cluster
    lat0, lon0 = sel["lat"].mean(), sel["lon"].mean()
    tf = Transformer.from_crs("epsg:4326", "epsg:4978", always_xy=True)  # ECEF
    x0, y0, z0 = tf.transform(lon0, lat0, 0)
    X, Y, Z = tf.transform(sel["lon"].values, sel["lat"].values, sel["z"].values)
    sel["x"] = X - x0
    sel["y"] = Y - y0
    sel["z"] = Z - z0
    sel["t"] = sel["time"] - sel["time"].min()

    out = sel[["icao24", "t", "x", "y", "z", "velocity", "heading"]].sort_values(
        ["icao24", "t"])
    out.to_csv(args.out, index=False)
    print(f"wrote {len(out)} rows for {out['icao24'].nunique()} aircraft to {args.out}")


if __name__ == "__main__":
    main()