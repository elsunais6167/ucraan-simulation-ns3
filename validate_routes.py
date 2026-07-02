#!/usr/bin/env python3
"""Offline oracle: for each route slot, check every mesh hop against TRUE
geometry. A hop whose true distance > mesh range is a blackhole in ns-3."""
import sys, numpy as np, pandas as pd

mob, R = sys.argv[1], float(sys.argv[2])
df = pd.read_csv(mob)
ids = list(dict.fromkeys(df["icao24"])); n=len(ids)
tr = {i:(g.sort_values("t")["t"].values, g.sort_values("t")[["x","y","z"]].values)
      for i,(a,g) in enumerate(df.groupby("icao24", sort=False))}
def pos(i,t):
    ts,X = tr[i]
    if t<ts[0] or t>ts[-1]: return None
    return np.array([np.interp(t,ts,X[:,k]) for k in range(3)])

for f in sys.argv[3:]:
    r = pd.read_csv(f); bad=tot=0
    for t,g in r.groupby("time"):
        for _,row in g.iterrows():
            nh = row["nexthop_ip"]
            if not nh.startswith("10.1."): continue   # sat legs always "up" here
            a = int(row["node"]); b = int(nh.split(".")[-1])-1
            pa,pb = pos(a,t),pos(b,t)
            tot += 1
            if pa is None or pb is None or np.linalg.norm(pa-pb) > R: bad += 1
    print(f"{f}: {tot} mesh-hop commits, {bad} phantom ({100*bad/max(tot,1):.1f}%)")