#!/usr/bin/env python3
# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | AC-TEB (ECC 2027): table and figures from the runs in ~/ECC             |
# +-------------------------------------------------------------------------+
"""
Reads every run written by ecc_logger.py

    <results_dir>/<YYYY-MM-DD_HH-MM-SS>_<controller>/{timeseries,summary}.csv

and writes into <results_dir>/figures/

    results_table.csv / .tex   mean +- std per controller over its runs
                               (success over all runs, the rest over the
                               successful ones)
    fig_trajectories.pdf/.png  top view of the route, one run per controller
    fig_ey.pdf/.png            e_y along the route, one run per controller
    fig_estimation.pdf/.png    AC-TEB: estimated vs true width and gamma_c

Usage (no ROS needed, only numpy + matplotlib):
    ros2 run ac_teb_local_planner ecc_plots.py
    ros2 run ac_teb_local_planner ecc_plots.py --results ~/ECC --map <map.yaml> --run last
    python3 ecc_plots.py --results ~/ECC
Options
    --run first|last|best   run of each controller drawn in the figures
                            (best = lowest e_y RMS among successful runs)
    --controllers ...       order and subset (default ac_teb teb mppi dwb rpp)
    --map map.yaml          draws the occupancy map below the trajectories
"""

import argparse
import csv
import glob
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

NAMES = {"ac_teb": "AC-TEB", "teb": "TEB", "mppi": "MPPI", "dwb": "DWB", "rpp": "RPP"}
COLORS = {"ac_teb": "#2a78d6", "teb": "#eb6834", "mppi": "#1baf7a", "dwb": "#4a3aa7",
          "rpp": "#eda100"}
STYLES = {"ac_teb": "-", "teb": "--", "mppi": "-.", "dwb": (0, (5, 1, 1, 1, 1, 1)), "rpp": ":"}
METRICS = [  # (summary column, table label, decimals)
    ("success_pct", "Success [\\%]", 0),
    ("time_s", "Time [s]", 1),
    ("e_y_rms_cm", "$e_y$ RMS [cm]", 1),
    ("e_y_max_cm", "$\\max|e_y|$ [cm]", 1),
    ("clearance_min_cm", "Min. clearance [cm]", 1),
    ("compute_time_mean_ms", "Comp. time [ms]", 2),
]

plt.rcParams.update({
    "font.family": "serif", "font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "axes.grid": True, "grid.color": "#e1e0d9",
    "grid.linewidth": 0.5, "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.1, "savefig.dpi": 300, "pdf.fonttype": 42, "mathtext.fontset": "cm",
})
COL_W = 3.45   # IEEE single column [in]


# ------------------------------------------------------------------- reading
def read_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return rows


def to_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def load_runs(root):
    runs = []
    for d in sorted(glob.glob(os.path.join(root, "*_*"))):
        s = os.path.join(d, "summary.csv")
        t = os.path.join(d, "timeseries.csv")
        if not os.path.isdir(d) or not os.path.exists(t):
            continue
        summ = read_csv(s)[0] if os.path.exists(s) else {"status": "no_summary", "success_pct": "0"}
        ts_rows = read_csv(t)
        if not ts_rows:
            continue
        ctrl = ts_rows[0]["controller"]
        ts = {k: np.array([to_float(r[k]) for r in ts_rows]) for k in ts_rows[0] if k != "controller"}
        ts["dist"] = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(ts["x"]), np.diff(ts["y"])))])
        runs.append({"dir": d, "controller": ctrl, "summary": summ, "ts": ts})
    return runs


def pick(runs, ctrl, how):
    rc = [r for r in runs if r["controller"] == ctrl]
    if not rc:
        return None
    if how == "first":
        return rc[0]
    if how == "last":
        return rc[-1]
    ok = [r for r in rc if r["summary"].get("status") == "success"] or rc
    return min(ok, key=lambda r: to_float(r["summary"].get("e_y_rms_cm")))


# --------------------------------------------------------------------- table
def table(runs, ctrls, out):
    rows_csv, cols = [], []
    for c in ctrls:
        rc = [r for r in runs if r["controller"] == c]
        if not rc:
            continue
        ok = [r for r in rc if r["summary"].get("status") == "success"]
        cells = {"controller": c, "runs": len(rc)}
        for key, _, _ in METRICS:
            src = rc if key == "success_pct" else ok
            v = np.array([to_float(r["summary"].get(key)) for r in src])
            v = v[np.isfinite(v)]
            cells[key + "_mean"] = float(v.mean()) if v.size else float("nan")
            cells[key + "_std"] = float(v.std(ddof=1)) if v.size > 1 else 0.0 if v.size else float("nan")
        rows_csv.append(cells)
        cols.append(c)
    if not rows_csv:
        return
    with open(os.path.join(out, "results_table.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows_csv[0]))
        wr.writeheader()
        wr.writerows(rows_csv)
    # LaTeX: metrics in rows, controllers in columns (fits one column)
    lines = ["\\begin{tabular}{l" + "c" * len(cols) + "}", "\\hline",
             " & " + " & ".join(NAMES.get(c, c) for c in cols) + " \\\\", "\\hline"]
    for key, label, nd in METRICS:
        cells = []
        for r in rows_csv:
            m, s = r[key + "_mean"], r[key + "_std"]
            if not math.isfinite(m):
                cells.append("--")
            elif key == "success_pct":
                cells.append(f"{m:.0f}")
            else:
                cells.append(f"{m:.{nd}f} $\\pm$ {s:.{nd}f}")
        lines.append(f"{label} & " + " & ".join(cells) + " \\\\")
    lines += ["\\hline", "\\end{tabular}"]
    with open(os.path.join(out, "results_table.tex"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


# ------------------------------------------------------------------- figures
def load_map(yaml_path):
    import yaml  # noqa: PLC0415  (only needed with --map)
    with open(yaml_path) as f:
        m = yaml.safe_load(f)
    img_path = m["image"] if os.path.isabs(m["image"]) else os.path.join(os.path.dirname(yaml_path), m["image"])
    img = plt.imread(img_path)
    if img.ndim == 3:
        img = img[..., 0]
    res, (ox, oy) = m["resolution"], m["origin"][:2]
    h, w = img.shape
    return img, [ox, ox + w * res, oy, oy + h * res]


def save(fig, out, name):
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(out, f"{name}.{ext}"), bbox_inches="tight")
    plt.close(fig)
    print("written", os.path.join(out, name + ".pdf"))


def fig_trajectories(sel, out, map_yaml):
    fig, ax = plt.subplots(figsize=(COL_W, COL_W))
    if map_yaml:
        img, ext = load_map(map_yaml)
        ax.imshow(img, cmap="gray", origin="upper", extent=ext, alpha=0.6, zorder=0)
    for c, r in sel:
        ts = r["ts"]
        ax.plot(ts["x"], ts["y"], color=COLORS.get(c), ls=STYLES.get(c, "-"), label=NAMES.get(c, c))
    ax.set_aspect("equal")
    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    ax.grid(False)
    ax.legend(frameon=False, ncol=len(sel), loc="lower center", bbox_to_anchor=(0.5, 1.0))
    save(fig, out, "fig_trajectories")


def fig_ey(sel, out):
    fig, ax = plt.subplots(figsize=(COL_W, 1.8))
    for c, r in sel:
        ts = r["ts"]
        ey = np.where(ts["in_aisle"] > 0.5, 100 * ts["e_y"], np.nan)   # only inside the rows
        ax.plot(ts["dist"], ey, color=COLORS.get(c), ls=STYLES.get(c, "-"), label=NAMES.get(c, c))
    ax.axhline(0, color="#898781", lw=0.6)
    ax.set_xlabel("distance along the route [m]")
    ax.set_ylabel("$e_y$ [cm]")
    ax.legend(frameon=False, ncol=len(sel), loc="lower center", bbox_to_anchor=(0.5, 1.0))
    save(fig, out, "fig_ey")


def fig_estimation(run, out, gamma_max):
    ts = run["ts"]
    if not np.isfinite(ts["est_W"]).any():
        print("fig_estimation skipped: the AC-TEB run has no aisle estimate")
        return
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(COL_W, 2.2), sharex=True,
                                 gridspec_kw={"height_ratios": [1.6, 1], "hspace": 0.12})
    d = ts["dist"]
    inside = ts["in_aisle"] > 0.5
    valid = ts["est_valid"] > 0.5
    a1.plot(d, np.where(inside, ts["W_true"], np.nan), color="#898781", label="true $W$")
    a1.plot(d, np.where(valid, ts["est_W"], np.nan), color=COLORS["ac_teb"], label=r"$\hat W$")
    a1.set_ylabel("$W$ [m]")
    a1.legend(frameon=False, ncol=2, loc="upper right")
    lam = np.nan_to_num(ts["gamma_c"], nan=0.0) / gamma_max
    a2.fill_between(d, 0, lam, color=COLORS["ac_teb"], alpha=0.25, lw=0)
    a2.plot(d, lam, color=COLORS["ac_teb"])
    a2.set_ylim(-0.05, 1.1)
    a2.set_yticks([0, 1])
    a2.set_ylabel(r"$\gamma_c/\gamma_c^{\max}$")
    a2.set_xlabel("distance along the route [m]")
    for ax in (a1, a2):   # headlands / turns shaded
        ax.fill_between(d, 0, 1, where=~inside, transform=ax.get_xaxis_transform(),
                        color="#e1e0d9", alpha=0.6, lw=0)
    save(fig, out, "fig_estimation")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default=os.path.expanduser("~/ECC"))
    ap.add_argument("--controllers", nargs="+", default=["ac_teb", "teb", "mppi", "dwb", "rpp"])
    ap.add_argument("--run", choices=["first", "last", "best"], default="best")
    ap.add_argument("--map", default=None, help="map yaml drawn below the trajectories")
    ap.add_argument("--gamma-max", type=float, default=30.0)
    a = ap.parse_args()

    root = os.path.expanduser(a.results)
    runs = load_runs(root)
    if not runs:
        print(f"no runs found in {root}")
        return
    out = os.path.join(root, "figures")
    os.makedirs(out, exist_ok=True)
    print(f"{len(runs)} runs: " + ", ".join(
        f"{c}={sum(r['controller'] == c for r in runs)}" for c in a.controllers))

    table(runs, a.controllers, out)
    sel = [(c, pick(runs, c, a.run)) for c in a.controllers]
    sel = [(c, r) for c, r in sel if r is not None]
    for c, r in sel:
        print(f"figures use {c}: {os.path.basename(r['dir'])}")
    fig_trajectories(sel, out, a.map)
    fig_ey(sel, out)
    ac = dict(sel).get("ac_teb")
    if ac:
        fig_estimation(ac, out, a.gamma_max)


if __name__ == "__main__":
    main()
