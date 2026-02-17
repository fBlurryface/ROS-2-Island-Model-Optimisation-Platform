#!/usr/bin/env python3
import argparse
import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")  # headless friendly (CI)
import matplotlib.pyplot as plt


def read_csv(path: str):
    xs, ys = [], []
    with open(path, "r", encoding="utf-8", newline="") as f:
        r = csv.reader(f)
        _ = next(r, None)  # header: Generation,Best_Fitness
        for row in r:
            if len(row) < 2:
                continue
            try:
                g = int(row[0])
                fit = float(row[1])
            except ValueError:
                continue
            xs.append(g)
            ys.append(fit)
    return xs, ys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True, help="e.g. evo_logs/run_YYYYMMDD_HHMMSS")
    ap.add_argument("--out", required=True, help="output png path")
    ap.add_argument("--max-points", type=int, default=0, help="plot only last N points (0=all)")
    ap.add_argument("--ylog", action="store_true", help="log-scale y axis")
    args = ap.parse_args()

    run_dir = args.run_dir
    out_path = args.out

    csv_files = sorted(glob.glob(os.path.join(run_dir, "*.csv")))
    if not csv_files:
        raise SystemExit(f"No CSV files found in: {run_dir}")

    plt.figure(figsize=(10, 6))
    for p in csv_files:
        xs, ys = read_csv(p)
        if not xs:
            continue
        if args.max_points and len(xs) > args.max_points:
            xs = xs[-args.max_points:]
            ys = ys[-args.max_points:]
        plt.plot(xs, ys, linewidth=1, alpha=0.85)

    plt.title(f"Convergence (all islands)\n{os.path.basename(run_dir)}")
    plt.xlabel("Generation")
    plt.ylabel("Best Fitness")
    if args.ylog:
        plt.yscale("log")
    plt.grid(True, which="both", alpha=0.3)

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
