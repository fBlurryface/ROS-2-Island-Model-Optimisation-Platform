#!/usr/bin/env python3
import csv
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import glob
import time

# Optional: limit points per curve for faster refresh on huge runs.
# Set EVO_MONITOR_MAX_POINTS=2000 to only plot the last 2000 generations.
MAX_POINTS = int(os.getenv("EVO_MONITOR_MAX_POINTS", "0"))

def read_generation_fitness(csv_path: str, max_points: int = 0):
    """Read Generation/Best_Fitness from a 2-column CSV written by island_node (no pandas)."""
    if max_points and max_points > 0:
        gens = deque(maxlen=max_points)
        fits = deque(maxlen=max_points)
        add_g, add_f = gens.append, fits.append
    else:
        gens, fits = [], []
        add_g, add_f = gens.append, fits.append

    try:
        with open(csv_path, "r", encoding="utf-8", newline="") as f:
            reader = csv.reader(f)
            _header = next(reader, None)  # Generation,Best_Fitness
            for row in reader:
                if len(row) < 2:
                    continue
                try:
                    g = int(row[0])
                    fit = float(row[1])
                except ValueError:
                    continue
                add_g(g)
                add_f(fit)
    except FileNotFoundError:
        return [], []
    except Exception:
        return [], []

    if isinstance(gens, deque):
        return list(gens), list(fits)
    return gens, fits

def get_latest_run_dir(base_dir):
    """找到最新的 run_XXXXXXXX_XXXXXX 文件夹"""
    dirs = glob.glob(os.path.join(base_dir, "run_*"))
    if not dirs:
        return None
    return max(dirs, key=os.path.getmtime)

def animate(i, base_dir, ax):
    run_dir = get_latest_run_dir(base_dir)
    if not run_dir:
        print("Waiting for log directory...")
        return

    csv_files = glob.glob(os.path.join(run_dir, "*.csv"))
    if not csv_files:
        return

    ax.clear()
    # 使用 Tab20 颜色循环，支持更多岛屿
    cmap = plt.get_cmap('tab20')

    # 记录所有岛屿的最终结果用于排序图例
    latest_results = []

    for idx, f in enumerate(csv_files):
        try:
            island_id = os.path.basename(f).replace(".csv", "")

            gens, fits = read_generation_fitness(f, MAX_POINTS)
            if not gens:
                continue

            # 画线
            ax.plot(gens, fits, color=cmap(idx % 20), linewidth=1, alpha=0.7)

            latest_results.append((island_id, fits[-1]))
        except Exception:
            continue

    ax.set_yscale('log')
    ax.set_title(f"Real-time Local Log Monitor\nSource: {os.path.basename(run_dir)}")
    ax.set_xlabel("Generations")
    ax.set_ylabel("Best Fitness (Log)")
    ax.grid(True, which="both", ls="-", alpha=0.3)

    # 简易图例（取前5个岛屿展示，防止挡住图）
    if latest_results:
        latest_results.sort(key=lambda x: x[1])
        legend_labels = [f"{name}: {val:.2e}" for name, val in latest_results[:5]]
        ax.legend(legend_labels, loc='upper right', fontsize='x-small')

def main():
    # 日志根目录，必须与 C++ 中的 log_dir 参数对应
    base_log_dir = os.path.join(os.getcwd(), "evo_logs")

    fig, ax = plt.subplots(figsize=(10, 6))

    print(f"🔍 Monitoring directory: {base_log_dir}")

    # 每 1 秒刷新一次界面
    _ani = FuncAnimation(fig, animate, fargs=(base_log_dir, ax), interval=1000)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
