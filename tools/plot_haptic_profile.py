#!/usr/bin/env python3
"""Plot the old/current commanded detent profiles, not measured motor torque."""
import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("docs/haptic-profile.png"))
    args = parser.parse_args()
    font_path = Path("C:/Windows/Fonts/msyh.ttc")
    if font_path.exists():
        plt.rcParams["font.family"] = FontProperties(fname=font_path).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    width = math.pi / 18
    angles = [20 * n / 1600 for n in range(1601)]
    old, new = [], []
    center = 0
    for degrees in angles:
        angle = math.radians(degrees)
        error = angle - center
        if error > width * .65:
            center += width
            error = angle - center
        old.append(max(-.35, min(.35, -4 * (error - max(-width*.1, min(width*.1, error))))))
        unit = angle / width
        unit -= math.floor(unit + .5)
        x = max(0, min(1, (abs(unit) - .30) / .20))
        new.append(-math.copysign(.6 * math.sin(math.pi*x)**2, unit))
    fig, ax = plt.subplots(figsize=(11, 5.2), dpi=150)
    for start, end in [(0, 3), (7, 13), (17, 20)]:
        ax.axvspan(start, end, color="#e3f4eb", zorder=0)
    ax.plot(angles, old, color="#8c929e", lw=2, label="原设备参数：全域弹力，0.35 A 上限")
    ax.plot(angles, new, color="#167566", lw=2.5, label="当前参数：跨档区集中，0.6 A 上限")
    for position in [5, 15]:
        ax.axvline(position, color="#b3bbc2", lw=1, ls="--")
    ax.axhline(0, color="#546475", lw=.8)
    ax.set(xlim=(0, 20), ylim=(-.93, 1.02), xlabel="正向转动角度（°），档距 10°",
           ylabel="目标电流（A，编码器方向）", title="跨档前阻挡，跨档后助推；绿色空隙内目标为零")
    ax.text(10, .88, "空隙内不叠加阻尼", ha="center", color="#167566", fontsize=10)
    ax.legend(loc="upper center", bbox_to_anchor=(.5, -.18), fontsize=9, frameon=False, ncol=2)
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="y", alpha=.15)
    fig.text(.5, .035, "算法目标曲线：速度设为零，未叠加点击、限流、速度包络或电流环响应；并非实测转矩。",
             ha="center", fontsize=8, color="#61707e")
    fig.subplots_adjust(left=.085, right=.98, bottom=.28, top=.87)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, facecolor="white")
    print(args.output)


if __name__ == "__main__":
    main()
