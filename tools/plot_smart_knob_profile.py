#!/usr/bin/env python3
"""Compare restored SmartKnob targets with the former localized detent law.

The restored curve is sampled from the repository's compiled C++ Haptics,
not a Python reimplementation. This is an algorithm simulation, not a motor
measurement. Requires matplotlib and a host g++ or clang++ compiler.
"""

import argparse
import csv
import io
import math
import os
from pathlib import Path
import shutil
import subprocess

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "build/smart-knob-restore-20261104/profile-comparison.png"
SAMPLER = r"""#include "control.hpp"
#include <iomanip>
#include <iostream>

int main() {
    knob::Config config;
    config.mode = knob::Mode::detents;
    config.width = knob::pi / 18;
    config.strength = 20;
    config.current_limit = .8f;
    config.damping = 0;
    config.snap = .55f;
    knob::Haptics haptics;
    haptics.reset(0);
    std::cout << "angle_deg,restored_target_a,position\n" << std::setprecision(9);
    constexpr int samples = 4000;
    for (int sample = 0; sample <= samples; ++sample) {
        const float angle = config.width * (2.0f * sample / samples);
        const float target = haptics.step(config, angle, 0);
        std::cout << (20.0 * sample / samples) << ',' << target << ','
                  << haptics.position() << '\n';
    }
}
"""


def former_target(angle_deg):
    """Former v5 geometric barrier: .4/.6-pitch peaks, independent of snap."""
    unit = angle_deg / 10
    unit -= math.floor(unit + .5)
    x = max(0, min(1, (abs(unit) - .30) / .20))
    peak = min(.8, 20 * math.radians(10) * .5)
    return -math.copysign(peak * math.sin(math.pi * x) ** 2, unit)


def sample_current_source(output_dir, compiler):
    source = output_dir / "smart_knob_profile_sampler.cpp"
    executable = output_dir / ("smart_knob_profile_sampler.exe" if os.name == "nt"
                               else "smart_knob_profile_sampler")
    source.write_text(SAMPLER, encoding="utf-8")
    subprocess.run([
        compiler, "-std=c++20", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(ROOT / "components/knob_core/include"),
        str(source), str(ROOT / "components/knob_core/src/control.cpp"),
        "-o", str(executable),
    ], check=True)
    result = subprocess.run([str(executable)], check=True, capture_output=True,
                            text=True)
    rows = []
    for raw in csv.DictReader(io.StringIO(result.stdout)):
        angle = float(raw["angle_deg"])
        rows.append({
            "angle_deg": angle,
            "restored_target_a": float(raw["restored_target_a"]),
            "former_v5_target_a": former_target(angle),
            "position": int(raw["position"]),
        })
    if len(rows) != 4001:
        raise RuntimeError(f"Expected 4001 source samples, received {len(rows)}")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help="Figure path; CSV and SVG are saved beside it")
    parser.add_argument("--cxx", default=os.environ.get("CXX"),
                        help="Path to host g++/clang++; defaults to CXX or PATH")
    args = parser.parse_args()
    compiler = args.cxx or shutil.which("g++") or shutil.which("clang++")
    if not compiler:
        parser.error("A host g++/clang++ is required; pass --cxx <compiler path>")
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = sample_current_source(output.parent, compiler)
    csv_path = output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    font_path = Path("C:/Windows/Fonts/msyh.ttc")
    if font_path.exists():
        plt.rcParams["font.family"] = FontProperties(fname=font_path).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["svg.fonttype"] = "path"

    angles = [row["angle_deg"] for row in rows]
    restored = [row["restored_target_a"] for row in rows]
    former = [row["former_v5_target_a"] for row in rows]
    positions = [row["position"] for row in rows]
    fig, (torque_ax, position_ax) = plt.subplots(
        2, 1, figsize=(12, 7.4), dpi=160, sharex=True,
        gridspec_kw={"height_ratios": [3, 1], "hspace": .16})
    fig.suptitle("SmartKnob 挡位恢复：算法目标对照（非实测）", fontsize=17, y=.97)
    fig.text(.5, .91, "正向扫描 0–20°  ·  档距 10°  ·  strength 20 A/rad  ·  限流 0.8 A  ·  damping 0  ·  snap 0.55",
             ha="center", fontsize=10, color="#586675")
    torque_ax.plot(angles, former, color="#b87549", lw=2,
                   label="原 v5：独立的局部周期力矩")
    torque_ax.plot(angles, restored, color="#167566", lw=2.3,
                   label="恢复后：当前 Haptics 源码采样")
    torque_ax.axhline(0, color="#596b7e", lw=.8)
    torque_ax.set(ylabel="目标电流（A）", ylim=(-1.02, 1.20))
    torque_ax.legend(loc="lower center", bbox_to_anchor=(.5, 1.01),
                     ncol=2, fontsize=10, frameon=False)
    for edge in (5.5, 15.5):
        for ax in (torque_ax, position_ax):
            ax.axvline(edge, color="#6c7c8c", lw=1, ls="--", zorder=0)
        torque_ax.text(edge, 1.02, f"{edge:g}° 换挡", ha="center",
                       fontsize=10, color="#536475")
    torque_ax.annotate("旧峰值提前出现", xy=(4, -.8), xytext=(1.1, -.97),
                       color="#9a603c", fontsize=9,
                       arrowprops={"arrowstyle": "->", "color": "#9a603c"})
    torque_ax.annotate("阻力 → 助力，与换挡同步", xy=(15.505, .5),
                       xytext=(10.5, .98), color="#167566", fontsize=10,
                       arrowprops={"arrowstyle": "->", "color": "#167566"})
    position_ax.step(angles, positions, where="post", color="#304c68", lw=2)
    position_ax.set(ylabel="逻辑挡位", xlabel="相对起始中心的机械角度（°，正向）",
                    xlim=(0, 20), ylim=(-.2, 2.3), yticks=[0, 1, 2],
                    xticks=list(range(0, 21, 2)))
    for ax in (torque_ax, position_ax):
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=.17)
    fig.text(.5, .035,
             "固定角度扫点，速度输入为零；只比较 Haptics 的目标，不包含电流环、实测折返限流、点击或高速包络。\n"
             "恢复后曲线直接编译本仓库 C++ 源码生成；原 v5 曲线按已移除的周期公式计算。步长 0.005°。",
             ha="center", fontsize=9, color="#586675", linespacing=1.7)
    fig.subplots_adjust(left=.085, right=.975, bottom=.15, top=.81)
    fig.savefig(output, facecolor="white")
    svg_path = output.with_suffix(".svg")
    if svg_path != output:
        fig.savefig(svg_path, facecolor="white")
    plt.close(fig)
    for path in (output, svg_path, csv_path):
        print(path)
    transitions = [row["angle_deg"] for previous, row in zip(rows, rows[1:])
                   if row["position"] != previous["position"]]
    print(f"Source samples: {len(rows)}; observed shifts (deg): {transitions}")


if __name__ == "__main__":
    main()
