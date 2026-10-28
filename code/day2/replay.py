"""Day 2 offline replay and plotting of prepared sensor recordings."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from kalman.filter import FilterConfig
from kalman.recording import load_recording, replay


def save_results(results, metadata, config, output):
    import matplotlib.pyplot as plt

    paths = [Path(str(output) + suffix) for suffix in (".csv", ".png", ".json")]
    if any(path.exists() for path in paths):
        raise ValueError("output already exists; choose a new --output prefix")
    output.parent.mkdir(parents=True, exist_ok=True)
    with paths[0].open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    times = [row["time_s"] for row in results]
    figure, axes = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
    for index, axis in enumerate("xyz"):
        position = np.array([row[f"p{axis}_m"] for row in results])
        sigma = np.array([row[f"sigma_{axis}_m"] for row in results])
        measured = [float(row[f"measurement_{axis}_m"]) if row[f"measurement_{axis}_m"] != "" else np.nan
                    for row in results]
        innovation = [float(row[f"innovation_{axis}_m"]) if row[f"innovation_{axis}_m"] != "" else np.nan
                      for row in results]
        axes[index, 0].plot(times, position, label="estimate")
        axes[index, 0].plot(times, measured, ".", label="observation")
        axes[index, 0].fill_between(times, position - 1.96 * sigma, position + 1.96 * sigma,
                                    alpha=0.2, label="model 95% band")
        axes[index, 0].set_ylabel(f"{axis} position [m]")
        # Mark isolated updates too: NaN gaps should not make sparse innovations invisible.
        axes[index, 1].plot(times, innovation, ".-")
        axes[index, 1].set_ylabel(f"{axis} innovation [m]")
    axes[0, 0].legend(fontsize="small")
    for plot in axes[-1]:
        plot.set_xlabel("Recording time [s]")
    figure.tight_layout()
    figure.savefig(paths[1], dpi=150)
    plt.close(figure)
    paths[2].write_text(json.dumps({"recording": metadata, "filter": vars(config)}, indent=2) + "\n")
    print(f"Wrote {paths[0]}, {paths[1]} and {paths[2]}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--metadata", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--position-std", type=float, default=0.04, help="assumed position std in metres")
    args = parser.parse_args(argv)
    if not math.isfinite(args.position_std) or args.position_std <= 0:
        parser.error("--position-std must be finite and positive")
    try:
        samples, metadata = load_recording(args.recording, args.metadata)
        config = FilterConfig(position_noise_std=args.position_std)
        save_results(replay(samples, config), metadata, config, args.output)
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))
    print("Offline only. Observations are not independent truth; this is not flight qualification.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
