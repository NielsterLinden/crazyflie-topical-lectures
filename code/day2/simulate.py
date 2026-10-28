"""Day 2 baseline: motion, estimation errors, uncertainty and innovation scale."""

import matplotlib.pyplot as plt

from kalman.simulation import run


def plot(records):
    time = records["time"]
    figure, axes = plt.subplots(3, 1, figsize=(10, 9))
    for values, label in ((records["truth"], "truth"), (records["estimate"], "estimate")):
        axes[0].plot(values[:, 0], values[:, 1], label=label)
    axes[0].set(xlabel="x [m]", ylabel="y [m]", title="Estimation baseline")
    axes[0].legend()
    error = records["estimate"] - records["truth"]
    for index, axis in enumerate("xyz"):
        line, = axes[1].plot(time, error[:, index], label=f"{axis} error")
        sigma = records["position_std"][:, index]
        axes[1].plot(time, 2 * sigma, "--", color=line.get_color(), alpha=0.5)
        axes[1].plot(time, -2 * sigma, "--", color=line.get_color(), alpha=0.5)
    axes[1].axvspan(7, 10, color="0.9", label="position dropout")
    axes[1].set(xlabel="time [s]", ylabel="error [m]", title="Position error and model +/-2 sigma")
    axes[1].legend()
    axes[2].plot(time, records["nis"])
    axes[2].set(xlabel="time [s]", ylabel="maximum NIS", title="Mixed-sensor anomaly locator; E3 supplies per-sensor diagnostics")
    axes[2].set_yscale("symlog", linthresh=1)
    figure.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot(run())
