import argparse
import glob
import os

import numpy as np
import matplotlib.pyplot as plt

from simulink_mat_reader import load_simulink_dataset
from nav_estimator_reader import load_estimator, load_truth_aligned, first_valid_index

QUAT_ESTIMATORS = ["mekf", "rawdog_triad", "rawdog_prop"]
COLORS = {"mekf": "tab:blue", "rawdog_triad": "tab:orange", "rawdog_prop": "tab:green"}

def quat_normalize(q):
    return q / np.linalg.norm(q, axis=1, keepdims=True)

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1[:, 0], q1[:, 1], q1[:, 2], q1[:, 3]
    w2, x2, y2, z2 = q2[:, 0], q2[:, 1], q2[:, 2], q2[:, 3]
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.stack([w, x, y, z], axis=1)

def quat_inv(q):
    conj = q.copy()
    conj[:, 1:] *= -1
    return conj

def quat_to_rotmat(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    N = q.shape[0]
    R = np.empty((N, 3, 3))
    R[:, 0, 0] = 1 - 2 * (y**2 + z**2)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x**2 + z**2)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x**2 + y**2)
    return R

def quat_to_euler_xyz_deg(q):
    R = quat_to_rotmat(q)
    sy = np.clip(-R[:, 2, 0], -1.0, 1.0)
    b = np.arcsin(sy)
    a = np.arctan2(R[:, 2, 1], R[:, 2, 2])
    c = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    return np.rad2deg(np.stack([a, b, c], axis=1))

def wrap_to_180(deg):
    return (deg + 180.0) % 360.0 - 180.0

def estimator_error(truth_time, truth_quat, est):
    raw_state = est["state"][:, :4]
    skip = first_valid_index(raw_state)

    q_est = quat_normalize(raw_state[skip:])
    q_truth, t = load_truth_aligned(truth_time, truth_quat, est["time"])
    q_truth, t = q_truth[skip:], t[skip:]

    q_err = quat_normalize(quat_mul(quat_inv(q_est), q_truth))
    sm_err = q_err[:, 1:4]
    eul_err = wrap_to_180(quat_to_euler_xyz_deg(q_truth) - quat_to_euler_xyz_deg(q_est))
    angle_err_deg = 2 * np.rad2deg(np.arccos(np.clip(np.abs(q_err[:, 0]), -1.0, 1.0)))

    return {"t": t, "sm_err": sm_err, "eul_err": eul_err, "angle_err_deg": angle_err_deg}

def analyze_file(path):
    t_truth, sig = load_simulink_dataset(path)
    q_truth = sig["q_b2icrf"]

    results = {}
    for name in QUAT_ESTIMATORS:
        est = load_estimator(path, name)
        if est is None:
            continue
        results[name] = estimator_error(t_truth, q_truth, est)
    return results

def plot_scenario(name, results, out_dir):
    fig, axes = plt.subplots(len(results) + 1, 1, figsize=(10, 3 * (len(results) + 1)), sharex=False)
    if len(results) == 0:
        plt.close(fig)
        return None

    ax = axes[0]
    for est_name, r in results.items():
        ax.plot(r["t"], r["angle_err_deg"], label=est_name, color=COLORS.get(est_name))
    ax.set_yscale("log")
    ax.set_ylabel("Total attitude\nerror (deg, log)")
    ax.set_title(f"Truth vs. estimator attitude error — {name}")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3, which="both")

    colors = ["r", "g", "b"]
    labels = ["X", "Y", "Z"]
    for i, (est_name, r) in enumerate(results.items(), start=1):
        ax = axes[i]
        for j in range(3):
            ax.plot(r["t"], r["sm_err"][:, j], colors[j], label=labels[j])
        ax.set_ylabel(f"{est_name}\nvector err")
        ax.legend(loc="upper right", ncol=3)
        ax.grid(alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    out_path = os.path.join(out_dir, f"{name}_estimator_error.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path

def plot_summary(all_results, out_dir):
    scenario_names = list(all_results.keys())
    est_names = QUAT_ESTIMATORS

    fig, ax = plt.subplots(figsize=(12, 6))
    width = 0.8 / len(est_names)
    x = np.arange(len(scenario_names))

    for k, est_name in enumerate(est_names):
        rms_vals = []
        for scenario in scenario_names:
            r = all_results[scenario].get(est_name)
            if r is None:
                rms_vals.append(np.nan)
            else:
                rms_vals.append(np.sqrt(np.mean(r["angle_err_deg"] ** 2)))
        ax.bar(x + k * width, rms_vals, width, label=est_name, color=COLORS.get(est_name))

    ax.set_xticks(x + width * (len(est_names) - 1) / 2)
    ax.set_xticklabels(scenario_names, rotation=45, ha="right")
    ax.set_ylabel("RMS attitude error (deg)")
    all_vals = [
        np.sqrt(np.mean(r["angle_err_deg"] ** 2))
        for scenario in all_results.values()
        for r in scenario.values()
    ]
    if all_vals and np.nanmin(all_vals) > 0:
        ax.set_yscale("log")
    ax.set_title("Estimator RMS attitude error vs. truth, across scenarios")
    ax.legend()
    ax.grid(alpha=0.3, axis="y", which="both")
    fig.tight_layout()
    out_path = os.path.join(out_dir, "summary_estimator_comparison.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--out-dir", default="figures")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    mat_files = sorted(glob.glob(os.path.join(args.data_dir, "*.mat")))
    if not mat_files:
        print(f"No .mat files found in {args.data_dir}")
        return

    all_results = {}
    for path in mat_files:
        name = os.path.splitext(os.path.basename(path))[0]
        print(f"Processing {name} ...")
        results = analyze_file(path)
        if not results:
            print(f"  no navigation estimator data found in {name}, skipping")
            continue
        all_results[name] = results
        out_path = plot_scenario(name, results, args.out_dir)
        print(f"  wrote {out_path}")

    if all_results:
        summary_path = plot_summary(all_results, args.out_dir)
        print(f"Wrote summary: {summary_path}")

if __name__ == "__main__":
    main()
