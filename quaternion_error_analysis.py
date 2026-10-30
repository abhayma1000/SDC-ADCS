import argparse
import glob
import os

import numpy as np
import matplotlib.pyplot as plt

from simulink_mat_reader import load_simulink_dataset, load_orbit_name

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

def rotmat_to_quat(R):
    N = R.shape[0]
    q = np.empty((N, 4))
    tr = R[:, 0, 0] + R[:, 1, 1] + R[:, 2, 2]

    for i in range(N):
        Ri = R[i]
        t = tr[i]
        if t > 0:
            S = np.sqrt(t + 1.0) * 2
            w = 0.25 * S
            x = (Ri[2, 1] - Ri[1, 2]) / S
            y = (Ri[0, 2] - Ri[2, 0]) / S
            z = (Ri[1, 0] - Ri[0, 1]) / S
        elif Ri[0, 0] > Ri[1, 1] and Ri[0, 0] > Ri[2, 2]:
            S = np.sqrt(1.0 + Ri[0, 0] - Ri[1, 1] - Ri[2, 2]) * 2
            w = (Ri[2, 1] - Ri[1, 2]) / S
            x = 0.25 * S
            y = (Ri[0, 1] + Ri[1, 0]) / S
            z = (Ri[0, 2] + Ri[2, 0]) / S
        elif Ri[1, 1] > Ri[2, 2]:
            S = np.sqrt(1.0 + Ri[1, 1] - Ri[0, 0] - Ri[2, 2]) * 2
            w = (Ri[0, 2] - Ri[2, 0]) / S
            x = (Ri[0, 1] + Ri[1, 0]) / S
            y = 0.25 * S
            z = (Ri[1, 2] + Ri[2, 1]) / S
        else:
            S = np.sqrt(1.0 + Ri[2, 2] - Ri[0, 0] - Ri[1, 1]) * 2
            w = (Ri[1, 0] - Ri[0, 1]) / S
            x = (Ri[0, 2] + Ri[2, 0]) / S
            y = (Ri[1, 2] + Ri[2, 1]) / S
            z = 0.25 * S
        q[i] = [w, x, y, z]

    return quat_normalize(q)

def quat_to_euler_xyz_deg(q):
    R = quat_to_rotmat(q)

    sy = np.clip(-R[:, 2, 0], -1.0, 1.0)
    b = np.arcsin(sy)
    a = np.arctan2(R[:, 2, 1], R[:, 2, 2])
    c = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    return np.rad2deg(np.stack([a, b, c], axis=1))

def wrap_to_180(deg):
    return (deg + 180.0) % 360.0 - 180.0

def nadir_reference_quat(position, velocity):
    r_hat = position / np.linalg.norm(position, axis=1, keepdims=True)
    h = np.cross(position, velocity)
    h_hat = h / np.linalg.norm(h, axis=1, keepdims=True)

    z_lvlh = -r_hat
    y_lvlh = -h_hat
    x_lvlh = np.cross(y_lvlh, z_lvlh)
    x_lvlh /= np.linalg.norm(x_lvlh, axis=1, keepdims=True)

    R = np.stack([x_lvlh, y_lvlh, z_lvlh], axis=2)
    return rotmat_to_quat(R)

def analyze_file(path):
    t, sig = load_simulink_dataset(path)
    q_output = quat_normalize(sig["q_b2icrf"])
    q_expected = nadir_reference_quat(sig["position_icrf"], sig["velocity_icrf"])

    q_err = quat_mul(quat_inv(q_expected), q_output)
    q_err = quat_normalize(q_err)

    sm_err = q_err[:, 1:4]

    eul_out = quat_to_euler_xyz_deg(q_output)
    eul_exp = quat_to_euler_xyz_deg(q_expected)
    eul_err = wrap_to_180(eul_out - eul_exp)

    angle_err_deg = 2 * np.rad2deg(np.arccos(np.clip(np.abs(q_err[:, 0]), -1.0, 1.0)))

    return {
        "t": t,
        "sm_err": sm_err,
        "eul_err": eul_err,
        "angle_err_deg": angle_err_deg,
    }

def plot_scenario(name, result, out_dir):
    t = result["t"]
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    colors = ["r", "g", "b"]
    labels = ["X", "Y", "Z"]

    ax = axes[0]
    for i in range(3):
        ax.plot(t, result["sm_err"][:, i], colors[i], label=labels[i])
    ax.set_ylabel("Small-angle error\n(vector part, unitless)")
    ax.set_title(f"Quaternion error vs nadir-pointing reference — {name}")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)

    ax = axes[1]
    for i in range(3):
        ax.plot(t, result["eul_err"][:, i], colors[i], label=labels[i])
    ax.set_ylabel("Euler angle error\n(XYZ, deg)")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)

    ax = axes[2]
    ax.plot(t, result["angle_err_deg"], "k")
    ax.set_ylabel("Total pointing\nerror (deg)")
    ax.set_xlabel("Time (s)")
    ax.grid(alpha=0.3)

    fig.tight_layout()
    out_path = os.path.join(out_dir, f"{name}_quat_error.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path

def plot_summary(all_results, out_dir):
    names = list(all_results.keys())
    rms = [np.sqrt(np.mean(r["angle_err_deg"] ** 2)) for r in all_results.values()]
    peak = [np.max(r["angle_err_deg"]) for r in all_results.values()]

    order = np.argsort(rms)
    names = [names[i] for i in order]
    rms = [rms[i] for i in order]
    peak = [peak[i] for i in order]

    x = np.arange(len(names))
    fig, ax = plt.subplots(figsize=(11, 6))
    width = 0.35
    ax.bar(x - width / 2, rms, width, label="RMS pointing error (deg)")
    ax.bar(x + width / 2, peak, width, label="Peak pointing error (deg)")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=45, ha="right")
    ax.set_ylabel("Pointing error (deg)")
    ax.set_title("Nadir-pointing error vs simulated attitude, across scenarios")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    out_path = os.path.join(out_dir, "summary_comparison.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", required=True, help="Directory containing .mat files")
    parser.add_argument("--out-dir", default="figures", help="Directory to write figures to")
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
        result = analyze_file(path)
        all_results[name] = result
        out_path = plot_scenario(name, result, args.out_dir)
        print(f"  wrote {out_path}")

    summary_path = plot_summary(all_results, args.out_dir)
    print(f"Wrote summary: {summary_path}")

if __name__ == "__main__":
    main()
