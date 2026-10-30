import h5py
import numpy as np

ESTIMATORS = ["mekf", "rawdog_triad", "rawdog_prop", "od_kf"]

def _orient(arr, n_time):
    arr = np.asarray(arr)
    if arr.ndim == 1:
        arr = arr.reshape(-1, 1)
    if arr.shape[0] == n_time:
        return arr
    if arr.shape[1] == n_time:
        return arr.T
    raise ValueError(f"Neither axis of shape {arr.shape} matches time length {n_time}")

def load_estimator(path, estimator):
    with h5py.File(path, "r") as f:
        time_key = f"nav_{estimator}_time"
        state_key = f"nav_{estimator}_state"
        p_key = f"nav_{estimator}_P"

        if time_key not in f:
            return None

        time = np.array(f[time_key][()]).flatten()
        state = _orient(f[state_key][()], len(time))

        P = None
        if p_key in f:
            p_raw = np.array(f[p_key][()])

            dims = p_raw.shape
            if len(dims) == 3:
                if dims[1] == dims[2]:
                    P = p_raw
                elif dims[0] == dims[1]:
                    P = np.transpose(p_raw, (2, 0, 1))
                else:
                    P = p_raw

        return {"time": time, "state": state, "P": P}

def load_truth_aligned(truth_time, truth_quat, est_time):
    n = min(len(truth_time), len(est_time))

    if np.allclose(truth_time[:n], est_time[:n], atol=1e-6):
        return truth_quat[:n], est_time[:n]

    out = np.empty((len(est_time), truth_quat.shape[1]))
    for i in range(truth_quat.shape[1]):
        out[:, i] = np.interp(est_time, truth_time, truth_quat[:, i])
    out /= np.linalg.norm(out, axis=1, keepdims=True)
    return out, est_time

def first_valid_index(state):
    nonzero_rows = np.any(state != 0, axis=1)
    idx = np.argmax(nonzero_rows)
    return idx if nonzero_rows[idx] else 0
