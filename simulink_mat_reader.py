import h5py
import numpy as np

def _decode_str(arr):
    return "".join(chr(int(c)) for c in np.array(arr).flatten() if c != 0).strip("<>")

def load_simulink_dataset(path):
    signals = {}
    with h5py.File(path, "r") as f:
        t = np.array(f["tout"][0, :])
        refs = f["#refs#"]
        sigstream = f["#sigstream#"]

        for key in refs.keys():
            obj = refs[key]
            if not isinstance(obj, h5py.Group) or "Name" not in obj or "Values" not in obj:
                continue

            name = _decode_str(obj["Name"][:])
            if not name:
                continue

            values = obj["Values"]
            if "SignalAttributes" not in values or "DataR2" not in values:
                continue

            dim = int(values["SignalAttributes"]["Dimension"][0, 0])
            data_idx = int(values["DataR2"][0, 0])

            raw = sigstream[str(data_idx)]["#data#"][()]
            arr = np.frombuffer(raw.tobytes(), dtype="<f8").reshape(-1, dim)

            if arr.shape[0] != t.shape[0]:
                continue

            signals[name] = arr

    return t, signals

def load_orbit_name(path):
    try:
        with h5py.File(path, "r") as f:
            if "orbit_params" in f and "name" in f["orbit_params"]:
                return _decode_str(f["orbit_params/name"][:])
    except Exception:
        pass
    return None
