# Commands

Run everything from `<project-root>`.

## Run the orbit batch

Basic (uses `orbits_example.json`, `Simulations\noise_profile.json`, writes to `Simulations`-relative `data\`):
```powershell
python run_orbits.py
```

With explicit data output folder:
```powershell
python run_orbits.py --data-dir "<project-root>\data"
```

With a specific orbit file and data folder:
```powershell
python run_orbits.py --orbits orbits_example.json --data-dir "<project-root>\data"
```

With a specific noise profile:
```powershell
python run_orbits.py --orbits orbits_example.json --data-dir "<project-root>\data" --noise-profile "<project-root>\Simulations\noise_profile.json"
```

Skip orbits whose `.mat` already exists in the data folder:
```powershell
python run_orbits.py --orbits orbits_example.json --data-dir "<project-root>\data" --skip-existing
```

### Diagnostic noise profiles used while debugging

All go in `Simulations\`, all run the same way as the `--noise-profile` example above, just swap the filename:

| File | What it isolates |
|---|---|
| `noise_profile_disabled.json` | Everything zeroed — noise fully off |
| `noise_profile_lowgyro.json` | Tiny gyro noise only |
| `noise_profile_lowmag.json` | Tiny gyro + real magnetometer noise |
| `noise_profile_lowgps.json` | Tiny gyro + real GPS noise |
| `noise_profile_tinygps.json` | Tiny gyro + near-zero GPS noise |

## Generate graphs

Estimator error + roll/pitch/yaw covariance bound plots:
```powershell
python estimator_error_analysis.py --data-dir "<project-root>\data" --out-dir "<project-root>\figures"
```

Quaternion (nadir-pointing) comparison plots:
```powershell
python quaternion_error_analysis.py --data-dir "<project-root>\data" --out-dir "<project-root>\figures"
```

Both in one line:
```powershell
python estimator_error_analysis.py --data-dir "<project-root>\data" --out-dir "<project-root>\figures"; python quaternion_error_analysis.py --data-dir "<project-root>\data" --out-dir "<project-root>\figures"
```

## One-time setup

```powershell
pip install matplotlib h5py
```
