import argparse
import json
import subprocess
import sys
from pathlib import Path


def load_orbits(orbits_path: Path) -> list[dict]:
    with open(orbits_path, "r") as f:
        config = json.load(f)

    if isinstance(config, list):
        orbits = config
    else:
        orbits = config.get("orbits", [])

    if not orbits:
        raise ValueError(f"No orbits found in {orbits_path}")

    return orbits


def run_one_orbit(matlab_exe: str, sims_dir: Path, orbit: dict, index: int, out_path: Path) -> bool:
    name = orbit.get("name", f"orbit_{index}")

    param_path = sims_dir / f"_tmp_params_{name}.json"
    with open(param_path, "w") as pf:
        json.dump(orbit, pf)

    matlab_cmd = (
        f"try; "
        f"run_simulink_batch('{param_path.as_posix()}', '{out_path.as_posix()}'); "
        f"catch e; disp(getReport(e)); exit(1); end; "
        f"exit(0);"
    )

    result = subprocess.run(
        [matlab_exe, "-batch", matlab_cmd],
        cwd=str(sims_dir),
    )

    param_path.unlink(missing_ok=True)

    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description="Batch-run orbits through run_simulink.m")
    parser.add_argument(
        "--orbits",
        type=Path,
        default=Path("orbits_example.json"),
        help="Path to the JSON file listing orbits (default: orbits_example.json)",
    )
    parser.add_argument(
        "--sims-dir",
        type=Path,
        default=Path("Simulations"),
        help="Path to the Simulations folder containing run_simulink_batch.m (default: Simulations)",
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path("data"),
        help="Output folder for results (default: data)",
    )
    parser.add_argument(
        "--matlab",
        type=str,
        default="matlab",
        help="Path to the MATLAB executable (default: 'matlab' on PATH)",
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip orbits whose output .mat file already exists in the data folder",
    )
    args = parser.parse_args()

    sims_dir = args.sims_dir.resolve()
    data_dir = args.data_dir.resolve()
    data_dir.mkdir(parents=True, exist_ok=True)

    if not (sims_dir / "run_simulink_batch.m").exists():
        print(
            f"WARNING: {sims_dir / 'run_simulink_batch.m'} not found. "
            "Make sure it's in place before running.",
            file=sys.stderr,
        )

    orbits = load_orbits(args.orbits)

    failures = []
    for i, orbit in enumerate(orbits):
        name = orbit.get("name", f"orbit_{i}")
        out_path = data_dir / f"{name}.mat"

        if args.skip_existing and out_path.exists():
            print(f"[{i+1}/{len(orbits)}] {name}: already exists, skipping")
            continue

        print(f"[{i+1}/{len(orbits)}] {name}: running...")
        ok = run_one_orbit(args.matlab, sims_dir, orbit, i, out_path)

        if ok:
            print(f"    -> saved {out_path}")
        else:
            print(f"    -> FAILED (see MATLAB output above)")
            failures.append(name)

    print()
    print(f"Done. {len(orbits) - len(failures)}/{len(orbits)} orbits succeeded.")
    if failures:
        print("Failed orbits:", ", ".join(failures))
        sys.exit(1)


if __name__ == "__main__":
    main()
