from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path


def load_plot_module(path: Path):
    spec = importlib.util.spec_from_file_location("flight_log_plot", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load parser: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tool", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=16.0)
    args = parser.parse_args()

    plot = load_plot_module(args.tool)
    header, records = plot.read_log(args.input)
    if not records:
        raise RuntimeError("no valid records found")

    start_s = float(records[0]["time_s"])
    selected = []
    for record in records:
        relative_s = float(record["time_s"]) - start_s
        if relative_s < -1e-9 or relative_s > args.duration + 1e-9:
            continue
        rebased = dict(record)
        rebased["time_s"] = round(relative_s, 3)
        selected.append(rebased)

    args.output.mkdir(parents=True, exist_ok=True)
    csv_path = args.output / "flight_log_0-16s.csv"
    svg_path = args.output / "flight_log_charts_0-16s.svg"
    plot.write_csv(selected, csv_path)
    plot.write_svg(selected, header, svg_path)
    print(f"records={len(selected)}")
    print(f"start_s={selected[0]['time_s']}")
    print(f"end_s={selected[-1]['time_s']}")
    print(f"csv={csv_path}")
    print(f"svg={svg_path}")


if __name__ == "__main__":
    main()
