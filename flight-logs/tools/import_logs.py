from __future__ import annotations

import csv
import hashlib
import io
import math
import shutil
import subprocess
import zipfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPO_ROOT.parent
ARCHIVE_ROOT = REPO_ROOT / "archive"
CAPTURE_ROOT = REPO_ROOT / "captures"
EXTERNAL_ROOT = REPO_ROOT / "external"
TOOLS_ROOT = REPO_ROOT / "tools" / "legacy"
EXTERNAL_ZIPS = [
    Path(
        r"E:\xwechat_files\wxid_wl4d4lcunbcs22_3129\msg\file\2026-09"
        r"\COM25_log_export_20260928_001527.zip"
    )
]


def copy_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        source_stat = source.stat()
        destination_stat = destination.stat()
        if (
            source_stat.st_size == destination_stat.st_size
            and source_stat.st_mtime_ns == destination_stat.st_mtime_ns
        ):
            return
    shutil.copy2(source, destination)


def copy_tree(source: Path, destination: Path) -> None:
    for path in sorted(source.rglob("*")):
        if path.is_file():
            copy_file(path, destination / path.relative_to(source))


def finite_float(value: str | None) -> float | None:
    try:
        result = float(value or "nan")
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def select_csv(directory: Path) -> Path | None:
    preferred = [
        "flight_log_0-16s.csv",
        "flight_log_0_16s.csv",
        "flight_log.csv",
    ]
    for name in preferred:
        candidate = directory / name
        if candidate.is_file():
            return candidate
    candidates = sorted(directory.rglob("*.csv"))
    return candidates[0] if candidates else None


def csv_stats(path: Path | None) -> dict[str, object]:
    stats: dict[str, object] = {
        "csv_file": "",
        "records": "",
        "duration_s": "",
        "peak_altitude_m": "",
        "stage2_delay_ms": "",
        "ignition_first_s": "",
    }
    if path is None:
        return stats

    times: list[float] = []
    altitudes: list[float] = []
    delays: list[float] = []
    ignition_times: list[float] = []
    record_count = 0
    try:
        with path.open(encoding="utf-8-sig", newline="") as stream:
            for row in csv.DictReader(stream):
                record_count += 1
                time_s = finite_float(row.get("time_s"))
                altitude = finite_float(row.get("relative_altitude_m"))
                delay = finite_float(row.get("config_ignition_delay_ms"))
                ignition = finite_float(row.get("ignition_active"))
                if time_s is not None:
                    times.append(time_s)
                if altitude is not None:
                    altitudes.append(altitude)
                if delay is not None:
                    delays.append(delay)
                if time_s is not None and ignition not in (None, 0.0):
                    ignition_times.append(time_s)
    except (OSError, UnicodeError, csv.Error):
        return stats

    stats["csv_file"] = path.name
    stats["records"] = record_count
    if times:
        stats["duration_s"] = f"{max(times) - min(times):.3f}"
    if altitudes:
        stats["peak_altitude_m"] = f"{max(altitudes):.3f}"
    if delays:
        stats["stage2_delay_ms"] = f"{delays[0]:.0f}"
    if ignition_times:
        stats["ignition_first_s"] = f"{min(ignition_times):.3f}"
    return stats


def directory_digest(directory: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in directory.rglob("*") if item.is_file()):
        digest.update(path.relative_to(directory).as_posix().encode("utf-8"))
        digest.update(path.stat().st_size.to_bytes(8, "little"))
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
    return digest.hexdigest()


def manifest_row(kind: str, source_name: str, directory: Path) -> dict[str, object]:
    files = [path for path in directory.rglob("*") if path.is_file()]
    row: dict[str, object] = {
        "kind": kind,
        "source_name": source_name,
        "repository_path": directory.relative_to(REPO_ROOT).as_posix(),
        "file_count": len(files),
        "total_bytes": sum(path.stat().st_size for path in files),
        "sha256": directory_digest(directory),
    }
    row.update(csv_stats(select_csv(directory)))
    return row


def import_source_directories() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    ARCHIVE_ROOT.mkdir(parents=True, exist_ok=True)
    for source in sorted(SOURCE_ROOT.iterdir(), key=lambda path: path.name.lower()):
        if not source.is_dir() or source == REPO_ROOT:
            continue
        if not (source.name.startswith("flight_log_") or source.name.startswith("_log_")):
            continue
        destination = ARCHIVE_ROOT / source.name
        copy_tree(source, destination)
        rows.append(manifest_row("directory", source.name, destination))
    return rows


def import_standalone_files() -> list[dict[str, object]]:
    CAPTURE_ROOT.mkdir(parents=True, exist_ok=True)
    patterns = ["_log_*_serial_capture.txt"]
    for pattern in patterns:
        for source in sorted(SOURCE_ROOT.glob(pattern)):
            copy_file(source, CAPTURE_ROOT / source.name)

    legacy_names = [
        "_compare_flight_logs.py",
        "_make_chart.py",
        "_make_full_chart.py",
        "_make_mix_chart.py",
        "_rebase_flight_log.py",
    ]
    for name in legacy_names:
        source = SOURCE_ROOT / name
        if source.is_file():
            copy_file(source, TOOLS_ROOT / source.name)

    rows: list[dict[str, object]] = []
    if any(CAPTURE_ROOT.iterdir()):
        rows.append(manifest_row("captures", "standalone serial captures", CAPTURE_ROOT))
    return rows


def import_external_archives() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for source in EXTERNAL_ZIPS:
        if not source.is_file():
            continue
        raw_destination = EXTERNAL_ROOT / "raw" / source.name
        copy_file(source, raw_destination)
        extracted = EXTERNAL_ROOT / source.stem
        extracted.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(source) as archive:
            archive.extractall(extracted)
        rows.append(manifest_row("external", source.name, extracted))
    return rows


def write_manifest(rows: list[dict[str, object]]) -> None:
    fields = [
        "kind",
        "source_name",
        "repository_path",
        "file_count",
        "total_bytes",
        "csv_file",
        "records",
        "duration_s",
        "peak_altitude_m",
        "stage2_delay_ms",
        "ignition_first_s",
        "sha256",
    ]
    with (REPO_ROOT / "MANIFEST.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_existing_rows(kind: str) -> list[dict[str, object]]:
    manifest = REPO_ROOT / "MANIFEST.csv"
    if manifest.is_file():
        with manifest.open(encoding="utf-8-sig", newline="") as stream:
            rows = [row for row in csv.DictReader(stream) if row.get("kind") == kind]
        if rows:
            return rows

    result = subprocess.run(
        ["git", "show", "HEAD:MANIFEST.csv"],
        cwd=REPO_ROOT,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        return []
    text = result.stdout.decode("utf-8-sig")
    return [row for row in csv.DictReader(io.StringIO(text)) if row.get("kind") == kind]


def main() -> None:
    rows = []
    rows.extend(import_source_directories())
    rows.extend(read_existing_rows("legacy_directory"))
    rows.extend(import_standalone_files())
    rows.extend(import_external_archives())
    write_manifest(rows)
    print(f"Imported {len(rows)} manifest entries into {REPO_ROOT}")


if __name__ == "__main__":
    main()
