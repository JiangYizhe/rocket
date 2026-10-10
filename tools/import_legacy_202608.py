from __future__ import annotations

import csv
import hashlib
import math
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DESTINATION_ROOT = REPO_ROOT / "archive" / "legacy_202608"
SOURCE_ROOT = Path(
    r"C:\Users\91321\Desktop\火箭飞控pcb\新火箭飞控板"
    r"\stm32-Rocket-FinControl_20260728 (2)\stm32-Rocket-FinControl_20260728"
)
SOURCES = [
    ("main", SOURCE_ROOT / "stm32-Rocket-FinControl"),
    (
        "backup_before_bench_20260821",
        SOURCE_ROOT / "stm32-Rocket-FinControl_backup_before_bench_20260821",
    ),
    (
        "weighted_backup_before_bench_20260821",
        SOURCE_ROOT
        / "加配重"
        / "stm32-Rocket-FinControl_backup_before_bench_20260821",
    ),
]
MANIFEST_FIELDS = [
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


def hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def hash_directory(directory: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in directory.rglob("*") if item.is_file()):
        digest.update(path.relative_to(directory).as_posix().encode("utf-8"))
        digest.update(path.stat().st_size.to_bytes(8, "little"))
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
    return digest.hexdigest()


def finite_float(value: str | None) -> float | None:
    try:
        result = float(value or "nan")
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def select_csv(directory: Path) -> Path | None:
    for name in ("flight_log.csv", "flight_log_0-16s.csv", "flight_log_0_16s.csv"):
        candidate = directory / name
        if candidate.is_file():
            return candidate
    candidates = sorted(directory.rglob("*.csv"))
    return candidates[0] if candidates else None


def csv_stats(path: Path | None) -> dict[str, object]:
    result: dict[str, object] = {
        "csv_file": "",
        "records": "",
        "duration_s": "",
        "peak_altitude_m": "",
        "stage2_delay_ms": "",
        "ignition_first_s": "",
    }
    if path is None:
        return result

    times: list[float] = []
    altitudes: list[float] = []
    delays: list[float] = []
    ignition_times: list[float] = []
    records = 0
    try:
        with path.open(encoding="utf-8-sig", newline="") as stream:
            for row in csv.DictReader(stream):
                records += 1
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
        return result

    result["csv_file"] = path.name
    result["records"] = records
    if times:
        result["duration_s"] = f"{max(times) - min(times):.3f}"
    if altitudes:
        result["peak_altitude_m"] = f"{max(altitudes):.3f}"
    if delays:
        result["stage2_delay_ms"] = f"{delays[0]:.0f}"
    if ignition_times:
        result["ignition_first_s"] = f"{min(ignition_times):.3f}"
    return result


def manifest_row(source_name: str, directory: Path, digest: str) -> dict[str, object]:
    files = [path for path in directory.rglob("*") if path.is_file()]
    row: dict[str, object] = {
        "kind": "legacy_directory",
        "source_name": source_name,
        "repository_path": directory.relative_to(REPO_ROOT).as_posix(),
        "file_count": len(files),
        "total_bytes": sum(path.stat().st_size for path in files),
        "sha256": digest,
    }
    row.update(csv_stats(select_csv(directory)))
    return row


def destination_for(name: str, variant: str, digest: str) -> Path:
    candidate = DESTINATION_ROOT / name
    if not candidate.exists() or hash_directory(candidate) == digest:
        return candidate
    return DESTINATION_ROOT / f"{name}__{variant}__{digest[:8]}"


def import_entries() -> tuple[list[dict[str, object]], list[dict[str, str]]]:
    DESTINATION_ROOT.mkdir(parents=True, exist_ok=True)
    canonical_by_digest: dict[str, Path] = {}
    manifest_rows: list[dict[str, object]] = []
    source_rows: list[dict[str, str]] = []

    for variant, source_root in SOURCES:
        if not source_root.is_dir():
            source_rows.append(
                {
                    "source_variant": variant,
                    "source_path": str(source_root),
                    "entry_type": "source_root",
                    "content_sha256": "",
                    "status": "missing",
                    "repository_path": "",
                }
            )
            continue

        entries = sorted(source_root.glob("flight_log_*"), key=lambda path: path.name.lower())
        for source in entries:
            if source.is_dir():
                digest = hash_directory(source)
                entry_type = "directory"
            elif source.is_file() and source.suffix.lower() == ".zip":
                digest = hash_file(source)
                entry_type = "zip"
            else:
                continue

            if digest in canonical_by_digest:
                destination = canonical_by_digest[digest]
                status = "duplicate"
            elif source.is_dir():
                destination = destination_for(source.name, variant, digest)
                shutil.copytree(source, destination, dirs_exist_ok=True, copy_function=shutil.copy2)
                canonical_by_digest[digest] = destination
                manifest_rows.append(manifest_row(source.name, destination, digest))
                status = "canonical"
            else:
                destination = DESTINATION_ROOT / "raw" / source.name
                destination.parent.mkdir(parents=True, exist_ok=True)
                if destination.exists() and hash_file(destination) != digest:
                    destination = destination.with_name(
                        f"{destination.stem}__{variant}__{digest[:8]}{destination.suffix}"
                    )
                shutil.copy2(source, destination)
                canonical_by_digest[digest] = destination
                status = "canonical"

            source_rows.append(
                {
                    "source_variant": variant,
                    "source_path": str(source),
                    "entry_type": entry_type,
                    "content_sha256": digest,
                    "status": status,
                    "repository_path": destination.relative_to(REPO_ROOT).as_posix(),
                }
            )

    return manifest_rows, source_rows


def write_sources(rows: list[dict[str, str]]) -> None:
    fields = [
        "source_variant",
        "source_path",
        "entry_type",
        "content_sha256",
        "status",
        "repository_path",
    ]
    with (DESTINATION_ROOT / "SOURCES.csv").open(
        "w", encoding="utf-8-sig", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def update_manifest(new_rows: list[dict[str, object]]) -> None:
    manifest_path = REPO_ROOT / "MANIFEST.csv"
    rows: list[dict[str, object]] = []
    if manifest_path.is_file():
        with manifest_path.open(encoding="utf-8-sig", newline="") as stream:
            rows = [
                row
                for row in csv.DictReader(stream)
                if not row.get("repository_path", "").startswith("archive/legacy_202608/")
            ]
    rows.extend(new_rows)
    rows.sort(key=lambda row: str(row.get("repository_path", "")).lower())
    with manifest_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    manifest_rows, source_rows = import_entries()
    write_sources(source_rows)
    update_manifest(manifest_rows)
    canonical = sum(row["status"] == "canonical" for row in source_rows)
    duplicates = sum(row["status"] == "duplicate" for row in source_rows)
    print(
        f"Imported {canonical} unique entries; mapped {duplicates} duplicates; "
        f"updated {REPO_ROOT / 'MANIFEST.csv'}"
    )


if __name__ == "__main__":
    main()
