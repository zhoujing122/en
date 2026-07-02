#!/usr/bin/env python3
import argparse
import csv
import hashlib
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


REQUIRED_OUTPUTS = [
    "config.resolved.yaml",
    "map.pgm",
    "map.yaml",
    "trajectory.csv",
    "localization_log.csv",
    "tof_log.csv",
    "metrics.csv",
]

SPIN_SCAN_OUTPUTS = [
    "spin_scan.csv",
    "spin_scan_bins.csv",
    "spin_match_curve.csv",
    "spin_match_summary.csv",
]


def md5_file(path):
    h = hashlib.md5()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def run_file(path):
    proc = subprocess.run(["file", str(path)], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return proc.returncode, (proc.stdout + proc.stderr).strip()


def parse_config(path):
    section = ""
    kv = {}
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip() or ":" not in line:
            continue
        indent = len(line) - len(line.lstrip(" "))
        key, value = line.strip().split(":", 1)
        value = value.strip().strip("\"'")
        if indent == 0 and not value:
            section = key.strip()
            continue
        full = f"{section}.{key.strip()}" if section else key.strip()
        kv[full] = value
    return kv


def bool_value(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def read_total_cpu_ticks():
    with Path("/proc/stat").open() as f:
        first = f.readline().split()
    if not first or first[0] != "cpu":
        return 0
    return sum(int(v) for v in first[1:])


def read_proc_stat(pid):
    data = Path(f"/proc/{pid}/stat").read_text()
    close = data.rfind(")")
    rest = data[close + 2 :].split()
    utime = int(rest[11])
    stime = int(rest[12])
    vsize = int(rest[20])
    rss_pages = int(rest[21])
    return utime + stime, vsize, rss_pages


def read_proc_status(pid):
    out = {"threads": 0, "rss_kb": 0, "vms_kb": 0}
    with Path(f"/proc/{pid}/status").open() as f:
        for line in f:
            if line.startswith("Threads:"):
                out["threads"] = int(line.split()[1])
            elif line.startswith("VmRSS:"):
                out["rss_kb"] = int(line.split()[1])
            elif line.startswith("VmSize:"):
                out["vms_kb"] = int(line.split()[1])
    return out


def sample_process(pid, start_time, last_proc_ticks, last_total_ticks, clk_tck, cpu_count):
    now = time.time()
    proc_ticks, vsize, rss_pages = read_proc_stat(pid)
    total_ticks = read_total_cpu_ticks()
    status = read_proc_status(pid)
    delta_proc = proc_ticks - last_proc_ticks
    delta_total = total_ticks - last_total_ticks
    cpu_percent = 0.0
    if delta_total > 0:
        cpu_percent = (float(delta_proc) / float(delta_total)) * float(cpu_count) * 100.0
    page_kb = os.sysconf("SC_PAGE_SIZE") // 1024
    rss_kb = status["rss_kb"] if status["rss_kb"] else rss_pages * page_kb
    vms_kb = status["vms_kb"] if status["vms_kb"] else vsize // 1024
    return {
        "timestamp_s": now,
        "elapsed_s": now - start_time,
        "pid": pid,
        "cpu_percent": cpu_percent,
        "rss_kb": rss_kb,
        "vms_kb": vms_kb,
        "thread_count": status["threads"],
        "proc_ticks": proc_ticks,
        "total_ticks": total_ticks,
    }


def latest_run(output_dir):
    latest = output_dir / "latest"
    if latest.exists():
        return latest.resolve()
    runs = output_dir / "runs"
    if not runs.exists():
        return None
    items = sorted([p for p in runs.iterdir() if p.is_dir()])
    return items[-1] if items else None


def write_profile_header(path):
    f = path.open("w", newline="")
    writer = csv.DictWriter(
        f,
        fieldnames=["timestamp_s", "elapsed_s", "pid", "cpu_percent", "rss_kb", "vms_kb", "thread_count"],
    )
    writer.writeheader()
    return f, writer


def run_smoke(args):
    binary = Path(args.binary)
    config = Path(args.config)
    output_dir = Path(args.output_dir)

    failures = []
    if not binary.exists():
        failures.append(f"binary not found: {binary}")
    if not config.exists():
        failures.append(f"config not found: {config}")
    if failures:
        for item in failures:
            print(f"error: {item}")
        return 1

    rc, file_text = run_file(binary)
    binary_md5 = md5_file(binary)
    print(f"binary: {binary}")
    print(f"file: {file_text}")
    print(f"md5: {binary_md5}")
    if rc != 0 or "ELF 32-bit" not in file_text or "ARM" not in file_text:
        failures.append("binary is not ELF 32-bit ARM")
    if args.expected_md5 and binary_md5 != args.expected_md5:
        failures.append(f"md5 mismatch: expected {args.expected_md5}, got {binary_md5}")

    cfg = parse_config(config)
    spin_enabled = bool_value(cfg.get("spin_scan_localization.enabled", "false"))
    output_dir.mkdir(parents=True, exist_ok=True)
    profile_tmp = output_dir / "runtime_profile.csv.tmp"
    profile_file, profile_writer = write_profile_header(profile_tmp)

    cmd = [str(binary), "--config", str(config), "--duration-s", str(args.duration_s), "--output-dir", str(output_dir)]
    print("running: " + " ".join(cmd))
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    start = time.time()
    clk_tck = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    cpu_count = os.cpu_count() or 1
    try:
        last_proc_ticks, _, _ = read_proc_stat(proc.pid)
        last_total_ticks = read_total_cpu_ticks()
        while proc.poll() is None:
            time.sleep(args.sample_period_s)
            if not Path(f"/proc/{proc.pid}").exists():
                break
            row = sample_process(proc.pid, start, last_proc_ticks, last_total_ticks, clk_tck, cpu_count)
            last_proc_ticks = row.pop("proc_ticks")
            last_total_ticks = row.pop("total_ticks")
            profile_writer.writerow(row)
            profile_file.flush()
    finally:
        stdout, stderr = proc.communicate()
        profile_file.close()

    print(stdout, end="")
    print(stderr, end="", file=sys.stderr)
    if proc.returncode != 0:
        failures.append(f"robot_slamd exited with {proc.returncode}")

    run_dir = latest_run(output_dir)
    if not run_dir:
        failures.append("run_dir/latest not generated")
    else:
        final_profile = run_dir / "runtime_profile.csv"
        shutil.move(str(profile_tmp), str(final_profile))
        print(f"run_dir: {run_dir}")
        print(f"runtime_profile_csv: {final_profile}")
        required = REQUIRED_OUTPUTS + (SPIN_SCAN_OUTPUTS if spin_enabled else [])
        missing = [name for name in required if not (run_dir / name).exists()]
        if missing:
            failures.append("missing outputs: " + ",".join(missing))

    if failures:
        print("board_runtime_smoke: fail")
        for item in failures:
            print(f"  {item}")
        return 1
    print("board_runtime_smoke: ok")
    return 0


def main():
    parser = argparse.ArgumentParser(description="Run robot_slamd board runtime smoke and lightweight /proc profiling.")
    parser.add_argument("--binary", required=True, help="Path to board robot_slamd binary")
    parser.add_argument("--config", required=True, help="Path to board smoke config")
    parser.add_argument("--output-dir", required=True, help="Output root for robot_slamd")
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--sample-period-s", type=float, default=1.0)
    parser.add_argument("--expected-md5", default="")
    args = parser.parse_args()
    raise SystemExit(run_smoke(args))


if __name__ == "__main__":
    main()
