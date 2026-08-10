#!/usr/bin/env python3
"""Concurrent stress test for ORB-SLAM3.

Runs several stereo_benchmark instances in parallel, repeatedly, to shake out
crashes and races that only appear under contention. Each instance installs the
crash monitor with a unique run label, so a fault leaves behind a report
containing a backtrace and the SLAM context (frame id, tracking state) instead
of only an exit code.
"""

import argparse
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_DATASET = "dataset/circle_run"
DEFAULT_BENCHMARK = "Examples/Stereo/stereo_benchmark"
DEFAULT_VOCAB = "Vocabulary/ORBvoc.txt"
DEFAULT_CRASH_DIR = "/data/orbslam3_artifacts/crash_reports"
# Peak RSS per benchmark instance in MB (measured empirically on this dataset).
APPROX_RSS_MB = 840
# Leave at least this much free RAM to avoid the OOM killer.
HEADROOM_MB = 1024


def safe_instance_count():
    """Return the number of instances that fit in available memory."""
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemAvailable:"):
                    avail_kb = int(line.split()[1])
                    avail_mb = avail_kb // 1024
                    return max(1, (avail_mb - HEADROOM_MB) // APPROX_RSS_MB)
    except Exception:
        pass
    return 1


def build_command(args, instance_id, crash_dir):
    """Build the benchmark command for one instance.

    Uses a list (never a shell string) so dataset paths containing spaces or
    shell metacharacters cannot be misinterpreted.
    """
    dataset = args.dataset
    settings = os.path.join(dataset, "camera.yaml")
    output_json = os.path.join(args.output_dir, f"bench_stress_{instance_id}.json")
    run_label = f"stress{instance_id}"

    return [
        args.benchmark,
        args.vocabulary,
        settings,
        dataset,
        output_json,
        crash_dir,
        run_label,
    ]


def run_single_instance(args, instance_id, iteration, crash_dir):
    cmd = build_command(args, instance_id, crash_dir)

    print(f"[iter {iteration}][instance {instance_id}] starting")
    start = time.time()
    try:
        res = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            cwd=REPO_ROOT,
            timeout=args.timeout,
        )
    except subprocess.TimeoutExpired:
        print(f"[iter {iteration}][instance {instance_id}] TIMEOUT after {args.timeout}s")
        return {"ok": False, "reason": "timeout", "instance": instance_id}

    duration = time.time() - start

    if res.returncode != 0:
        # Negative return code means terminated by signal N.
        if res.returncode < 0:
            reason = f"signal {-res.returncode}"
        else:
            reason = f"exit code {res.returncode}"
        print(f"[iter {iteration}][instance {instance_id}] CRASHED ({reason})")
        return {
            "ok": False,
            "reason": reason,
            "instance": instance_id,
            "stderr_tail": "\n".join(res.stderr.strip().splitlines()[-20:]),
        }

    print(f"[iter {iteration}][instance {instance_id}] completed in {duration:.2f}s")
    return {"ok": True, "instance": instance_id}


def summarize_failure(iteration, failures, crash_dir):
    print(f"\n!!! {len(failures)} failure(s) in iteration {iteration} !!!")
    for failure in failures:
        print(f"  instance {failure['instance']}: {failure['reason']}")
        tail = failure.get("stderr_tail")
        if tail:
            print("    stderr tail:")
            for line in tail.splitlines():
                print(f"      {line}")

    reports = []
    if os.path.isdir(crash_dir):
        reports = sorted(
            os.path.join(crash_dir, name)
            for name in os.listdir(crash_dir)
            if name.startswith("crash-")
        )
    if reports:
        print(f"  crash reports ({len(reports)}) in {crash_dir}:")
        for report in reports:
            print(f"    {report}")
    else:
        print(f"  no crash reports found in {crash_dir}"
              " (process may have been killed by SIGKILL, e.g. the OOM killer,"
              " which cannot be intercepted)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instances", type=int, default=0,
                        help="concurrent instances (default: auto-sized from available RAM; "
                             "~820 MB/instance on circle_run)")
    parser.add_argument("--iterations", type=int, default=0,
                        help="number of iterations; 0 means run until interrupted")
    parser.add_argument("--dataset", default=DEFAULT_DATASET,
                        help=f"dataset directory (default: {DEFAULT_DATASET})")
    parser.add_argument("--benchmark", default=DEFAULT_BENCHMARK,
                        help=f"benchmark binary (default: {DEFAULT_BENCHMARK})")
    parser.add_argument("--vocabulary", default=DEFAULT_VOCAB,
                        help=f"ORB vocabulary (default: {DEFAULT_VOCAB})")
    parser.add_argument("--crash-dir", default=DEFAULT_CRASH_DIR,
                        help=f"crash report directory (default: {DEFAULT_CRASH_DIR})")
    parser.add_argument("--output-dir", default="/data/orbslam3_artifacts/benchmarks",
                        help="directory for per-instance benchmark JSON "
                             "(default: /data/orbslam3_artifacts/benchmarks). Keep large artifacts on /data:\n"
                             "the root filesystem is small.")
    parser.add_argument("--timeout", type=int, default=1800,
                        help="per-instance timeout in seconds (default: 1800)")
    parser.add_argument("--stop-on-crash", action="store_true",
                        help="exit after the first iteration containing a failure")
    args = parser.parse_args()

    if args.instances <= 0:
        args.instances = safe_instance_count()
        print(f"Auto-sized to {args.instances} instances based on available RAM "
              f"(~{APPROX_RSS_MB} MB/instance, {HEADROOM_MB} MB headroom reserved).")
    elif args.instances * APPROX_RSS_MB > (safe_instance_count() * APPROX_RSS_MB + HEADROOM_MB):
        avail = 0
        try:
            with open("/proc/meminfo") as f:
                for line in f:
                    if line.startswith("MemAvailable:"):
                        avail = int(line.split()[1]) // 1024
        except Exception:
            pass
        print(f"WARNING: {args.instances} instances × ~{APPROX_RSS_MB} MB = "
              f"~{args.instances * APPROX_RSS_MB} MB, but only ~{avail} MB available. "
              f"The OOM killer may fire. Consider --instances {safe_instance_count()}.")

    benchmark_path = os.path.join(REPO_ROOT, args.benchmark)
    if not os.path.exists(benchmark_path):
        print(f"ERROR: benchmark binary not found: {benchmark_path}", file=sys.stderr)
        print("Build it first: cmake --build <build dir> --target stereo_benchmark",
              file=sys.stderr)
        return 2

    dataset_path = os.path.join(REPO_ROOT, args.dataset)
    if not os.path.isdir(dataset_path):
        print(f"ERROR: dataset not found: {dataset_path}", file=sys.stderr)
        return 2

    # os.path.join returns args.crash_dir unchanged when it is absolute.
    crash_dir = os.path.join(REPO_ROOT, args.crash_dir)
    os.makedirs(crash_dir, exist_ok=True)
    output_dir = os.path.join(REPO_ROOT, args.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    args.output_dir = output_dir

    if shutil.which("ulimit") is None:
        pass  # ulimit is a shell builtin; core dump size is left to the caller.

    print(f"Stress test: {args.instances} concurrent instances of {args.dataset}")
    print(f"Crash reports: {crash_dir}")
    print("Press Ctrl+C to stop." if args.iterations == 0
          else f"Running {args.iterations} iteration(s).")

    iteration = 1
    total_failures = 0
    try:
        while args.iterations == 0 or iteration <= args.iterations:
            print(f"\n--- iteration {iteration} ---")
            with ThreadPoolExecutor(max_workers=args.instances) as pool:
                futures = [
                    pool.submit(run_single_instance, args, i, iteration, crash_dir)
                    for i in range(args.instances)
                ]
                results = [f.result() for f in futures]

            failures = [r for r in results if not r["ok"]]
            if failures:
                total_failures += len(failures)
                summarize_failure(iteration, failures, crash_dir)
                if args.stop_on_crash:
                    print("Stopping due to --stop-on-crash.")
                    return 1
            else:
                print(f"iteration {iteration}: all {args.instances} instances succeeded")

            iteration += 1
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStress test stopped by user.")

    print(f"\nCompleted {iteration - 1} iteration(s), {total_failures} failure(s).")
    return 1 if total_failures else 0


if __name__ == "__main__":
    sys.exit(main())
