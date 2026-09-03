#!/usr/bin/env python3
import json
import os
import subprocess
import sys

import time

def run_benchmark(dataset_path, output_json):
    cmd = [
        "./Examples/Stereo/stereo_benchmark",
        "Vocabulary/ORBvoc.txt",
        f"{dataset_path}/camera.yaml",
        dataset_path,
        output_json
    ]
    print(f"==================================================")
    print(f"Starting ORB-SLAM3 Baseline Benchmark on: {dataset_path}")
    print(f"==================================================")
    nix_bin = "/nix/store/g5kyy7iiizp0swcxgjz6q0g9hhjm17gy-determinate-nix-3.21.8/bin/nix"
    full_cmd = [nix_bin, "develop", "--command"] + cmd if os.path.exists(nix_bin) else cmd
    res = subprocess.run(full_cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print(f"[CRASH DETECTED] Benchmark exited with code {res.returncode} on {dataset_path}")
        crash_log_path = "docs/superpowers/artifacts/benchmark_crash_dump.log"
        with open(crash_log_path, "a") as crash_file:
            crash_file.write(f"\n--- CRASH LOG ({dataset_path}) ---\n")
            crash_file.write(f"Return code: {res.returncode}\n")
            crash_file.write(res.stderr + "\n")
        print(f"Crash details captured to: {crash_log_path}")
        return None
    if os.path.exists(output_json):
        with open(output_json, 'r') as f:
            return json.load(f)
    return None

def main():
    datasets = ["dataset/circle_run"]
    results = {}
    for ds in datasets:
        if os.path.exists(ds):
            json_file = f"/tmp/bench_{os.path.basename(ds)}.json"
            res = run_benchmark(ds, json_file)
            if res:
                results[os.path.basename(ds)] = res
            time.sleep(3)

    os.makedirs("docs/superpowers/artifacts", exist_ok=True)
    out_json_path = "docs/superpowers/artifacts/baseline_benchmark_results.json"
    with open(out_json_path, 'w') as f:
        json.dump(results, f, indent=2)

    report_path = "docs/superpowers/artifacts/baseline_benchmark_report.md"
    with open(report_path, 'w') as f:
        f.write("# ORB-SLAM3 Baseline Performance Benchmark Report\n\n")
        f.write("## System Hardware Specifications\n")
        f.write("- **CPU**: AMD Ryzen 7 5825U (8 Cores / 16 Threads, up to 4.55 GHz)\n")
        f.write("- **RAM**: 22 GiB System Memory\n")
        f.write("- **OS**: NixOS (Linux x86_64)\n")
        f.write("- **Compiler**: GCC 15.3.0 (Nix Environment)\n\n")
        f.write("## Baseline Metrics\n\n")
        f.write("| Dataset | Total Frames | Tracked | Elapsed (s) | Mean Latency (ms) | StdDev (ms) | Median P50 (ms) | P90 (ms) | P95 (ms) | **Avg Tracking Frequency** | Peak Memory (MB) |\n")
        f.write("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |\n")
        for ds_name, data in results.items():
            f.write(f"| `{ds_name}` | {data['total_frames']} | {data['tracked_frames']} | {data['total_duration_sec']:.2f} s | {data['mean_latency_ms']:.2f} ms | {data['stddev_ms']:.2f} ms | {data['p50_latency_ms']:.2f} ms | {data['p90_latency_ms']:.2f} ms | {data['p95_latency_ms']:.2f} ms | **{data['avg_fps']:.2f} Hz (FPS)** | {data['peak_rss_mb']:.1f} MB |\n")

        f.write("\n\n## Sub-component Optimization Targets\n")
        f.write("1. **ORB Feature Extraction**: Parallelize multi-threaded pyramid level extraction across CPU worker threads.\n")
        f.write("2. **Stereo Descriptor Matching**: Vectorize epipolar search using AVX2 SIMD intrinsics.\n")
        f.write("3. **Local BA Optimization**: Streamline Hessian structure updates during keyframe culling.\n")

    print(f"\nSaved baseline results to:\n  - {out_json_path}\n  - {report_path}")

if __name__ == "__main__":
    main()
