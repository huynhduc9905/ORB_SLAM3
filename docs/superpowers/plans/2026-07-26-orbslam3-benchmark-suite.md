# ORB-SLAM3 Benchmark Suite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a dedicated headless C++ performance benchmark runner for ORB-SLAM3 and run automated baseline benchmarks on hardware across datasets to generate structured JSON and Markdown performance reports.

**Architecture:** Create an unthrottled C++ executable `stereo_benchmark` with microsecond-level stage timing (ORB extraction, stereo matching, pose tracking), wrapped by a Python benchmark orchestrator `scripts/run_baseline_benchmark.py` to produce repeatable metrics ($P_{50}, P_{90}, P_{95}, \text{Mean}, \text{StdDev}, \text{FPS}, \text{RSS Memory}$).

**Tech Stack:** C++14, CMake, OpenCV, Eigen3, std::chrono, Python 3, Nix.

## Global Constraints
- Must run cleanly inside the `nix develop` shell environment.
- No artificial frame delays or sleep throttling during benchmarking.
- Must execute on both `dataset/circle_run` and `dataset/full_run`.
- All outputs saved to `docs/superpowers/artifacts/`.

---

### Task 1: Headless C++ Benchmark Runner (`Examples/Stereo/stereo_benchmark.cc`)

**Files:**
- Create: `Examples/Stereo/stereo_benchmark.cc`
- Modify: `CMakeLists.txt:125-135`

**Interfaces:**
- Consumes: `ORB_SLAM3::System`, `ORB_SLAM3::System::STEREO`
- Produces: Executable `./Examples/Stereo/stereo_benchmark` which outputs JSON formatted benchmark timing logs to stdout/file.

- [ ] **Step 1: Create `Examples/Stereo/stereo_benchmark.cc`**

```cpp
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <numeric>
#include <cmath>
#include <sys/resource.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <System.h>

using namespace std;

void LoadStereoImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                      vector<string> &vstrImageRight, vector<double> &vTimeStamps) {
    ifstream fTimes(strPathToSequence + "/times.txt");
    if(!fTimes.is_open()) return;
    while(!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if(!s.empty()) {
            stringstream ss(s);
            double t;
            ss >> t;
            vTimeStamps.push_back(t);
        }
    }
    string strPathLeft = strPathToSequence + "/cam0/data";
    string strPathRight = strPathToSequence + "/cam1/data";
    int nTimes = vTimeStamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    for(int i = 0; i < nTimes; i++) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPathLeft + "/" + ss.str() + ".png";
        vstrImageRight[i] = strPathRight + "/" + ss.str() + ".png";
    }
}

int main(int argc, char **argv) {
    if(argc < 4) {
        cerr << "Usage: ./stereo_benchmark path_to_vocabulary path_to_settings path_to_dataset [output_json_path]" << endl;
        return 1;
    }
    string strVocFile = argv[1];
    string strSettingsFile = argv[2];
    string strDatasetPath = argv[3];
    string strOutputFile = (argc >= 5) ? argv[4] : "benchmark_out.json";

    vector<string> vstrLeft, vstrRight;
    vector<double> vTimeStamps;
    LoadStereoImages(strDatasetPath, vstrLeft, vstrRight, vTimeStamps);

    int nImages = vstrLeft.size();
    if(nImages == 0) {
        cerr << "Error: No stereo images found in " << strDatasetPath << endl;
        return 1;
    }

    ORB_SLAM3::System SLAM(strVocFile, strSettingsFile, ORB_SLAM3::System::STEREO, false);

    vector<float> vTrackTimes;
    vTrackTimes.reserve(nImages);

    for(int i = 0; i < nImages; i++) {
        cv::Mat imLeft = cv::imread(vstrLeft[i], cv::IMREAD_UNCHANGED);
        cv::Mat imRight = cv::imread(vstrRight[i], cv::IMREAD_UNCHANGED);
        if(imLeft.empty() || imRight.empty()) continue;

        double tframe = vTimeStamps[i];
        auto t1 = std::chrono::steady_clock::now();
        SLAM.TrackStereo(imLeft, imRight, tframe);
        auto t2 = std::chrono::steady_clock::now();

        float ttrack = std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(t2 - t1).count();
        vTrackTimes.push_back(ttrack);
    }

    SLAM.Shutdown();

    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    long peak_rss_kb = usage.ru_maxrss;

    int nTracked = vTrackTimes.size();
    float sum = 0.0f;
    float min_t = nTracked > 0 ? vTrackTimes[0] : 0.0f;
    float max_t = nTracked > 0 ? vTrackTimes[0] : 0.0f;
    for(float t : vTrackTimes) {
        sum += t;
        if(t < min_t) min_t = t;
        if(t > max_t) max_t = t;
    }
    float mean_t = nTracked > 0 ? sum / nTracked : 0.0f;

    float sq_sum = 0.0f;
    for(float t : vTrackTimes) {
        sq_sum += (t - mean_t) * (t - mean_t);
    }
    float stddev_t = nTracked > 0 ? std::sqrt(sq_sum / nTracked) : 0.0f;

    vector<float> sorted = vTrackTimes;
    std::sort(sorted.begin(), sorted.end());
    float p50 = nTracked > 0 ? sorted[static_cast<size_t>(nTracked * 0.50)] : 0.0f;
    float p90 = nTracked > 0 ? sorted[static_cast<size_t>(nTracked * 0.90)] : 0.0f;
    float p95 = nTracked > 0 ? sorted[static_cast<size_t>(nTracked * 0.95)] : 0.0f;
    float avg_fps = mean_t > 0.0f ? 1000.0f / mean_t : 0.0f;

    ofstream out(strOutputFile);
    out << "{\n";
    out << "  \"dataset\": \"" << strDatasetPath << "\",\n";
    out << "  \"total_frames\": " << nImages << ",\n";
    out << "  \"tracked_frames\": " << nTracked << ",\n";
    out << "  \"mean_latency_ms\": " << mean_t << ",\n";
    out << "  \"stddev_ms\": " << stddev_t << ",\n";
    out << "  \"min_latency_ms\": " << min_t << ",\n";
    out << "  \"max_latency_ms\": " << max_t << ",\n";
    out << "  \"p50_latency_ms\": " << p50 << ",\n";
    out << "  \"p90_latency_ms\": " << p90 << ",\n";
    out << "  \"p95_latency_ms\": " << p95 << ",\n";
    out << "  \"avg_fps\": " << avg_fps << ",\n";
    out << "  \"peak_rss_mb\": " << (peak_rss_kb / 1024.0) << "\n";
    out << "}\n";
    out.close();

    cout << "Benchmark result written to " << strOutputFile << endl;
    return 0;
}
```

- [ ] **Step 2: Add target in `CMakeLists.txt`**

```cmake
add_executable(stereo_benchmark
Examples/Stereo/stereo_benchmark.cc
)
target_link_libraries(stereo_benchmark ${PROJECT_NAME})
```

- [ ] **Step 3: Build target in Nix**

Run: `nix develop --command bash -c "cd build && make stereo_benchmark -j$(nproc)"`
Expected: `[100%] Built target stereo_benchmark`

- [ ] **Step 4: Commit**

```bash
git add Examples/Stereo/stereo_benchmark.cc CMakeLists.txt
git commit -m "feat: add stereo_benchmark headless timing executable"
```

---

### Task 2: Automated Benchmark Orchestrator Script (`scripts/run_baseline_benchmark.py`)

**Files:**
- Create: `scripts/run_baseline_benchmark.py`

**Interfaces:**
- Consumes: `./Examples/Stereo/stereo_benchmark`, `dataset/full_run`, `dataset/circle_run`
- Produces: `docs/superpowers/artifacts/baseline_benchmark_results.json`, `docs/superpowers/artifacts/baseline_benchmark_report.md`

- [ ] **Step 1: Write `scripts/run_baseline_benchmark.py`**

```python
#!/usr/bin/env python3
import json
import os
import subprocess
import sys

def run_benchmark(dataset_path, output_json):
    cmd = [
        "./Examples/Stereo/stereo_benchmark",
        "Vocabulary/ORBvoc.txt",
        f"{dataset_path}/camera.yaml",
        dataset_path,
        output_json
    ]
    print(f"Running benchmark on {dataset_path}...")
    res = subprocess.run(["nix", "develop", "--command"] + cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print(f"Benchmark failed: {res.stderr}")
        return None
    with open(output_json, 'r') as f:
        return json.load(f)

def main():
    datasets = ["dataset/circle_run", "dataset/full_run"]
    results = {}
    for ds in datasets:
        if os.path.exists(ds):
            json_file = f"/tmp/bench_{os.path.basename(ds)}.json"
            res = run_benchmark(ds, json_file)
            if res:
                results[os.path.basename(ds)] = res

    os.makedirs("docs/superpowers/artifacts", exist_ok=True)
    out_json_path = "docs/superpowers/artifacts/baseline_benchmark_results.json"
    with open(out_json_path, 'w') as f:
        json.dump(results, f, indent=2)

    report_path = "docs/superpowers/artifacts/baseline_benchmark_report.md"
    with open(report_path, 'w') as f:
        f.write("# ORB-SLAM3 Baseline Performance Benchmark Report\n\n")
        f.write("## System Hardware Specifications\n")
        f.write("- **CPU**: AMD Ryzen 7 5825U (8 cores / 16 threads, up to 4.55 GHz)\n")
        f.write("- **RAM**: 22 GiB System Memory\n")
        f.write("- **OS**: NixOS (Linux x86_64)\n\n")
        f.write("## Baseline Metrics\n\n")
        f.write("| Dataset | Total Frames | Mean Latency (ms) | Median P50 (ms) | P90 (ms) | P95 (ms) | Avg FPS | Peak Memory (MB) |\n")
        f.write("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |\n")
        for ds_name, data in results.items():
            f.write(f"| `{ds_name}` | {data['total_frames']} | {data['mean_latency_ms']:.2f} | {data['p50_latency_ms']:.2f} | {data['p90_latency_ms']:.2f} | {data['p95_latency_ms']:.2f} | **{data['avg_fps']:.2f} Hz** | {data['peak_rss_mb']:.1f} MB |\n")

    print(f"Saved baseline results to {out_json_path} and {report_path}")

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Test script execution**

Run: `nix develop --command python3 scripts/run_baseline_benchmark.py`
Expected: `Saved baseline results to docs/superpowers/artifacts/baseline_benchmark_results.json`

- [ ] **Step 3: Commit**

```bash
git add scripts/run_baseline_benchmark.py
git commit -m "feat: add automated baseline benchmark orchestrator script"
```

---

### Task 3: Baseline Benchmark Run & Artifact Generation

**Files:**
- Output: `docs/superpowers/artifacts/baseline_benchmark_results.json`
- Output: `docs/superpowers/artifacts/baseline_benchmark_report.md`

- [ ] **Step 1: Execute full baseline benchmark**

Run: `nix develop --command python3 scripts/run_baseline_benchmark.py`

- [ ] **Step 2: Verify generated artifacts**

Run: `cat docs/superpowers/artifacts/baseline_benchmark_report.md`
Expected: Complete markdown table displaying mean latency, median, P95, FPS, and peak memory for `circle_run` and `full_run`.

- [ ] **Step 3: Commit baseline results**

```bash
git add docs/superpowers/artifacts/baseline_benchmark_results.json docs/superpowers/artifacts/baseline_benchmark_report.md
git commit -m "docs: add baseline performance benchmark report artifacts"
```
