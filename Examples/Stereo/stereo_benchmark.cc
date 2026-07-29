#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <numeric>
#include <cmath>
#include <sstream>
#include <csignal>
#include <execinfo.h>
#include <sys/resource.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <System.h>

using namespace std;

// Signal handler for crash capture
void BenchmarkCrashHandler(int sig) {
    void* callstack[128];
    int frames = backtrace(callstack, 128);
    char** strs = backtrace_symbols(callstack, frames);

    cerr << "\n==================================================" << endl;
    cerr << "  CRASH DETECTED IN BENCHMARK RUNNER (Signal " << sig << ")" << endl;
    cerr << "==================================================" << endl;

    ofstream crash_file("docs/superpowers/artifacts/benchmark_crash_dump.log", ios::app);
    if(crash_file.is_open()) {
        crash_file << "CRASH SIGNAL: " << sig << "\n";
        crash_file << "STACK TRACE:\n";
        for(int i = 0; i < frames; ++i) {
            cerr << strs[i] << endl;
            crash_file << strs[i] << "\n";
        }
        crash_file.close();
    }
    free(strs);
    exit(sig);
}

void RegisterCrashHandlers() {
    signal(SIGSEGV, BenchmarkCrashHandler);
    signal(SIGABRT, BenchmarkCrashHandler);
    signal(SIGFPE, BenchmarkCrashHandler);
    signal(SIGILL, BenchmarkCrashHandler);
    signal(SIGBUS, BenchmarkCrashHandler);
}

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

            stringstream ss_filename;
            ss_filename << fixed << setprecision(6) << t << ".png";
            string filename = ss_filename.str();

            vstrImageLeft.push_back(strPathToSequence + "/cam0/data/" + filename);
            vstrImageRight.push_back(strPathToSequence + "/cam1/data/" + filename);
        }
    }
}

int main(int argc, char **argv) {
    cv::setNumThreads(1);
    RegisterCrashHandlers();

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

    cout << "Loaded " << nImages << " stereo frame pairs from " << strDatasetPath << endl;
    ORB_SLAM3::System SLAM(strVocFile, strSettingsFile, ORB_SLAM3::System::STEREO, false);

    vector<float> vTrackTimes;
    vTrackTimes.reserve(nImages);

    auto total_start = std::chrono::steady_clock::now();

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

        if(true) {
            cout << "Processed " << i << "/" << nImages << " frames (latency: " << fixed << setprecision(2) << ttrack << " ms)" << endl;
        }
    }

    auto total_end = std::chrono::steady_clock::now();
    float total_duration_sec = std::chrono::duration_cast<std::chrono::duration<float>>(total_end - total_start).count();

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

#ifdef REGISTER_TIMES
    auto tracker = SLAM.GetTracker();
    auto compute_mean = [](const vector<double>& v) {
        if(v.empty()) return 0.0;
        double sum = 0.0;
        for(double x : v) sum += x;
        return sum / v.size();
    };
    double mean_orb_ext = compute_mean(tracker->vdORBExtract_ms);
    double mean_stereo_match = compute_mean(tracker->vdStereoMatch_ms);
    double mean_pose_pred = compute_mean(tracker->vdPosePred_ms);
    double mean_lm_track = compute_mean(tracker->vdLMTrack_ms);
#endif

    ofstream out(strOutputFile);
    out << "{\n";
    out << "  \"dataset\": \"" << strDatasetPath << "\",\n";
    out << "  \"total_frames\": " << nImages << ",\n";
    out << "  \"tracked_frames\": " << nTracked << ",\n";
    out << "  \"total_duration_sec\": " << fixed << setprecision(2) << total_duration_sec << ",\n";
    out << "  \"mean_latency_ms\": " << fixed << setprecision(2) << mean_t << ",\n";
    out << "  \"stddev_ms\": " << setprecision(2) << stddev_t << ",\n";
    out << "  \"min_latency_ms\": " << setprecision(2) << min_t << ",\n";
    out << "  \"max_latency_ms\": " << setprecision(2) << max_t << ",\n";
    out << "  \"p50_latency_ms\": " << setprecision(2) << p50 << ",\n";
    out << "  \"p90_latency_ms\": " << setprecision(2) << p90 << ",\n";
    out << "  \"p95_latency_ms\": " << setprecision(2) << p95 << ",\n";
    out << "  \"avg_fps\": " << setprecision(2) << avg_fps << ",\n";
#ifdef REGISTER_TIMES
    out << "  \"stage_orb_extract_ms\": " << setprecision(2) << mean_orb_ext << ",\n";
    out << "  \"stage_stereo_match_ms\": " << setprecision(2) << mean_stereo_match << ",\n";
    out << "  \"stage_pose_pred_ms\": " << setprecision(2) << mean_pose_pred << ",\n";
    out << "  \"stage_local_map_track_ms\": " << setprecision(2) << mean_lm_track << ",\n";
#endif
    out << "  \"peak_rss_mb\": " << setprecision(1) << (peak_rss_kb / 1024.0) << "\n";
    out << "}\n";
    out.close();

    cout << "\n==================================================" << endl;
    cout << "  ORB-SLAM3 BASELINE BENCHMARK COMPLETE" << endl;
    cout << "==================================================" << endl;
    cout << "Dataset:              " << strDatasetPath << endl;
    cout << "Total Frames:         " << nImages << endl;
    cout << "Tracked Frames:       " << nTracked << endl;
    cout << "Total Elapsed Time:   " << fixed << setprecision(2) << total_duration_sec << " s" << endl;
    cout << "Mean Frame Latency:   " << setprecision(2) << mean_t << " ms" << endl;
    cout << "Std Deviation:        " << setprecision(2) << stddev_t << " ms" << endl;
    cout << "Median P50 Latency:   " << setprecision(2) << p50 << " ms (" << setprecision(2) << (1000.0f / p50) << " Hz)" << endl;
    cout << "P90 Latency:          " << setprecision(2) << p90 << " ms" << endl;
    cout << "P95 Latency:          " << setprecision(2) << p95 << " ms" << endl;
    cout << "AVG TRACKING FREQ:    " << setprecision(2) << avg_fps << " Hz (FPS)" << endl;
#ifdef REGISTER_TIMES
    cout << "--------------------------------------------------" << endl;
    cout << "  PER-FUNCTION STAGE TIMING BREAKDOWN" << endl;
    cout << "--------------------------------------------------" << endl;
    cout << "1. ORB Feature Extraction:   " << setprecision(2) << mean_orb_ext << " ms (" << setprecision(1) << (mean_orb_ext / mean_t * 100.0) << "%)" << endl;
    cout << "2. Epipolar Stereo Matching: " << setprecision(2) << mean_stereo_match << " ms (" << setprecision(1) << (mean_stereo_match / mean_t * 100.0) << "%)" << endl;
    cout << "3. Pose Motion Prediction:  " << setprecision(2) << mean_pose_pred << " ms (" << setprecision(1) << (mean_pose_pred / mean_t * 100.0) << "%)" << endl;
    cout << "4. Local Map Projection & BA: " << setprecision(2) << mean_lm_track << " ms (" << setprecision(1) << (mean_lm_track / mean_t * 100.0) << "%)" << endl;
#endif
    cout << "==================================================" << endl;
    cout << "Peak RSS Memory:      " << setprecision(1) << (peak_rss_kb / 1024.0) << " MB" << endl;
    cout << "Benchmark JSON saved to: " << strOutputFile << endl;
    exit(0);
    return 0;
}
