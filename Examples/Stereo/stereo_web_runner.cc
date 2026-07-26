#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "System.h"

using namespace std;

void LoadStereoImages(const string &strDatasetPath, vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    string strTimesPath = strDatasetPath + "/times.txt";
    ifstream fTimes(strTimesPath.c_str());
    if(!fTimes.is_open())
    {
        cerr << "Failed to open timestamps file at " << strTimesPath << endl;
        return;
    }

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if(!s.empty())
        {
            stringstream ss(s);
            double t;
            ss >> t;
            vTimeStamps.push_back(t);

            stringstream ss_filename;
            ss_filename << fixed << setprecision(6) << t << ".png";
            string filename = ss_filename.str();

            string leftPath = strDatasetPath + "/cam0/data/" + filename;
            string rightPath = strDatasetPath + "/cam1/data/" + filename;

            vstrImageLeft.push_back(leftPath);
            vstrImageRight.push_back(rightPath);
        }
    }
}

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./stereo_web_runner path_to_vocabulary path_to_settings path_to_dataset" << endl;
        return 1;
    }

    string strVocFile = argv[1];
    string strSettingsFile = argv[2];
    string strDatasetPath = argv[3];

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamps;
    LoadStereoImages(strDatasetPath, vstrImageLeft, vstrImageRight, vTimeStamps);

    int nImages = vstrImageLeft.size();
    if(nImages == 0)
    {
        cerr << "Failed to load images from " << strDatasetPath << endl;
        return 1;
    }
    cout << "Loaded " << nImages << " stereo IR image pairs." << endl;

    // Create SLAM system in STEREO mode
    ORB_SLAM3::System SLAM(strVocFile, strSettingsFile, ORB_SLAM3::System::STEREO, false);

    cout << endl << "------- STARTING STEREO SLAM TRACKING LOOP -------" << endl;

    vector<float> vTrackTimes;
    vTrackTimes.reserve(nImages);

    for(int ni = 0; ni < nImages; ni++)
    {
        cv::Mat imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        cv::Mat imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);

        double tframe = vTimeStamps[ni];

        if(imLeft.empty() || imRight.empty())
        {
            cerr << "Failed to load image at index " << ni << endl;
            continue;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        SLAM.TrackStereo(imLeft, imRight, tframe);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        float ttrack = std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(t2 - t1).count();
        vTrackTimes.push_back(ttrack);

        if(ni % 200 == 0)
        {
            cout << "Processed " << ni << "/" << nImages << " stereo frames (t = " << fixed << setprecision(6) << tframe << "s, track_time = " << setprecision(2) << ttrack << "ms)" << endl;
        }
    }

    // Compute statistical metrics
    int nTracked = vTrackTimes.size();
    if (nTracked > 0) {
        float sum = 0.0f;
        float min_time = vTrackTimes[0];
        float max_time = vTrackTimes[0];
        for (float t : vTrackTimes) {
            sum += t;
            if (t < min_time) min_time = t;
            if (t > max_time) max_time = t;
        }
        float mean_time = sum / nTracked;

        float sq_sum = 0.0f;
        for (float t : vTrackTimes) {
            sq_sum += (t - mean_time) * (t - mean_time);
        }
        float stddev_time = std::sqrt(sq_sum / nTracked);

        vector<float> sorted_times = vTrackTimes;
        std::sort(sorted_times.begin(), sorted_times.end());
        float p50 = sorted_times[static_cast<size_t>(nTracked * 0.50)];
        float p90 = sorted_times[static_cast<size_t>(nTracked * 0.90)];
        float p95 = sorted_times[static_cast<size_t>(nTracked * 0.95)];

        float avg_fps = 1000.0f / mean_time;
        float median_fps = 1000.0f / p50;

        cout << endl << "==================================================" << endl;
        cout << "  ORB-SLAM3 STEREO TRACKING BENCHMARK REPORT" << endl;
        cout << "==================================================" << endl;
        cout << "Dataset: " << strDatasetPath << endl;
        cout << "Total Frames Processed: " << nTracked << endl;
        cout << "Average Tracking Time: " << fixed << setprecision(2) << mean_time << " ms" << endl;
        cout << "Std Dev:               " << setprecision(2) << stddev_time << " ms" << endl;
        cout << "Min Tracking Time:     " << setprecision(2) << min_time << " ms" << endl;
        cout << "Max Tracking Time:     " << setprecision(2) << max_time << " ms" << endl;
        cout << "P50 (Median) Latency:  " << setprecision(2) << p50 << " ms (" << setprecision(2) << median_fps << " Hz)" << endl;
        cout << "P90 Latency:           " << setprecision(2) << p90 << " ms" << endl;
        cout << "P95 Latency:           " << setprecision(2) << p95 << " ms" << endl;
        cout << "--------------------------------------------------" << endl;
        cout << "AVG ORB TRACKING FREQUENCY: " << setprecision(2) << avg_fps << " Hz (FPS)" << endl;
        cout << "==================================================" << endl;

        // Write report file
        std::ofstream report("docs/superpowers/artifacts/full_run_tracking_performance.md");
        if (report.is_open()) {
            report << "# ORB-SLAM3 Tracking Frequency Report: Full Run Dataset\n\n";
            report << "| Metric | Value |\n";
            report << "| :--- | :--- |\n";
            report << "| **Dataset** | `" << strDatasetPath << "` |\n";
            report << "| **Total Frames** | " << nTracked << " |\n";
            report << "| **Average Tracking Latency** | **" << fixed << setprecision(2) << mean_time << " ms** |\n";
            report << "| **Average ORB Tracking Frequency** | **" << setprecision(2) << avg_fps << " Hz (FPS)** |\n";
            report << "| **Median (P50) Latency** | " << setprecision(2) << p50 << " ms (" << setprecision(2) << median_fps << " Hz) |\n";
            report << "| **Std Deviation** | " << setprecision(2) << stddev_time << " ms |\n";
            report << "| **Min Latency** | " << setprecision(2) << min_time << " ms |\n";
            report << "| **Max Latency** | " << setprecision(2) << max_time << " ms |\n";
            report << "| **P90 Latency** | " << setprecision(2) << p90 << " ms |\n";
            report << "| **P95 Latency** | " << setprecision(2) << p95 << " ms |\n";
            report.close();
        }
    }

    cout << "Sequence finished processing. WebViewer server remains active for visualization..." << endl;
    cout << "Press Ctrl+C or kill process to terminate." << endl;
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    SLAM.Shutdown();
    return 0;
}
