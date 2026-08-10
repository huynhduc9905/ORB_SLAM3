#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include <CrashMonitor.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss(s);
            double t;
            ss >> t;
            vTimestamps.push_back(t);

            char buf[64];
            snprintf(buf, sizeof(buf), "cam0/data/%.6f.png", t);
            vstrImageFilenames.push_back(string(buf));
        }
    }
}

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_web_runner path_to_vocabulary path_to_settings path_to_dataset [max_frames]" << endl;
        return 1;
    }

    // Capture diagnostics if a fatal signal terminates this run.
    if(!ORB_SLAM3::CrashMonitor::Install("/data/orbslam3_artifacts/crash_reports", "mono_web_runner"))
        cerr << "WARNING: could not install crash monitor" << endl;
    else if(!ORB_SLAM3::CrashMonitor::StartWatchdog(/*stall_timeout_ms=*/90000, /*poll_interval_ms=*/1000))
        cerr << "WARNING: could not start crash monitor watchdog" << endl;

    int max_frames = (argc >= 5) ? atoi(argv[4]) : 100000;

    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3]) + "/cam0/times.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = min(static_cast<int>(vstrImageFilenames.size()), max_frames);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence for Web Visualizer test..." << endl;
    cout << "Images to process: " << nImages << endl << endl;

    // Create SLAM system with WebViewer active
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);

    cv::Mat im;
    for(int ni = 0; ni < nImages; ni++)
    {
        string imgPath = string(argv[3]) + "/" + vstrImageFilenames[ni];
        im = cv::imread(imgPath, cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << imgPath << endl;
            continue;
        }

        SLAM.TrackMonocular(im, tframe);

        // Sleep to simulate ~30 FPS playback rate
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // Sequence complete. Stop the watchdog before Shutdown(): joining the
    // detached global-bundle-adjustment thread during teardown is a bounded,
    // expected wait during which frame_id no longer advances, and would
    // otherwise trip the watchdog as a false positive.
    ORB_SLAM3::CrashMonitor::StopWatchdog();

    // Keep server running for inspection if desired
    cout << "Dataset execution complete. Shutting down SLAM." << endl;
    SLAM.Shutdown();

    return 0;
}
