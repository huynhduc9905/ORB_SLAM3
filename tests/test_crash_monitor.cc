#include <gtest/gtest.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "CrashMonitor.h"

namespace
{

using ORB_SLAM3::CrashMonitor;

std::string MakeTempDir()
{
    char tmpl[] = "/tmp/orbslam3_crashmon_XXXXXX";
    char *dir = mkdtemp(tmpl);
    if (!dir)
        throw std::runtime_error("mkdtemp failed");
    return std::string(dir);
}

std::vector<std::string> ListFiles(const std::string &dir)
{
    std::vector<std::string> out;
    std::string cmd = "ls -1 " + dir + " 2>/dev/null";
    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
        return out;
    char buf[512];
    while (fgets(buf, sizeof(buf), pipe))
    {
        std::string line(buf);
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
            line.pop_back();
        if (!line.empty())
            out.push_back(line);
    }
    pclose(pipe);
    return out;
}

std::string ReadFile(const std::string &path)
{
    std::ifstream in(path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

class CrashMonitorTest : public ::testing::Test
{
protected:
    void SetUp() override { mDir = MakeTempDir(); }

    void TearDown() override
    {
        CrashMonitor::Uninstall();
        if (!mDir.empty())
            (void)system(("rm -rf " + mDir).c_str());
    }

    std::string mDir;
};

TEST_F(CrashMonitorTest, InstallCreatesReportDirectoryAndIsIdempotent)
{
    const std::string nested = mDir + "/reports/run";
    ASSERT_TRUE(CrashMonitor::Install(nested, "unit"));
    EXPECT_TRUE(CrashMonitor::IsInstalled());

    struct stat st{};
    ASSERT_EQ(stat(nested.c_str(), &st), 0) << "report directory must be created";
    EXPECT_TRUE(S_ISDIR(st.st_mode));

    // Second install must not fail or change the report path.
    const std::string first_path = CrashMonitor::ReportPath();
    EXPECT_TRUE(CrashMonitor::Install(nested, "other"));
    EXPECT_EQ(CrashMonitor::ReportPath(), first_path);
}

TEST_F(CrashMonitorTest, ReportPathContainsSanitizedRunLabel)
{
    ASSERT_TRUE(CrashMonitor::Install(mDir, "stress/9 instance"));
    const std::string path = CrashMonitor::ReportPath();
    EXPECT_NE(path.find("stress_9_instance"), std::string::npos)
        << "unsafe characters must be sanitized, got: " << path;
    EXPECT_EQ(path.find('/', mDir.size() + 1), std::string::npos)
        << "label must not introduce extra path separators: " << path;
}

TEST_F(CrashMonitorTest, HandledSignalsCoverFatalFaults)
{
    const std::vector<int> sigs = CrashMonitor::HandledSignals();
    for (int expected : {SIGSEGV, SIGABRT, SIGFPE, SIGBUS, SIGILL})
    {
        EXPECT_NE(std::find(sigs.begin(), sigs.end(), expected), sigs.end())
            << "missing handler for " << expected;
    }
}

TEST_F(CrashMonitorTest, SignalNameMapsKnownSignals)
{
    EXPECT_EQ(CrashMonitor::SignalName(SIGSEGV), "SIGSEGV");
    EXPECT_EQ(CrashMonitor::SignalName(SIGABRT), "SIGABRT");
    EXPECT_EQ(CrashMonitor::SignalName(SIGFPE), "SIGFPE");
    EXPECT_EQ(CrashMonitor::SignalName(12345), "UNKNOWN");
}

TEST_F(CrashMonitorTest, FormatReportIncludesSlamContext)
{
    ASSERT_TRUE(CrashMonitor::Install(mDir, "ctx"));

    CrashMonitor::Context().frame_id.store(4321);
    CrashMonitor::Context().tracking_state.store(2);
    CrashMonitor::Context().keyframes_in_map.store(77);
    CrashMonitor::Context().map_points_in_map.store(8888);
    CrashMonitor::Context().maps_in_atlas.store(3);

    const std::string report = CrashMonitor::FormatReport(SIGSEGV, "SIGSEGV");

    EXPECT_NE(report.find("SIGSEGV"), std::string::npos);
    EXPECT_NE(report.find("4321"), std::string::npos) << "frame id missing";
    EXPECT_NE(report.find("77"), std::string::npos) << "keyframe count missing";
    EXPECT_NE(report.find("8888"), std::string::npos) << "map point count missing";
    EXPECT_NE(report.find("3"), std::string::npos) << "atlas map count missing";
}

TEST_F(CrashMonitorTest, WriteSyntheticReportProducesReadableFile)
{
    ASSERT_TRUE(CrashMonitor::Install(mDir, "synth"));
    CrashMonitor::Context().frame_id.store(999);

    ASSERT_TRUE(CrashMonitor::WriteSyntheticReport("watchdog timeout"));

    const std::vector<std::string> files = ListFiles(mDir);
    ASSERT_EQ(files.size(), 1u) << "expected exactly one report file";

    // The stall report must live at a distinct path so the SIGABRT handler
    // cannot truncate away the stall diagnosis.
    EXPECT_NE(files[0].find(".stall"), std::string::npos)
        << "synthetic report should use the .stall suffix, got: " << files[0];

    const std::string body = ReadFile(mDir + "/" + files[0]);
    EXPECT_NE(body.find("watchdog timeout"), std::string::npos);
    EXPECT_NE(body.find("999"), std::string::npos);
    EXPECT_NE(body.find("Backtrace"), std::string::npos);
}

TEST_F(CrashMonitorTest, WatchdogNotRunningByDefault)
{
    ASSERT_TRUE(CrashMonitor::Install(mDir, "wd_default"));
    EXPECT_FALSE(CrashMonitor::IsWatchdogRunning());
}

TEST_F(CrashMonitorTest, WatchdogStartAndStopAreIdempotent)
{
    ASSERT_TRUE(CrashMonitor::Install(mDir, "wd_lifecycle"));

    EXPECT_TRUE(CrashMonitor::StartWatchdog(/*stall_timeout_ms=*/60000, /*poll_interval_ms=*/10));
    EXPECT_TRUE(CrashMonitor::IsWatchdogRunning());

    // Starting again while already running must not spawn a second thread or
    // otherwise misbehave.
    EXPECT_TRUE(CrashMonitor::StartWatchdog(60000, 10));
    EXPECT_TRUE(CrashMonitor::IsWatchdogRunning());

    CrashMonitor::StopWatchdog();
    EXPECT_FALSE(CrashMonitor::IsWatchdogRunning());

    // Stopping again when not running must not hang or crash.
    CrashMonitor::StopWatchdog();
    EXPECT_FALSE(CrashMonitor::IsWatchdogRunning());
}

TEST_F(CrashMonitorTest, WatchdogToleratesActiveProgress)
{
    ASSERT_TRUE(CrashMonitor::Install(mDir, "wd_progress"));
    CrashMonitor::Context().frame_id.store(1);

    ASSERT_TRUE(CrashMonitor::StartWatchdog(/*stall_timeout_ms=*/150, /*poll_interval_ms=*/10));

    // Keep advancing frame_id faster than the stall timeout. If the watchdog
    // fired, it would have aborted this process by now.
    for (int i = 2; i < 40; ++i)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        CrashMonitor::Context().frame_id.store(i);
    }

    EXPECT_TRUE(CrashMonitor::IsWatchdogRunning()) << "process should still be alive and watchdog active";
    CrashMonitor::StopWatchdog();

    EXPECT_TRUE(ListFiles(mDir).empty()) << "no report should have been written for active progress";
}

// The behaviour that actually matters: a genuinely stalled frame_id must
// terminate the process via abort() and leave a synthetic report behind.
// Runs in a forked child so the abort() is contained to that child.
TEST_F(CrashMonitorTest, StalledFrameIdTriggersAbortAndSyntheticReport)
{
    pid_t pid = fork();
    ASSERT_NE(pid, -1) << "fork failed";

    if (pid == 0)
    {
        CrashMonitor::Install(mDir, "stalled_child");
        CrashMonitor::Context().frame_id.store(42);
        CrashMonitor::StartWatchdog(/*stall_timeout_ms=*/100, /*poll_interval_ms=*/10);

        // Never advance frame_id again; just wait to be aborted.
        std::this_thread::sleep_for(std::chrono::seconds(5));
        _exit(0); // unreachable if the watchdog works
    }

    int status = 0;
    ASSERT_EQ(waitpid(pid, &status, 0), pid);

    EXPECT_TRUE(WIFSIGNALED(status)) << "child should be terminated by the watchdog's abort()";
    if (WIFSIGNALED(status))
        EXPECT_EQ(WTERMSIG(status), SIGABRT);

    const std::vector<std::string> files = ListFiles(mDir);
    ASSERT_GE(files.size(), 1u) << "watchdog must produce a report file";

    // Both reports should exist and be distinguishable: the .stall report
    // carries the stall reason, the plain one carries the SIGABRT backtrace.
    std::string stall_body, signal_body;
    for (const std::string &f : files)
    {
        if (f.find(".stall") != std::string::npos)
            stall_body = ReadFile(mDir + "/" + f);
        else
            signal_body = ReadFile(mDir + "/" + f);
    }

    ASSERT_FALSE(stall_body.empty()) << "a .stall report must be written before aborting";
    EXPECT_NE(stall_body.find("watchdog"), std::string::npos) << "stall reason missing";
    EXPECT_NE(stall_body.find("42"), std::string::npos) << "last known frame id missing";

    // The SIGABRT report is written by the handler and must not have clobbered
    // the stall report.
    if (!signal_body.empty())
        EXPECT_NE(signal_body.find("SIGABRT"), std::string::npos);
}

// The behaviour that actually matters: a real fatal signal in a real process
// must leave a report on disk. Runs in a forked child so the crash is contained.
TEST_F(CrashMonitorTest, RealSegfaultInChildWritesReportAndPreservesExitStatus)
{
    pid_t pid = fork();
    ASSERT_NE(pid, -1) << "fork failed";

    if (pid == 0)
    {
        // Child: install, publish context, then genuinely crash.
        CrashMonitor::Install(mDir, "child");
        CrashMonitor::Context().frame_id.store(1234);
        CrashMonitor::Context().tracking_state.store(2);

        volatile int *p = nullptr;
        *p = 42;   // SIGSEGV
        _exit(0);  // unreachable
    }

    int status = 0;
    ASSERT_EQ(waitpid(pid, &status, 0), pid);

    // The handler must re-raise so the process still dies from the signal:
    // supervisors and core-dump collection depend on the real exit status.
    EXPECT_TRUE(WIFSIGNALED(status)) << "child should terminate via signal";
    if (WIFSIGNALED(status))
        EXPECT_EQ(WTERMSIG(status), SIGSEGV);

    const std::vector<std::string> files = ListFiles(mDir);
    ASSERT_GE(files.size(), 1u) << "crash must produce a report file";

    const std::string body = ReadFile(mDir + "/" + files[0]);
    EXPECT_NE(body.find("SIGSEGV"), std::string::npos);
    EXPECT_NE(body.find("1234"), std::string::npos) << "SLAM context missing from report";
    EXPECT_NE(body.find("Backtrace"), std::string::npos);
}

} // namespace
