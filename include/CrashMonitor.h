/**
* This file is part of ORB-SLAM3
*
* Crash monitor: captures diagnostic reports when a fatal signal terminates the
* process, so long-running and stress-test runs leave behind something
* actionable instead of only an exit code.
*/

#ifndef CRASHMONITOR_H
#define CRASHMONITOR_H

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace ORB_SLAM3
{

/**
 * Context describing where the SLAM pipeline was when a crash happened.
 *
 * Fields are updated from the tracking thread on a hot path, so every member is
 * a lock-free atomic. A signal handler may only call async-signal-safe
 * functions, which excludes locking a mutex, so this type deliberately avoids
 * any locking.
 */
struct CrashContext
{
    std::atomic<long long> frame_id{-1};
    std::atomic<int> tracking_state{-1};
    std::atomic<int> keyframes_in_map{-1};
    std::atomic<int> map_points_in_map{-1};
    std::atomic<int> maps_in_atlas{-1};
    std::atomic<long long> timestamp_ns{0};
};

/**
 * Installs handlers for fatal signals and writes a crash report describing the
 * signal, a native backtrace, and the last known SLAM context.
 *
 * The monitor is process-wide. Install() is idempotent: repeated calls keep the
 * first installation and return true.
 */
class CrashMonitor
{
public:
    /**
     * Install signal handlers.
     *
     * @param report_directory directory for crash reports; created if missing.
     * @param run_label short identifier included in the report filename, used
     *        to tell concurrent stress-test instances apart. Non-alphanumeric
     *        characters are replaced with '_'.
     * @return true if handlers are installed (or were already installed).
     */
    static bool Install(const std::string &report_directory, const std::string &run_label);

    /** Restore default handlers. Safe to call when not installed. */
    static void Uninstall();

    static bool IsInstalled();

    /** Mutable SLAM context published by the tracking thread. */
    static CrashContext &Context();

    /**
     * Path of the report that would be written for the next crash. Computed at
     * Install() time because building a path inside a signal handler is not
     * async-signal-safe.
     */
    static std::string ReportPath();

    /**
     * Render a crash report body. Exposed for testing: it is the same routine
     * the handler uses, minus the signal-unsafe formatting.
     *
     * @param signal_number signal that fired, or 0 for a synthetic report.
     * @param signal_name human-readable signal name.
     */
    static std::string FormatReport(int signal_number, const std::string &signal_name);

    /** Signals the monitor installs handlers for. */
    static std::vector<int> HandledSignals();

    /** Human-readable name for a handled signal; "UNKNOWN" if unhandled. */
    static std::string SignalName(int signal_number);

    /**
     * Write a report immediately without a signal having fired. Used by the
     * watchdog path and by tests.
     *
     * @return true if the report was written.
     */
    static bool WriteSyntheticReport(const std::string &reason);

    /**
     * Start a background watchdog that periodically checks whether
     * Context().frame_id has advanced. If it has not changed for
     * stall_timeout_ms, the watchdog writes a synthetic report describing the
     * stall and aborts the process.
     *
     * This exists because a deadlock produces no signal: the process is alive
     * and idle forever, so the fatal-signal handlers above never fire and a
     * hung run silently occupies a stress-test slot until an external
     * timeout kills it with no diagnostic information. abort() converts a
     * silent hang into a SIGABRT that the installed handler enriches with a
     * backtrace of whichever thread happens to be running when abort() is
     * delivered -- typically not the stuck thread, but the stall reason and
     * last-known SLAM context are still captured in the synthetic report
     * written just before aborting.
     *
     * No-op if a watchdog is already running or the monitor is not installed.
     *
     * @param stall_timeout_ms abort if frame_id has not changed for this long.
     * @param poll_interval_ms how often to check; must be < stall_timeout_ms.
     * @return true if the watchdog thread was started.
     */
    static bool StartWatchdog(int stall_timeout_ms, int poll_interval_ms);

    /** Stop the watchdog thread if running. Safe to call when not running. */
    static void StopWatchdog();

    static bool IsWatchdogRunning();

private:
    CrashMonitor() = delete;
};

} // namespace ORB_SLAM3

#endif // CRASHMONITOR_H
