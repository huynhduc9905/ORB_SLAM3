/**
* This file is part of ORB-SLAM3
*
* Crash monitor implementation.
*
* Design notes on signal safety: once a fatal signal fires, the process is in an
* undefined state and only async-signal-safe functions may be called. In
* particular malloc, printf, std::string, and iostreams are all unsafe, because
* the signal may have interrupted them mid-update and their internal locks may
* be held. The handler therefore:
*   - writes with write(2) into a preallocated buffer, never allocating;
*   - uses a preformatted report path computed at Install() time;
*   - formats integers with a hand-rolled routine instead of snprintf;
*   - reads SLAM context only through lock-free atomics;
*   - re-raises the original signal so the process still dies from it,
*     preserving the exit status supervisors and core-dump collectors expect.
*
* backtrace() is not formally async-signal-safe (its first call may resolve
* symbols and allocate). We accept this narrow risk because a backtrace is the
* single most useful item in a crash report, and we prime it during Install() so
* the lazy-loading path is already warm when the handler runs.
*/

#include "CrashMonitor.h"

#include <execinfo.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>
#include <sstream>
#include <thread>
#include <algorithm>

namespace ORB_SLAM3
{

namespace
{

constexpr int kMaxBacktraceFrames = 64;
constexpr std::size_t kMaxPathLength = 1024;

std::atomic<bool> gInstalled{false};

// Preformatted, NUL-terminated report path. Built in Install() so the handler
// never has to construct a string.
char gReportPath[kMaxPathLength] = {0};

CrashContext gContext;

// Saved previous dispositions so Uninstall() can restore them.
struct sigaction gPreviousActions[NSIG];
bool gPreviousActionValid[NSIG] = {false};

// Watchdog state. Independent of the signal-handling state above: the
// watchdog runs on an ordinary background thread and may use mutexes,
// allocate, and format strings freely, since it never executes inside a
// signal handler.
std::atomic<bool> gWatchdogRunning{false};
std::atomic<bool> gWatchdogStopRequested{false};
std::thread gWatchdogThread;
std::mutex gWatchdogMutex;
std::condition_variable gWatchdogCv;

// Guards against a second fatal signal (e.g. SIGSEGV inside the handler)
// producing a recursive report.
volatile sig_atomic_t gHandlerEntered = 0;

const std::vector<int> &HandledSignalList()
{
    static const std::vector<int> kSignals = {SIGSEGV, SIGABRT, SIGFPE, SIGBUS, SIGILL};
    return kSignals;
}

/** Async-signal-safe write of a NUL-terminated string. */
void SafeWrite(int fd, const char *text)
{
    if (!text)
        return;
    std::size_t len = std::strlen(text);
    while (len > 0)
    {
        const ssize_t written = write(fd, text, len);
        if (written <= 0)
        {
            if (errno == EINTR)
                continue;
            return;
        }
        text += written;
        len -= static_cast<std::size_t>(written);
    }
}

/** Async-signal-safe signed integer rendering. Returns bytes written to buf. */
std::size_t SafeFormatLongLong(long long value, char *buf, std::size_t buf_size)
{
    if (buf_size < 2)
        return 0;

    std::size_t pos = 0;
    bool negative = value < 0;

    // Build digits in reverse. Use unsigned to handle LLONG_MIN safely.
    unsigned long long magnitude =
        negative ? (~static_cast<unsigned long long>(value) + 1ULL)
                 : static_cast<unsigned long long>(value);

    char digits[24];
    std::size_t digit_count = 0;
    if (magnitude == 0)
        digits[digit_count++] = '0';
    while (magnitude > 0 && digit_count < sizeof(digits))
    {
        digits[digit_count++] = static_cast<char>('0' + (magnitude % 10ULL));
        magnitude /= 10ULL;
    }

    if (negative && pos < buf_size - 1)
        buf[pos++] = '-';
    while (digit_count > 0 && pos < buf_size - 1)
        buf[pos++] = digits[--digit_count];

    buf[pos] = '\0';
    return pos;
}

void SafeWriteLongLong(int fd, long long value)
{
    char buf[24];
    SafeFormatLongLong(value, buf, sizeof(buf));
    SafeWrite(fd, buf);
}

void SafeWriteLabelledValue(int fd, const char *label, long long value)
{
    SafeWrite(fd, label);
    SafeWriteLongLong(fd, value);
    SafeWrite(fd, "\n");
}

const char *SignalNameRaw(int signal_number)
{
    switch (signal_number)
    {
        case SIGSEGV: return "SIGSEGV";
        case SIGABRT: return "SIGABRT";
        case SIGFPE:  return "SIGFPE";
        case SIGBUS:  return "SIGBUS";
        case SIGILL:  return "SIGILL";
        default:      return "UNKNOWN";
    }
}

/** Recursively create a directory path. Returns true on success. */
bool MakeDirectories(const std::string &path)
{
    if (path.empty())
        return false;

    std::string current;
    current.reserve(path.size());

    std::size_t index = 0;
    if (path[0] == '/')
    {
        current = "/";
        index = 1;
    }

    while (index <= path.size())
    {
        const std::size_t next = path.find('/', index);
        const std::string component =
            path.substr(index, next == std::string::npos ? std::string::npos : next - index);

        if (!component.empty())
        {
            if (!current.empty() && current.back() != '/')
                current += '/';
            current += component;

            if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
                return false;
        }

        if (next == std::string::npos)
            break;
        index = next + 1;
    }

    struct stat st{};
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

std::string SanitizeLabel(const std::string &label)
{
    std::string out;
    out.reserve(label.size());
    for (char c : label)
    {
        const bool allowed = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                             (c >= '0' && c <= '9') || c == '-' || c == '_';
        out += allowed ? c : '_';
    }
    if (out.empty())
        out = "run";
    return out;
}

long long NowNanoseconds()
{
    // Must stay async-signal-safe: this is reached from the fatal-signal handler
    // via WriteReportToFd. POSIX guarantees clock_gettime() is async-signal-safe;
    // std::chrono::system_clock::now() is not (implementation may take non-trivial
    // code paths). Use the raw syscall wrapper directly.
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0)
        return 0;
    return static_cast<long long>(ts.tv_sec) * 1000000000LL +
           static_cast<long long>(ts.tv_nsec);
}

/**
 * Write the full report to an already-open fd using only signal-safe calls.
 */
void WriteReportToFd(int fd, int signal_number, const char *signal_name)
{
    SafeWrite(fd, "=== ORB-SLAM3 crash report ===\n");
    SafeWrite(fd, "Signal: ");
    SafeWrite(fd, signal_name);
    SafeWrite(fd, "\n");
    SafeWriteLabelledValue(fd, "Signal number: ", signal_number);
    SafeWriteLabelledValue(fd, "PID: ", static_cast<long long>(getpid()));
    SafeWriteLabelledValue(fd, "Report time (ns since epoch): ", NowNanoseconds());

    SafeWrite(fd, "\n--- SLAM context (last published by tracking thread) ---\n");
    SafeWriteLabelledValue(fd, "Frame id: ", gContext.frame_id.load(std::memory_order_relaxed));
    SafeWriteLabelledValue(fd, "Tracking state: ", gContext.tracking_state.load(std::memory_order_relaxed));
    SafeWriteLabelledValue(fd, "KeyFrames in map: ", gContext.keyframes_in_map.load(std::memory_order_relaxed));
    SafeWriteLabelledValue(fd, "MapPoints in map: ", gContext.map_points_in_map.load(std::memory_order_relaxed));
    SafeWriteLabelledValue(fd, "Maps in atlas: ", gContext.maps_in_atlas.load(std::memory_order_relaxed));
    SafeWriteLabelledValue(fd, "Frame timestamp (ns): ", gContext.timestamp_ns.load(std::memory_order_relaxed));

    SafeWrite(fd, "\n--- Backtrace ---\n");
    void *frames[kMaxBacktraceFrames];
    const int frame_count = backtrace(frames, kMaxBacktraceFrames);
    if (frame_count > 0)
    {
        // backtrace_symbols_fd does not allocate, unlike backtrace_symbols.
        backtrace_symbols_fd(frames, frame_count, fd);
    }
    else
    {
        SafeWrite(fd, "(backtrace unavailable)\n");
    }

    SafeWrite(fd, "=== end of report ===\n");
}

void FatalSignalHandler(int signal_number)
{
    // If a second fatal signal arrives while reporting, die immediately rather
    // than recursing.
    if (gHandlerEntered)
        _exit(128 + signal_number);
    gHandlerEntered = 1;

    const char *signal_name = SignalNameRaw(signal_number);

    if (gReportPath[0] != '\0')
    {
        const int fd = open(gReportPath, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0)
        {
            WriteReportToFd(fd, signal_number, signal_name);
            fsync(fd);
            close(fd);
        }
    }

    // Also emit to stderr so supervisors capturing output see it.
    SafeWrite(STDERR_FILENO, "\nORB-SLAM3 CRASH: ");
    SafeWrite(STDERR_FILENO, signal_name);
    SafeWrite(STDERR_FILENO, " -- report: ");
    SafeWrite(STDERR_FILENO, gReportPath[0] ? gReportPath : "(none)");
    SafeWrite(STDERR_FILENO, "\n");

    // Restore the default action and re-raise so the process dies from the
    // original signal. This preserves WTERMSIG for the parent and allows core
    // dumps to be produced normally.
    //
    // The triggering signal is blocked for the duration of the handler, so
    // raise() alone would merely mark it pending and return, and the _exit()
    // below would then terminate us with an ordinary exit status. Unblock it
    // first so delivery is immediate and actually kills the process.
    struct sigaction dfl;
    std::memset(&dfl, 0, sizeof(dfl));
    dfl.sa_handler = SIG_DFL;
    sigemptyset(&dfl.sa_mask);
    sigaction(signal_number, &dfl, nullptr);

    sigset_t unblock_set;
    sigemptyset(&unblock_set);
    sigaddset(&unblock_set, signal_number);
    sigprocmask(SIG_UNBLOCK, &unblock_set, nullptr);

    raise(signal_number);

    // Only reached if the signal somehow still did not terminate us.
    _exit(128 + signal_number);
}

} // namespace

bool CrashMonitor::Install(const std::string &report_directory, const std::string &run_label)
{
    if (gInstalled.load())
        return true;

    if (!MakeDirectories(report_directory))
        return false;

    // Build the report path once, here, so the handler never formats a string.
    std::ostringstream path;
    path << report_directory;
    if (!report_directory.empty() && report_directory.back() != '/')
        path << '/';

    const std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm_buf);

    path << "crash-" << SanitizeLabel(run_label) << "-" << stamp << "-pid" << getpid() << ".log";

    const std::string path_str = path.str();
    if (path_str.size() >= kMaxPathLength)
        return false;
    std::memcpy(gReportPath, path_str.c_str(), path_str.size() + 1);

    // Prime backtrace() so its lazy symbol resolution happens now rather than
    // inside the signal handler.
    void *warmup[4];
    (void)backtrace(warmup, 4);

    struct sigaction action;
    std::memset(&action, 0, sizeof(action));
    action.sa_handler = &FatalSignalHandler;
    sigemptyset(&action.sa_mask);
    // Run the handler on an alternate consideration: SA_RESETHAND is not used
    // because the handler restores SIG_DFL itself before re-raising.
    action.sa_flags = SA_RESTART;

    for (int sig : HandledSignalList())
    {
        struct sigaction previous;
        if (sigaction(sig, &action, &previous) == 0)
        {
            gPreviousActions[sig] = previous;
            gPreviousActionValid[sig] = true;
        }
    }

    gHandlerEntered = 0;
    gInstalled.store(true);
    return true;
}

void CrashMonitor::Uninstall()
{
    if (!gInstalled.exchange(false))
        return;

    StopWatchdog();

    for (int sig : HandledSignalList())
    {
        if (gPreviousActionValid[sig])
        {
            sigaction(sig, &gPreviousActions[sig], nullptr);
            gPreviousActionValid[sig] = false;
        }
    }
    gReportPath[0] = '\0';
    gHandlerEntered = 0;
}

bool CrashMonitor::IsInstalled()
{
    return gInstalled.load();
}

CrashContext &CrashMonitor::Context()
{
    return gContext;
}

std::string CrashMonitor::ReportPath()
{
    return std::string(gReportPath);
}

std::vector<int> CrashMonitor::HandledSignals()
{
    return HandledSignalList();
}

std::string CrashMonitor::SignalName(int signal_number)
{
    return std::string(SignalNameRaw(signal_number));
}

std::string CrashMonitor::FormatReport(int signal_number, const std::string &signal_name)
{
    // Not called from a signal handler, so ordinary formatting is fine here.
    std::ostringstream out;
    out << "=== ORB-SLAM3 crash report ===\n"
        << "Signal: " << signal_name << "\n"
        << "Signal number: " << signal_number << "\n"
        << "PID: " << getpid() << "\n"
        << "Report time (ns since epoch): " << NowNanoseconds() << "\n"
        << "\n--- SLAM context (last published by tracking thread) ---\n"
        << "Frame id: " << gContext.frame_id.load(std::memory_order_relaxed) << "\n"
        << "Tracking state: " << gContext.tracking_state.load(std::memory_order_relaxed) << "\n"
        << "KeyFrames in map: " << gContext.keyframes_in_map.load(std::memory_order_relaxed) << "\n"
        << "MapPoints in map: " << gContext.map_points_in_map.load(std::memory_order_relaxed) << "\n"
        << "Maps in atlas: " << gContext.maps_in_atlas.load(std::memory_order_relaxed) << "\n"
        << "Frame timestamp (ns): " << gContext.timestamp_ns.load(std::memory_order_relaxed) << "\n"
        << "\n--- Backtrace ---\n";

    void *frames[kMaxBacktraceFrames];
    const int frame_count = backtrace(frames, kMaxBacktraceFrames);
    char **symbols = backtrace_symbols(frames, frame_count);
    if (symbols)
    {
        for (int i = 0; i < frame_count; ++i)
            out << symbols[i] << "\n";
        free(symbols);
    }
    else
    {
        out << "(backtrace unavailable)\n";
    }

    out << "=== end of report ===\n";
    return out.str();
}

bool CrashMonitor::WriteSyntheticReport(const std::string &reason)
{
    if (gReportPath[0] == '\0')
        return false;

    // Write to a distinct path, not gReportPath: the watchdog calls this and
    // then abort()s, and the SIGABRT handler writes gReportPath with O_TRUNC.
    // Sharing the path would erase the stall diagnosis we just recorded --
    // exactly the information needed to tell a watchdog-detected hang apart
    // from an ordinary crash.
    char synthetic_path[kMaxPathLength];
    const std::size_t base_len = std::strlen(gReportPath);
    static const char kSuffix[] = ".stall";
    if (base_len + sizeof(kSuffix) >= kMaxPathLength)
        return false;
    std::memcpy(synthetic_path, gReportPath, base_len);
    std::memcpy(synthetic_path + base_len, kSuffix, sizeof(kSuffix));

    const int fd = open(synthetic_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0)
        return false;

    SafeWrite(fd, "=== ORB-SLAM3 stall report (watchdog, no signal) ===\n");
    SafeWrite(fd, "Reason: ");
    SafeWrite(fd, reason.c_str());
    SafeWrite(fd, "\n");

    // Memory/swap state is the single most useful discriminator for a stall:
    // a genuine deadlock shows a healthy system, whereas heavy swap usage
    // means the process was simply starved of RAM and the stall is an
    // environment problem, not a code bug. Copy /proc verbatim rather than
    // parsing it, since we may be in a degraded state.
    SafeWrite(fd, "\n--- System memory at stall (from /proc/meminfo) ---\n");
    {
        const int meminfo_fd = open("/proc/meminfo", O_RDONLY);
        if (meminfo_fd >= 0)
        {
            char buf[4096];
            const ssize_t n = read(meminfo_fd, buf, sizeof(buf) - 1);
            if (n > 0)
            {
                buf[n] = '\0';
                SafeWrite(fd, buf);
            }
            close(meminfo_fd);
        }
        else
        {
            SafeWrite(fd, "(unavailable)\n");
        }
    }

    SafeWrite(fd, "\n--- This process (from /proc/self/status) ---\n");
    {
        const int status_fd = open("/proc/self/status", O_RDONLY);
        if (status_fd >= 0)
        {
            char buf[4096];
            const ssize_t n = read(status_fd, buf, sizeof(buf) - 1);
            if (n > 0)
            {
                buf[n] = '\0';
                SafeWrite(fd, buf);
            }
            close(status_fd);
        }
        else
        {
            SafeWrite(fd, "(unavailable)\n");
        }
    }
    SafeWrite(fd, "\n");

    WriteReportToFd(fd, 0, "NONE");
    fsync(fd);
    close(fd);

    // Also announce it on stderr so it survives even if the file is missed.
    SafeWrite(STDERR_FILENO, "\nORB-SLAM3 WATCHDOG STALL: ");
    SafeWrite(STDERR_FILENO, reason.c_str());
    SafeWrite(STDERR_FILENO, "\n  stall report: ");
    SafeWrite(STDERR_FILENO, synthetic_path);
    SafeWrite(STDERR_FILENO, "\n");
    return true;
}

bool CrashMonitor::StartWatchdog(int stall_timeout_ms, int poll_interval_ms)
{
    if (!gInstalled.load())
        return false;
    if (gWatchdogRunning.load())
        return true; // already running: idempotent per the documented contract

    if (stall_timeout_ms <= 0 || poll_interval_ms <= 0 || poll_interval_ms >= stall_timeout_ms)
        return false;

    gWatchdogStopRequested.store(false);
    gWatchdogRunning.store(true);

    gWatchdogThread = std::thread([stall_timeout_ms, poll_interval_ms]() {
        long long last_seen_frame = gContext.frame_id.load(std::memory_order_relaxed);
        auto last_progress_time = std::chrono::steady_clock::now();

        std::unique_lock<std::mutex> lock(gWatchdogMutex);
        while (!gWatchdogStopRequested.load())
        {
            const bool stop_requested = gWatchdogCv.wait_for(
                lock, std::chrono::milliseconds(poll_interval_ms),
                [] { return gWatchdogStopRequested.load(); });
            if (stop_requested)
                break;

            const long long current_frame = gContext.frame_id.load(std::memory_order_relaxed);
            const auto now = std::chrono::steady_clock::now();

            if (current_frame != last_seen_frame)
            {
                last_seen_frame = current_frame;
                last_progress_time = now;
                continue;
            }

            const auto stalled_for =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - last_progress_time).count();
            if (stalled_for >= stall_timeout_ms)
            {
                std::ostringstream reason;
                reason << "watchdog: frame_id has not advanced from " << current_frame
                       << " for " << stalled_for << " ms (timeout " << stall_timeout_ms << " ms)."
                       << " Check the memory section below BEFORE suspecting a deadlock:"
                       << " if SwapTotal-SwapFree is large or MemAvailable is small, this is"
                       << " RAM starvation (too many concurrent instances), not a code bug.";
                WriteSyntheticReport(reason.str());

                // abort() delivers SIGABRT, which the installed handler
                // enriches with a backtrace and re-raises so the exit status
                // still reflects a fatal signal. The backtrace will show
                // whichever thread the signal lands on, not necessarily the
                // stuck one -- the synthetic report above is what carries the
                // stall diagnosis and last-known SLAM context.
                std::abort();
            }
        }
    });

    return true;
}

void CrashMonitor::StopWatchdog()
{
    if (!gWatchdogRunning.exchange(false))
        return;

    {
        std::lock_guard<std::mutex> lock(gWatchdogMutex);
        gWatchdogStopRequested.store(true);
    }
    gWatchdogCv.notify_all();

    if (gWatchdogThread.joinable())
        gWatchdogThread.join();
}

bool CrashMonitor::IsWatchdogRunning()
{
    return gWatchdogRunning.load();
}

} // namespace ORB_SLAM3
