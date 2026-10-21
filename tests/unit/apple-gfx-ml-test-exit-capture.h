// Proposed test-only opt-in exit capture. No product ABI or native operation.
#pragma once
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>

namespace qmetal_exit_capture {
struct State {
    bool enabled = false;
    bool successful = false;
    int ready_fd = -1;
    int release_fd = -1;
};
inline State state;

enum class ReleaseRead { released, eof, read_failure, wrong_token };

// Reused F5 functions; only namespace and storage location differ.
inline bool parse_nonnegative_fd(const char* text, int* out)
{
    if (!text || !*text) return false;
    unsigned long value = 0;
    for (const char* cursor = text; *cursor; ++cursor) {
        if (*cursor < '0' || *cursor > '9') return false;
        const unsigned digit = static_cast<unsigned>(*cursor - '0');
        if (value > (static_cast<unsigned long>(INT_MAX) - digit) / 10) return false;
        value = value * 10 + digit;
    }
    *out = static_cast<int>(value);
    return true;
}

inline bool descriptor_is_open(int fd)
{
    int result = -1;
    do {
        result = fcntl(fd, F_GETFD);
    } while (result == -1 && errno == EINTR);
    return result != -1;
}

inline bool write_ready_signal(int fd)
{
    static constexpr char signal[] = "F5_PLATFORM_READY\n";
    struct sigaction ignore{};
    ignore.sa_handler = SIG_IGN;
    sigemptyset(&ignore.sa_mask);
    struct sigaction prior{};
    if (sigaction(SIGPIPE, &ignore, &prior) != 0) return false;

    bool complete = true;
    size_t offset = 0;
    while (offset < sizeof(signal) - 1) {
        const ssize_t written = write(fd, signal + offset, sizeof(signal) - 1 - offset);
        if (written > 0) {
            offset += static_cast<size_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR) continue;
        complete = false;
        break;
    }
    if (sigaction(SIGPIPE, &prior, nullptr) != 0) complete = false;
    return complete;
}

inline ReleaseRead wait_for_release(int fd)
{
    char token{};
    for (;;) {
        const ssize_t read_count = read(fd, &token, sizeof(token));
        if (read_count == 1) return token == 'R' ? ReleaseRead::released : ReleaseRead::wrong_token;
        if (read_count == 0) return ReleaseRead::eof;
        if (errno != EINTR) return ReleaseRead::read_failure;
    }
}

[[noreturn]] inline void protocol_failure(int status)
{
    // exit() inside an atexit callback is not used. All graphics owners have
    // already returned; a failed protocol is fatal to this disposable row.
    std::fprintf(stderr, "QMETAL_EXIT_CAPTURE protocol_failure=%d errno=%d\n", status, errno);
    std::fflush(stderr);
    std::_Exit(status);
}

inline void checkpoint()
{
    if (!state.enabled || !state.successful) return;
    if (!write_ready_signal(state.ready_fd)) protocol_failure(113);
    switch (wait_for_release(state.release_fd)) {
    case ReleaseRead::released: break;
    case ReleaseRead::eof: protocol_failure(114);
    case ReleaseRead::read_failure: protocol_failure(115);
    case ReleaseRead::wrong_token: protocol_failure(116);
    }
    // Linux close is not retried after EINTR: a recycled FD is never targeted.
    if (close(state.ready_fd) != 0) protocol_failure(117);
    if (close(state.release_fd) != 0) protocol_failure(118);
    if (std::fprintf(stderr, "QMETAL_EXIT_CAPTURE result=RELEASED\n") < 0 ||
        std::fflush(stderr) != 0) protocol_failure(119);
}

inline int initialize(int argc, char** argv)
{
    // Registered-case and help paths retain their original output and never
    // participate in the actual-row protocol, even with inherited opt-in keys.
    for (int i = 1; i < argc; ++i) {
        if (argv[i] && (!std::strcmp(argv[i], "--list-test-cases") ||
            !std::strcmp(argv[i], "-ltc") || !std::strcmp(argv[i], "--help") ||
            !std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "-?"))) return 0;
    }
    const char* ready = std::getenv("QMETAL_TEST_EXIT_READY_FD");
    const char* release = std::getenv("QMETAL_TEST_EXIT_RELEASE_FD");
    if (!ready && !release) return 0;
    int ready_fd = -1, release_fd = -1;
    if (!ready || !release || !parse_nonnegative_fd(ready, &ready_fd) ||
        !parse_nonnegative_fd(release, &release_fd) || ready_fd < 3 ||
        release_fd < 3 || ready_fd == release_fd ||
        !descriptor_is_open(ready_fd) || !descriptor_is_open(release_fd)) return 111;
    state.enabled = true;
    state.ready_fd = ready_fd;
    state.release_fd = release_fd;
    // First main operation; every later successful atexit registration runs
    // before this callback. Check this registration itself; no logger claim.
    if (std::atexit(checkpoint) != 0) return 112;
    return 0;
}

inline int result(int value)
{
    state.successful = value == 0;
    return value;
}
} // namespace qmetal_exit_capture
