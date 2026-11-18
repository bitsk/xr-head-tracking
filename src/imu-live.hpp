// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Bounded real-input experiment, not an OpenXR pose provider. See
// imu-input-contract.json and start-contract.json for the recovered wire ABI.
#include "qvr_bridge.hpp"
#include "a081_pose_output.hpp"
#include <xr/device/a081_camera_capture.hpp>
#include <xr/device/a081_camera_groups.hpp>
#include "a081_tracking_server.hpp"
#include <xr/device/see_low_latency.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <condition_variable>
#include <memory>
#include <xr/device/a081_controller_frame_export.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <linux/dma-buf.h>
#include <misc/fastrpc.h>
#include <poll.h>
#include <spawn.h>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <vector>

extern char **environ;
extern "C" void a081_apps_mem_counters(std::uint64_t out[4]) __attribute__((weak));
extern "C" void a081_apps_mem_lifetimes(std::uint64_t out[6]) __attribute__((weak));

namespace a081::stock::imu_live {
inline void report(const char *stage, Result r, const std::string &detail = {})
{
    std::lock_guard<std::mutex> log(camera::output_mutex);
    std::cout << "stage=" << stage << " rc=" << r.rc << " errno=" << r.system_errno
              << " message=" << std::quoted(describe(r));
    if (!detail.empty()) std::cout << " detail=" << std::quoted(detail);
    std::cout << std::endl;
}
inline Result failure(int error) { return {-1, error}; }
inline Result sysresult(int rc) { return {rc, rc == -1 ? errno : 0}; }

inline bool text_file(const std::string &path, std::string &out)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;
    std::ostringstream contents;
    contents << file.rdbuf();
    if (file.bad()) return false;
    out = contents.str();
    return true;
}
inline bool attribute(const std::string &path, std::string &out)
{
    if (!text_file(path, out)) return false;
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
    return true;
}
inline Result write_attribute(const std::string &path, const std::string &value)
{
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) return failure(errno);
    const auto n = ::write(fd, value.data(), value.size());
    Result result = n == static_cast<ssize_t>(value.size()) ? Result{} : failure(n < 0 ? errno : EIO);
    if (::close(fd) != 0 && result) result = failure(errno);
    return result;
}
inline std::uint64_t boottime_ns()
{
    timespec now{};
    if (clock_gettime(CLOCK_BOOTTIME, &now) != 0) return 0;
    return static_cast<std::uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;
}

// Use the installed coreutils program through argv, never through a shell.
// Verify the raw files, then apply only the proven config6 LF normalization.
inline bool verified_xml(const char *variable, const char *expected, std::string &xml)
{
    const char *path = std::getenv(variable);
    if (!path || path[0] != '/') {
        report("xml_path", failure(EINVAL), std::string(variable) + " must name an absolute stock XML path");
        return false;
    }
    int descriptors[2];
    if (pipe2(descriptors, O_CLOEXEC) != 0) {
        report("xml_hash_pipe", failure(errno));
        return false;
    }
    posix_spawn_file_actions_t actions;
    int rc = posix_spawn_file_actions_init(&actions);
    const bool actions_initialized = rc == 0;
    if (rc == 0) {
        rc = posix_spawn_file_actions_adddup2(&actions, descriptors[1], STDOUT_FILENO);
        if (rc == 0) rc = posix_spawn_file_actions_addclose(&actions, descriptors[0]);
        if (rc == 0) rc = posix_spawn_file_actions_addclose(&actions, descriptors[1]);
    }
    char command[] = "sha256sum", separator[] = "--";
    char *arguments[] = {command, separator, const_cast<char *>(path), nullptr};
    pid_t child = -1;
    if (rc == 0) rc = posix_spawnp(&child, command, &actions, nullptr, arguments, environ);
    if (actions_initialized) posix_spawn_file_actions_destroy(&actions);
    ::close(descriptors[1]);
    std::string output;
    char buffer[512];
    ssize_t n;
    while ((n = ::read(descriptors[0], buffer, sizeof(buffer))) > 0)
        output.append(buffer, static_cast<std::size_t>(n));
    ::close(descriptors[0]);
    int status = 0;
    pid_t waited = -1;
    if (child > 0) {
        do { waited = waitpid(child, &status, 0); } while (waited < 0 && errno == EINTR);
    }
    const bool matched = rc == 0 && waited == child && WIFEXITED(status) && WEXITSTATUS(status) == 0 &&
                         output.size() >= 64 && output.compare(0, 64, expected) == 0;
    report("xml_hash", matched ? Result{} : failure(rc ? rc : EINVAL),
           std::string(variable) + " path=" + path + " expected=" + expected +
           " actual=" + output.substr(0, std::min<std::size_t>(64, output.size())));
    if (!matched || !text_file(path, xml) || xml.empty() || xml.find('\0') != std::string::npos)
        return false;
    return true;
}

inline volatile std::sig_atomic_t interrupted = 0;
inline void request_stop(int) { interrupted = 1; }
class Signals {
public:
    bool install()
    {
        interrupted = 0;
        struct sigaction action{};
        action.sa_handler = request_stop;
        // Preserve the daemon listener's blocking syscalls when a termination
        // request goes to that thread; the client loop observes the flag.
        action.sa_flags = SA_RESTART;
        sigemptyset(&action.sa_mask);
        if (sigaction(SIGINT, &action, &old_int_) != 0) return false;
        int_set_ = true;
        if (sigaction(SIGTERM, &action, &old_term_) != 0) return false;
        term_set_ = true;
        return true;
    }
    ~Signals()
    {
        if (term_set_) sigaction(SIGTERM, &old_term_, nullptr);
        if (int_set_) sigaction(SIGINT, &old_int_, nullptr);
    }
private:
    struct sigaction old_int_{}, old_term_{};
    bool int_set_{}, term_set_{};
};

struct Sample {
    std::uint64_t time{};
    std::array<float, 3> value{};
    bool gyro{};
    xr::see::Packet source{}; // Original native record and measured clock tuple.
    std::uint64_t qtimer_ns() const { return xr::see::get64(source.event,16); }
};
#include "see-input.hpp"

struct RingSpec {
    head::RingId id;
    std::uint32_t bytes, index, offset, element, count, type;
};
// AP descriptor rows in service-layout-constants.json; allocation sizes from
// stock ELF block table base0x136004 + block_id*28. Frame block21 is16384,
// debug block3 is10485760, prediction block12 is4096, DL block39 is12288.
inline constexpr std::array<RingSpec, 8> ring_specs{{
    {head::RingId::health, 8192, 6816, 0, 6816, 1, 7},
    {head::RingId::raw_gyro, 4096, 0, 64, 64, 8, 1},
    {head::RingId::pose, 16384, 0, 64, 192, 80, 0},
    {head::RingId::frame_pose, 16384, 0, 64, 64, 80, 10},
    {head::RingId::debug_6dof, 10485760, 4096, 0, 4096, 1, 4},
    {head::RingId::predicted_pose, 4096, 0, 64, 192, 8, 8},
    {head::RingId::dl_prediction, 12288, 0, 64, 128, 80, 15},
    {head::RingId::raw_accel, 4096, 0, 64, 64, 8, 1},
}};

class Ring {
public:
    Ring() = default;
    Ring(const Ring &) = delete;
    Ring &operator=(const Ring &) = delete;
    ~Ring() { if (!retained_) release(); }
    bool allocate(int session, RingSpec spec)
    {
        spec_ = spec;
        fastrpc_alloc_dma_buf allocation{};
        allocation.fd = -1;
        allocation.size = spec.bytes;
        errno = 0;
        auto result = sysresult(ioctl(session, FASTRPC_IOCTL_ALLOC_DMA_BUFF, &allocation));
        report("ring_alloc", result, detail() + " allocated_fd=" + std::to_string(allocation.fd));
        if (!result) return false;
        fd_ = allocation.fd;
        host_ = mmap(nullptr, spec.bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        result = host_ == MAP_FAILED ? failure(errno) : Result{};
        report("ring_mmap", result, detail());
        if (!result || !sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE)) return false;
        std::memset(host_, 0, spec.bytes);
        std::atomic_thread_fence(std::memory_order_release);
        return sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
    }
    head::RingDescriptor descriptor() const
    {
        return {fd_, spec_.bytes, spec_.index, spec_.offset, spec_.element, spec_.count, spec_.type, 0};
    }
    std::string detail() const
    {
        return "ring=" + std::to_string(static_cast<unsigned>(spec_.id)) + " fd=" + std::to_string(fd_) +
               " bytes=" + std::to_string(spec_.bytes) + " index=" + std::to_string(spec_.index) +
               " offset=" + std::to_string(spec_.offset) + " element=" + std::to_string(spec_.element) +
               " count=" + std::to_string(spec_.count) + " type=" + std::to_string(spec_.type);
    }
    // Takes ownership of an SCM_RIGHTS reference to the actual SEE DMA-buf.
    // The SDSP source already initialized it. Never clear or repack its bytes.
    bool import(int fd, RingSpec spec)
    {
        if(fd<0 || spec.bytes!=xr::see::allocation_bytes) return false;
        spec_=spec;fd_=fd;
        host_=mmap(nullptr,spec.bytes,PROT_READ|PROT_WRITE,MAP_SHARED,fd_,0);
        report("ring_import",host_==MAP_FAILED ? failure(errno) : Result{},detail());
        return host_!=MAP_FAILED;
    }
    bool summary(const char *when)
    {
        std::lock_guard<std::mutex> guard(read_mutex_);
        if (fd_ < 0 || host_ == MAP_FAILED) return true;
        if (!sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ)) return false;
        std::atomic_thread_fence(std::memory_order_acquire);
        const auto *base = static_cast<const std::uint8_t *>(host_);
        const std::size_t length = static_cast<std::size_t>(spec_.element) * spec_.count;
        std::vector<std::uint8_t> snapshot(length);
        std::memcpy(snapshot.data(), base + spec_.offset, length);
        std::uint32_t header = 0;
        if (spec_.index + sizeof(header) <= spec_.bytes) std::memcpy(&header, base + spec_.index, sizeof(header));
        if (!sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ)) return false;
        std::uint64_t hash = 14695981039346656037ULL, maximum32 = 0;
        std::size_t nonzero = 0;
        for (auto byte : snapshot) { hash = (hash ^ byte) * 1099511628211ULL; nonzero += byte != 0; }
        // SDK offset32 is a timestamp candidate, not a verified live pose ABI.
        // Report the raw field only; no position/orientation validity flags.
        if (spec_.element == 192)
            for (unsigned slot = 0; slot < spec_.count; ++slot) {
                std::uint64_t value = 0;
                std::memcpy(&value, snapshot.data() + slot * spec_.element + 32, sizeof(value));
                maximum32 = std::max(maximum32, value);
            }
        if (!have_baseline_) { baseline_ = hash; have_baseline_ = true; }
        std::cout << "stage=ring_raw_summary when=" << when << " " << detail()
                  << " raw_header_u32=" << header << " fnv1a64=" << hash
                  << " nonzero_bytes=" << nonzero << " changed_from_initial=" << (hash != baseline_)
                  << " raw_u64_at32_max=" << maximum32
                  << " validity=uninterpreted" << std::endl;
        return true;
    }
    bool save_stopped_snapshot()
    {
        std::lock_guard<std::mutex> guard(read_mutex_);
        if (!xr_recording_enabled() || !xr_recording_room(std::getenv("A081_QVR_OUTPUT_DIR"))) return true;
        if (spec_.id != head::RingId::pose && spec_.id != head::RingId::dl_prediction) return true;
        const char *directory = std::getenv("A081_QVR_OUTPUT_DIR");
        if (!directory || directory[0] != '/') {
            report("ring_snapshot", failure(EINVAL), "wrapper output directory required");
            return false;
        }
        if (!sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ)) return false;
        std::atomic_thread_fence(std::memory_order_acquire);
        std::vector<std::uint8_t> snapshot(spec_.bytes), confirm(spec_.bytes);
        std::memcpy(snapshot.data(), host_, snapshot.size());
        std::atomic_thread_fence(std::memory_order_acquire);
        std::memcpy(confirm.data(), host_, confirm.size());
        if (!sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ)) return false;
        // The original ring has only a wrapping slot index, not a seqlock.
        // This is after successful stop and with no new IMU notifications;
        // byte equality alone is not a general concurrent-reader protocol.
        if (snapshot != confirm) {
            report("ring_snapshot", failure(EAGAIN), "buffer changed after stop");
            return false;
        }
        const std::string path = std::string(directory) + "/ring-" +
            std::to_string(static_cast<unsigned>(spec_.id)) + "-after-stop.bin";
        const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
        if (fd < 0) { report("ring_snapshot", failure(errno), path); return false; }
        std::size_t written = 0;
        bool ok = true;
        while (written < snapshot.size()) {
            const auto n = ::write(fd, snapshot.data() + written, snapshot.size() - written);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) { ok = false; break; }
            written += static_cast<std::size_t>(n);
        }
        ok = (::close(fd) == 0) && ok;
        report("ring_snapshot", ok ? Result{} : failure(EIO), path + " bytes=" + std::to_string(written));
        if (spec_.id == head::RingId::pose) {
            pose_output::PoseSnapshot pose;
            const auto decoded = pose_output::decode_latest_pose_snapshot(snapshot.data(), snapshot.size(), pose);
            double norm_squared = 0;
            for (const auto value : pose.rotation_xyzw) norm_squared += static_cast<double>(value) * value;
            std::cout << "stage=pose_snapshot_decode rc=" << static_cast<unsigned>(decoded)
                      << " slot=" << pose.published_index << " timestamp_ns=" << pose.timestamp_ns
                      << " rotation_xyzw=" << pose.rotation_xyzw[0] << ',' << pose.rotation_xyzw[1] << ','
                      << pose.rotation_xyzw[2] << ',' << pose.rotation_xyzw[3]
                      << " quaternion_norm=" << std::sqrt(norm_squared)
                      << " translation=" << pose.translation[0] << ',' << pose.translation[1] << ',' << pose.translation[2]
                      << " stock_tracking_state=" << pose.tracking_state << " warning_flags=" << pose.warning_flags
                      << " pose_quality=" << pose.pose_quality << " sensor_quality=" << pose.sensor_quality
                      << " camera_quality=" << pose.camera_quality
                      << " openxr_valid=0 quiescent_snapshot=1" << std::endl;
            ok = decoded == pose_output::DecodeStatus::ok && ok;
        } else {
            pose_output::DlPredictionSnapshot dl;
            const auto decoded = pose_output::decode_latest_dl_prediction_snapshot(snapshot.data(), snapshot.size(), dl);
            std::cout << "stage=dl_snapshot_decode rc=" << static_cast<unsigned>(decoded)
                      << " slot=" << dl.published_index << " timestamp_ns=" << dl.timestamp_ns
                      << " coefficient_counts=" << unsigned(dl.coefficient_counts[0]) << ','
                      << unsigned(dl.coefficient_counts[1]) << ',' << unsigned(dl.coefficient_counts[2]) << ','
                      << unsigned(dl.coefficient_counts[3]) << " producer_marker=" << unsigned(dl.producer_marker)
                      << " openxr_valid=0 quiescent_snapshot=1" << std::endl;
            ok = decoded == pose_output::DecodeStatus::ok && ok;
        }
        return ok;
    }
    // Original native writer cleans a completed slot before publishing its
    // wrapping index. Select only that published slot. Equal indexes are not
    // a generation/seqlock: copy comparisons detect observed interference,
    // but do not establish a stronger atomicity contract than the OEM reader.
    bool copy_native(pose_output::PoseSnapshot &out, std::uint32_t &status)
    {
        std::lock_guard<std::mutex> guard(read_mutex_);
        if(spec_.id!=head::RingId::pose) return false;
        status=A081_POSE_CHANGED_DURING_COPY;
        for(unsigned attempt=0;attempt<3;++attempt) {
            if(!sync(DMA_BUF_SYNC_START|DMA_BUF_SYNC_READ)) return false;
            const auto *base=static_cast<const std::uint8_t *>(host_);
            const auto *index=reinterpret_cast<const std::uint32_t *>(base+spec_.index);
            const auto first=__atomic_load_n(index,__ATOMIC_ACQUIRE);
            std::array<std::uint8_t,pose_output::pose_slot_bytes> copy{},confirm{};
            if(first<spec_.count) {
                const volatile std::uint8_t *slot=base+spec_.offset+first*spec_.element;
                for(std::size_t i=0;i<copy.size();++i) copy[i]=slot[i];
                std::atomic_thread_fence(std::memory_order_acquire);
                for(std::size_t i=0;i<confirm.size();++i) confirm[i]=slot[i];
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            if(!sync(DMA_BUF_SYNC_END|DMA_BUF_SYNC_READ)) return false;
            if(first>=spec_.count) { status=A081_POSE_BAD_INDEX;return true; }
            // Publishing another slot does not invalidate this completed
            // record. Only interference with the selected record matters.
            if(copy!=confirm) continue;
            pose_output::PoseSnapshot pose{};
            if(pose_output::decode_pose_record(copy.data(),copy.size(),first,pose)!=pose_output::DecodeStatus::ok)
                return false;
            if(!pose.timestamp_ns) { status=A081_POSE_NOT_READ;return true; }
            out=pose;status=A081_POSE_COPIED;return true;
        }
        return true;
    }
    bool query_history(std::uint64_t query_qtimer_ns,controller::NativeHeadPose &out)
    {
        std::lock_guard<std::mutex> guard(read_mutex_);
        ++history_queries_;
        if(spec_.id!=head::RingId::pose || !query_qtimer_ns) { ++history_unavailable_;return false; }
        // Same native no-generation boundary as latest(). The immutable copy
        // covers the complete original history, not an AP sampling cache.
        for(unsigned attempt=0;attempt<3;++attempt) {
            if(!sync(DMA_BUF_SYNC_START|DMA_BUF_SYNC_READ)) { ++history_io_errors_;return false; }
            const auto *base=static_cast<const std::uint8_t *>(host_);
            const auto *index=reinterpret_cast<const std::uint32_t *>(base+spec_.index);
            const auto first=__atomic_load_n(index,__ATOMIC_ACQUIRE);
            std::array<std::uint8_t,pose_output::pose_ring_bytes> snapshot{};
            std::memcpy(snapshot.data(),&first,sizeof(first));
            const volatile std::uint8_t *records=base+spec_.offset;
            for(std::size_t i=pose_output::header_bytes;i<snapshot.size();++i)
                snapshot[i]=records[i-pose_output::header_bytes];
            pose_output::PoseSnapshot selected{};
            const auto decoded=pose_output::decode_historical_pose_snapshot(snapshot.data(),snapshot.size(),
                                                                             query_qtimer_ns,selected);
            std::array<std::uint8_t,pose_output::pose_slot_bytes> confirm{};
            if(decoded==pose_output::DecodeStatus::ok) {
                const volatile std::uint8_t *record=records+selected.published_index*spec_.element;
                for(std::size_t i=0;i<confirm.size();++i) confirm[i]=record[i];
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            if(!sync(DMA_BUF_SYNC_END|DMA_BUF_SYNC_READ)) { ++history_io_errors_;return false; }
            // History spans 80 independent completed records. The current
            // publication index may advance while an older selected record
            // stays unchanged; it is not a generation for the whole ring.
            if(decoded!=pose_output::DecodeStatus::ok) { ++history_unavailable_;return false; }
            const auto *copy=snapshot.data()+pose_output::header_bytes+selected.published_index*spec_.element;
            if(!std::equal(confirm.begin(),confirm.end(),copy)) { ++history_interference_;continue; }
            out.sample_qtimer_ns=selected.timestamp_ns;
            out.position=selected.translation;out.orientation_xyzw=selected.rotation_xyzw;
            ++history_returned_;return true;
        }
        ++history_unavailable_;return false;
    }
    void retain_until_process_exit() { retained_ = true; }
    bool release()
    {
        std::lock_guard<std::mutex> guard(read_mutex_);
        if(history_queries_) {
            report("native_history_summary",{},"queries="+std::to_string(history_queries_)+
                " returned="+std::to_string(history_returned_)+" unavailable="+std::to_string(history_unavailable_)+
                " observed_interference="+std::to_string(history_interference_)+" io_errors="+std::to_string(history_io_errors_)+
                " source=original80slotRing1 sample_epoch=unchanged generation=absent");
            history_queries_=0;
        }
        if (retained_) return false;
        bool ok = true;
        if (host_ != MAP_FAILED) {
            errno = 0;
            const auto result = sysresult(munmap(host_, spec_.bytes));
            report("ring_munmap", result, detail());
            ok = static_cast<bool>(result);
            host_ = MAP_FAILED;
        }
        if (fd_ >= 0) {
            errno = 0;
            const auto result = sysresult(::close(fd_));
            report("ring_close_fd", result, detail());
            ok = static_cast<bool>(result) && ok;
            fd_ = -1;
        }
        return ok;
    }
private:
    bool sync(std::uint64_t flags)
    {
        dma_buf_sync request{flags};
        int rc;
        do { errno = 0; rc = ioctl(fd_, DMA_BUF_IOCTL_SYNC, &request); } while (rc < 0 && errno == EINTR);
        if (rc != 0) report("ring_cpu_sync", sysresult(rc), detail() + " flags=" + std::to_string(flags));
        return rc == 0;
    }
    std::mutex read_mutex_;
    std::uint64_t history_queries_{},history_returned_{},history_unavailable_{},history_interference_{},history_io_errors_{};
    RingSpec spec_{};
    int fd_ = -1;
    void *host_ = MAP_FAILED;
    std::uint64_t baseline_{};
    bool have_baseline_{}, retained_{};
};

class VisualInput {
public:
    explicit VisualInput(std::uint64_t exposure_ns)
        : groups(exposure_ns, [this](const camera::FrameGroup &group) { controller_export.frame(group); }),
          exposure_ns(exposure_ns) {}
    ~VisualInput() { finish(); }
    void start()
    {
        callbacks.frame = [this](unsigned slot, camera::TimedFrame &&frame) { groups.push(slot, std::move(frame)); };
        callbacks.gate = [this](std::uint64_t time) { groups.gate_high(time); };
        callbacks.abort = &abort;
        worker = std::thread([this] {
            result.store(camera::capture_live(&callbacks));
            ended_at.store(boottime_ns());
            done.store(true);
        });
    }
    void finish()
    {
        abort.store(true);
        if (worker.joinable()) worker.join();
    }
    // finish() joins capture before groups and the exporter are destroyed.
    controller::FrameExporter controller_export;
    controller::FrameExporter hand_export{"A081_HAND_INPUT_SOCKET"};
    camera::FrameGroups groups;
    const std::uint64_t exposure_ns;
    std::atomic<bool> done{false}, abort{false};
    std::atomic<int> result{-1};
    std::atomic<std::uint64_t> ended_at{0};
private:
    camera::CaptureCallbacks callbacks;
    std::thread worker;
};

// AP publication is independent of the camera RPC caller. QVR's original
// native adsp_imu/QDI thread alone runs HRP; this reader never feeds the DSP.
class NativePoseReader {
public:
    NativePoseReader(DirectImuInput &input,Ring &pose,TrackingServer &stream)
        : input_(input),pose_(pose),stream_(stream) {}
    ~NativePoseReader() { stop(); }
    bool start(const std::array<int,3> &axes) {
        worker_=std::thread([this,axes] { run(axes); });
        if(input_.start()) return true;
        failed_=true;stop();return false;
    }
    void stop() {
        stopping_=true;wake_.notify_all();
        if(worker_.joinable()) worker_.join();
    }
    bool wait() {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock,std::chrono::milliseconds(20));
        return !failed_.load();
    }
    std::uint64_t latest_time() const { return latest_boot_.load(); }
    std::uint64_t samples() const { return gyro_samples_.load(); }
    void camera_submitted(std::uint64_t time) { camera_time_=time; }
    void summary() const {
        std::cout << "stage=native_pose_reads copied=" << reads_[A081_POSE_COPIED]
                  << " bad_index=" << reads_[A081_POSE_BAD_INDEX]
                  << " changed=" << reads_[A081_POSE_CHANGED_DURING_COPY]
                  << " timestamp_unmatched=" << reads_[A081_POSE_TIMESTAMP_MISMATCH]
                  << " not_ready=" << reads_[A081_POSE_NOT_READ]
                  << " missing_input=" << missing_input_ << " association_waits=" << unmatched_
                  << " pose_updates=" << pose_updates_ << " last_pose_qtimer_ns=" << last_pose_epoch_
                  << " producer_error=" << failed_.load()
                  << " input_capacity=" << xr::see::native_capacity
                  << " consistency=OEM_published_slot_no_generation physical_validated=0" << std::endl;
    }
private:
    void run(const std::array<int,3> &axes) {
        std::deque<Sample> history[2];
        std::deque<pose_output::PoseSnapshot> pending;
        std::uint64_t observed_epoch=0,published_epoch=0;
        while(!stopping_.load()) {
            pollfd descriptor{input_.fd(),POLLIN,0};
            const int ready=poll(&descriptor,1,10);
            if(ready<0 && errno==EINTR) continue;
            if(ready<0 || ((descriptor.revents&(POLLERR|POLLHUP|POLLNVAL)) && !(descriptor.revents&POLLIN))) {
                failed_=true;break;
            }
            if(!ready) continue;
            std::vector<Sample> batch;
            if(!input_.read_batch(batch,axes)) { failed_=true;break; }
            for(const auto &sample:batch) {
                auto &cache=history[sample.gyro ? 1 : 0];cache.push_back(sample);
                // Exactly the original 4096/104 native input capacity; no
                // additional motion, age, quality or prediction threshold.
                while(cache.size()>xr::see::native_capacity) cache.pop_front();
                if(sample.gyro) {
                    ++gyro_samples_;latest_boot_=sample.time;
                }
            }
            wake_.notify_one();
            const auto samples=gyro_samples_.load();
            if(samples && (last_progress_group_==UINT64_MAX ||
                (xr_diagnostic_logs_enabled() && samples/xr::see::native_capacity!=last_progress_group_))) {
                last_progress_group_=samples/xr::see::native_capacity;
                report("native_pose_progress",{},"gyro_samples="+std::to_string(samples)+
                    " pose_updates="+std::to_string(pose_updates_)+" last_pose_qtimer_ns="+std::to_string(last_pose_epoch_)+
                    " association_waits="+std::to_string(unmatched_)+" producer_error=0 freshness=not_inferred_from_state");
            }
            if(history[0].empty() || history[1].empty()) { ++missing_input_;continue; }
            pose_output::PoseSnapshot pose{};std::uint32_t status=A081_POSE_NOT_READ;
            if(!pose_.copy_native(pose,status)) { failed_=true;break; }
            ++reads_.at(status);
            if(status==A081_POSE_COPIED && pose.timestamp_ns>observed_epoch) {
                observed_epoch=pose.timestamp_ns;pending.push_back(pose);
                while(pending.size()>pose_output::slot_count) { pending.pop_front();++unmatched_; }
            }
            // QDI and AP metadata delivery are independent. Preserve real
            // published records while their matching source event is in flight;
            // selecting only the newest ring entry can perpetually run ahead.
            auto selected=pending.rend();auto gyro=history[1].end();
            for(auto candidate=pending.rbegin();candidate!=pending.rend();++candidate) {
                if(candidate->timestamp_ns<=published_epoch) break;
                gyro=std::find_if(history[1].begin(),history[1].end(),[&](const Sample &sample) {
                    return sample.qtimer_ns()==candidate->timestamp_ns;
                });
                if(gyro!=history[1].end()) { selected=candidate;break; }
            }
            if(selected==pending.rend()) { ++unmatched_;continue; }
            pose=*selected;
            auto accel=history[0].rbegin();
            const auto distance=[&](const Sample &sample) {
                const auto a=sample.qtimer_ns(),g=gyro->qtimer_ns();return a>g ? a-g : g-a;
            };
            for(auto candidate=history[0].rbegin();candidate!=history[0].rend();++candidate)
                if(distance(*candidate)<distance(*accel)) accel=candidate;
            const __int128 accel_boot=static_cast<__int128>(accel->qtimer_ns())+gyro->source.qtimer_to_boottime_ns;
            if(accel_boot<=0 || accel_boot>UINT64_MAX) { failed_=true;break; }
            a081_tracking_packet packet{};
            packet.flags=A081_TRACKING_IMU;
            packet.gyro_boottime_ns=gyro->time;
            packet.accel_boottime_ns=static_cast<std::uint64_t>(accel_boot);
            packet.last_camera_boottime_ns=camera_time_.load();
            if(packet.last_camera_boottime_ns) packet.flags|=A081_TRACKING_CAMERA_SUBMITTED;
            std::copy(gyro->value.begin(),gyro->value.end(),packet.gyro);
            std::copy(accel->value.begin(),accel->value.end(),packet.accel);
            packet.pose_read_status=A081_POSE_COPIED;
            {
                packet.flags|=A081_TRACKING_RAW_POSE;
                // SLPI VR mode writes QTimer ticks*625/12, no offset. The
                // native DSP copies that gyro timestamp unchanged into pose.
                // Use that record's measured mapping, never AP receive time.
                packet.pose_boottime_ns=gyro->time;packet.pose_index=pose.published_index;
                std::copy(pose.rotation_xyzw.begin(),pose.rotation_xyzw.end(),packet.rotation_xyzw);
                std::copy(pose.translation.begin(),pose.translation.end(),packet.translation);
                std::copy(pose.angular_s.begin(),pose.angular_s.end(),packet.angular_s);
                std::copy(pose.angular_b.begin(),pose.angular_b.end(),packet.angular_b);
                std::copy(pose.angular_bdt.begin(),pose.angular_bdt.end(),packet.angular_bdt);
                std::copy(pose.angular_bdt2.begin(),pose.angular_bdt2.end(),packet.angular_bdt2);
                std::copy(pose.translation_s.begin(),pose.translation_s.end(),packet.translation_s);
                std::copy(pose.translation_b.begin(),pose.translation_b.end(),packet.translation_b);
                packet.pose_quality=pose.pose_quality;packet.tracking_state=pose.tracking_state;
                packet.warning_flags=pose.warning_flags;
            }
            stream_.publish(packet);published_epoch=pose.timestamp_ns;
            ++pose_updates_;last_pose_epoch_=pose.timestamp_ns;
            while(!pending.empty() && pending.front().timestamp_ns<=published_epoch) pending.pop_front();
        }
        wake_.notify_all();
    }
    DirectImuInput &input_;Ring &pose_;TrackingServer &stream_;
    std::thread worker_;std::atomic<bool> stopping_{false},failed_{false};
    std::atomic<std::uint64_t> latest_boot_{0},gyro_samples_{0},camera_time_{0};
    std::mutex mutex_;std::condition_variable wake_;
    std::array<std::uint64_t,5> reads_{};std::uint64_t missing_input_{},unmatched_{},pose_updates_{},last_pose_epoch_{},last_progress_group_=UINT64_MAX;
};

inline bool feed(QvrModule &module, DirectImuInput &input, Ring &pose_ring,
                 TrackingServer &stream, const std::array<int, 3> &axes, std::uint64_t start,
                 VisualInput *visual = nullptr)
{
    const char *continuous_value = std::getenv("A081_QVR_CONTINUOUS");
    const bool continuous = visual && continuous_value && std::string(continuous_value) == "1";
    const auto deadline = continuous ? UINT64_MAX : start + (visual ? 370000000000ULL : 5000000000ULL);
    const std::uint64_t max_notifications = continuous ? UINT64_MAX : (visual ? 200000 : 5000);
    std::uint64_t camera_calls=0,last_camera_time=0,last_camera_rpc_ns=0;
    std::uint64_t last_qvr_exposure_start=0,camera_short_skipped=0,camera_rate_skipped=0;
    NativePoseReader reader(input,pose_ring,stream);
    if(!reader.start(axes)) { report("native_reader_start",failure(EIO));return false; }
    bool ok=true;
    while(!interrupted && boottime_ns()<deadline && reader.samples()<max_notifications) {
        if(!reader.wait()) { ok=false;break; }
        if(visual && visual->done.load()) {
            if(visual->result.load()!=0) { ok=false;break; }
            if(visual->groups.empty()) break;
        }
        const auto last=reader.latest_time();
        if(!last) continue;
        if (visual) {
            // The native QDI thread advances HRP independently. This
            // caller only submits real camera groups once source time reached
            // their capture time; it never drains or injects IMU notifications.
            camera::FrameGroup group;
            if (visual->groups.take(last, group)) {
                head::Bytes tracking, controllers;
                if (group.submit_boottime_ns <= last_camera_time ||
                    group.frames[0].image.sequence > INT32_MAX || group.frames[2].image.sequence > INT32_MAX ||
                    !camera::side_by_side(group.frames[0].image.y8, group.frames[1].image.y8, tracking) ||
                    !camera::side_by_side(group.frames[2].image.y8, group.frames[3].image.y8, controllers)) {
                    report("camera_group", failure(EINVAL), "Invalid real stereo payload/order"); ok = false; break;
                }
                // Stock VRTrackerF skips frames whose AE-active flag is false
                // and enforces a 30,303,030ns SOE interval before DSP submission.
                // In our kernel three-phase schedule only phase2 is the held-AE
                // tracking exposure. Controller export still receives both short
                // phases, with their real sensor frame numbers and timestamps.
                const auto &tracking_meta = group.frames[0].timing;
                camera::StockY8NativeTime qvr_time{};
                if (!camera::stock_a081_y8_native_time(tracking_meta.sof_qtimer,
                        camera::frame_exposure_ns(tracking_meta, visual->exposure_ns), qvr_time)) {
                    report("camera_group", failure(EINVAL), "Invalid real tracking exposure time"); ok = false; break;
                }
                const bool short_phase = camera::valid_pattern_exposure(tracking_meta) &&
                    tracking_meta.exposure_phase != 2;
                const bool rate_skip = last_qvr_exposure_start &&
                    (qvr_time.exposure_start_qtimer_ns <= last_qvr_exposure_start ||
                     qvr_time.exposure_start_qtimer_ns - last_qvr_exposure_start < 30303030ULL);
                if (short_phase || rate_skip) {
                    if (short_phase) ++camera_short_skipped; else ++camera_rate_skipped;
                    continue;
                }
                // The real QCFT SOF counter is the same19.2MHz QTimer
                // as native IMU. Apply625/12 and original exposure arithmetic
                // directly, avoiding a BOOTTIME/NTP conversion round trip.
                const auto camera_begin_ns = boottime_ns();
                const auto r = module.camera_frame_from_qtimer_sof(group.frames[0].timing.sof_qtimer,
                    camera::frame_exposure_ns(group.frames[0].timing, visual->exposure_ns), tracking, controllers,
                    static_cast<std::int32_t>(group.frames[0].image.sequence),
                    static_cast<std::int32_t>(group.frames[2].image.sequence));
                last_camera_rpc_ns = boottime_ns() - camera_begin_ns;
                if (!r || xr_diagnostic_logs_enabled()) report("camera_frame", r, "submit_boottime_ns=" + std::to_string(group.submit_boottime_ns) +
                       " rpc_wall_ns=" + std::to_string(last_camera_rpc_ns) +
                       " tracking_frame=" + std::to_string(group.frames[0].image.sequence) +
                       " controller_frame=" + std::to_string(group.frames[2].image.sequence));
                if (!r) { ok = false; break; }
                if (camera_calls == 0 && xr_recording_enabled() && xr_recording_room(std::getenv("A081_QVR_OUTPUT_DIR"))) {
                    const std::string output = std::getenv("A081_QVR_OUTPUT_DIR");
                    for (unsigned slot = 0; slot < 4; ++slot)
                        camera::save_bytes(output + "/camera-slot" + std::to_string(slot) +
                                           "-first-submitted.y8", group.frames[slot].image.y8);
                }
                if (xr_diagnostic_logs_enabled()) {
                    std::lock_guard<std::mutex> log(camera::output_mutex);
                    std::cout << "stage=camera_group_submitted submit_boottime_ns=" << group.submit_boottime_ns;
                    for (unsigned slot = 0; slot < 4; ++slot) {
                        const auto &f = group.frames[slot];
                        std::cout << " slot" << slot << "=" << f.timing.stream_id << ',' << f.image.sequence << ','
                                  << f.image.buffer_index << ',' << f.timing.enqueue_cookie << ',' << f.timing.sof_boottime_ns;
                    }
                    std::cout << " fields=stream_id,sequence,index,cookie,sof_boottime_ns" << std::endl;
                }
                ++camera_calls; last_camera_time = group.submit_boottime_ns;
                reader.camera_submitted(last_camera_time);
                last_qvr_exposure_start = qvr_time.exposure_start_qtimer_ns;
                // This same group's synchronous QVR camera RPC returned OK.
                // It is not a global DSP-idle barrier and does not wait for
                // hand inference; future original IMU/camera calls continue.
                visual->hand_export.frame(group);
            }
        }
    }
    reader.stop();
    {
        std::lock_guard<std::mutex> log(camera::output_mutex);
        reader.summary();
        std::cout << "stage=imu_input_summary elapsed_ns=" << boottime_ns()-start
                  << " native_gyro_samples=" << reader.samples()
                  << " camera_rpc_success=" << camera_calls
                  << " camera_short_skipped=" << camera_short_skipped
                  << " camera_rate_skipped=" << camera_rate_skipped
                  << " last_camera_submit_ns=" << last_camera_time
                  << " last_camera_rpc_ns=" << last_camera_rpc_ns
                  << " native_QDI_HRP=requires_device_verification" << std::endl;
    }
    input.summary();
    return ok && !interrupted && reader.samples()>0 && (!visual || camera_calls>0);
}

inline int run(int session, const std::string &uri)
{
    if (!a081_apps_mem_counters || !a081_apps_mem_lifetimes) {
        report("imu_live_gate", failure(ENOTSUP), "requires private in-process -P plugin");
        return 2;
    }
    const std::array<int, 3> axes{1,2,3};
    constexpr unsigned requested_rate = 800;
    DirectImuInput input;
    std::unique_ptr<VisualInput> visual;
    if (const char *option = std::getenv("A081_QVR_VISUAL")) {
        const char *value = std::getenv("A081_QVR_CAMERA_EXPOSURE_NS");
        char *end = nullptr;
        errno = 0;
        const auto exposure = value ? std::strtoull(value, &end, 10) : 0;
        if (std::string(option) != "qcft" || errno || !exposure || exposure > 1000000000ULL ||
            !end || *end || !std::getenv("A081_QVR_CAMERA_TIMING_NODES") || !std::getenv("A081_QVR_CAMERA_STOP_FD")) {
            report("visual_gate", failure(EINVAL), "Requires QCFT and a real gate owner with fixed pre-STREAMON exposure"); return 2;
        }
        visual = std::make_unique<VisualInput>(exposure);
    }
    std::string model, config, calibration;
    const char *device_option = std::getenv("XR_DEVICE");
    const std::string device = device_option ? device_option : "a081";
    if (device != "a081" && device != "vr700") {
        report("imu_live_gate", failure(EINVAL), "XR_DEVICE must be a081 or vr700");
        return 2;
    }
    const bool vr700 = device == "vr700";
    const char *cvp_option = std::getenv("A081_QVR_CVP_MODE");
    const std::string cvp_mode = cvp_option ? cvp_option : "stock";
    if (cvp_mode != "stock" && cvp_mode != "disabled") {
        report("imu_live_gate", failure(EINVAL), "A081_QVR_CVP_MODE must be stock or disabled");
        return 2;
    }
    // The stock parser and Tracker3_5Core branches support this exact XML
    // option. It disables CVP work; it does not emulate a CVP service or alter
    // the signed algorithm. See cvp-start-dependency.json.
    const char *config_hash = cvp_mode == "disabled"
        ? "f67e7189447212a13b82bb18cc14161221e84d1f36eece205eed6a61ed9be276"
        : "aeef461d9ac5b191f0d2c3e83de3a5d45f305b0f7286c8fc4c5bcecb3b612db9";
    const char *calibration_hash = "eeee483e5ac268a85cd2b6ba69241bb951612045fe05d61de2f35301d816ac00";
    if (vr700) {
        // VR700 ships the same tracker, CVP DSP library and stock XML as
        // the analyzed algorithm. Both exact XML variants use its existing
        // CvpConfig.enable branch. Calibration is per device and
        // its expected digest comes from the private imported session profile.
        const char *profile_config_hash = std::getenv("XR_HEAD_CONFIG_SHA256");
        calibration_hash = std::getenv("XR_HEAD_CALIBRATION_SHA256");
        if (!profile_config_hash || std::strcmp(profile_config_hash, config_hash) ||
            !calibration_hash || std::strlen(calibration_hash) != 64 ||
            std::strspn(calibration_hash, "0123456789abcdef") != 64) {
            report("imu_live_gate", failure(EINVAL), "VR700 requires the pinned config variant digest and a private calibration digest");
            return 2;
        }
        std::string compatible;
        if (!text_file("/proc/device-tree/compatible", compatible) ||
            compatible.compare(0, sizeof("iqiyi,vr700"), "iqiyi,vr700", sizeof("iqiyi,vr700"))) {
            report("imu_live_gate", failure(EINVAL), "VR700 requires matching device-tree compatible");
            return 2;
        }
    }
    if (!text_file("/proc/device-tree/model", model) || model.find(vr700 ? "VR700" : "A081") == std::string::npos ||
        !verified_xml("A081_QVR_CONFIG6", config_hash, config) ||
        !verified_xml("A081_QVR_CALIBRATION", calibration_hash, calibration)) {
        report("imu_live_gate", failure(EINVAL), "Matching device identity, pinned config variant and original calibration are required before mutation");
        return 2;
    }
    report("config_variant", {}, "device=" + device + " cvp_mode=" + cvp_mode +
           " config_sha256=" + config_hash + " signed_algorithm=unchanged calibration=original");
    config.erase(std::remove(config.begin(), config.end(), '\n'), config.end());
    std::cout << "stage=imu_live_scope max_input_seconds=" << (visual ? 370 : 5) << " max_notifications=" << (visual ? 200000 : 5000) <<
                 " source=real_see_low_latency camera_input=" << (visual ? "real_qcft_four_camera" : "none")
              << " visual_6dof=not_validated DSP_start_sensor_frequency=" << requested_rate
              << " continuous_requested=" << (std::getenv("A081_QVR_CONTINUOUS") ? std::getenv("A081_QVR_CONTINUOUS") : "0")
              << " actual_frequency=reported_per_stream RPC_calls=synchronous_external_process_timeout_still_required" << std::endl;
    Signals signals;
    std::array<Ring, 8> rings;
    TrackingServer stream;
    QvrModule module(session);
    bool opened = false, initialized = false, ok = true, safe_to_release = true;
    std::uint64_t before[4]{};
    a081_apps_mem_counters(before);
    std::uint64_t owned_before[6]{};
    a081_apps_mem_lifetimes(owned_before);
    std::string message;
    try {
        ok = stream.open_from_environment();
        if (!ok) report("tracking_stream_open", failure(errno));
        ok = ok && signals.install();
        if (ok) { const auto r = module.open(uri, message); report("open", r, message); opened = ok = static_cast<bool>(r); }
        if (ok) { const auto r = module.driver_version(message); report("driver_version", r, message); ok = static_cast<bool>(r); }
        if(ok) ok=input.connect();
        for(std::size_t i=0;ok && i<rings.size();++i) {
            if(ring_specs[i].id==head::RingId::raw_gyro) ok=rings[i].import(input.take_buffer(1),ring_specs[i]);
            else if(ring_specs[i].id==head::RingId::raw_accel) ok=rings[i].import(input.take_buffer(0),ring_specs[i]);
            else ok=rings[i].allocate(session,ring_specs[i]);
        }
        if (ok) { const auto r = module.init(); report("init", r); initialized = ok = static_cast<bool>(r); }
        if (ok) { const auto r = module.tracker_version(message); report("tracker_version", r, message); ok = static_cast<bool>(r); }
        for (std::size_t i = 0; ok && i < rings.size(); ++i) {
            const auto r = module.set_ring_layout(ring_specs[i].id, rings[i].descriptor());
            report("set_ring_layout", r, rings[i].detail());
            ok = static_cast<bool>(r);
        }
        for (auto &ring : rings) if (ok) ok = ring.summary("before_start");
        if (ok) { const auto r = module.enable_position_mode(1); report("enable_position_mode", r, "DSP mode=1"); ok = static_cast<bool>(r); }
        if (ok) { const auto r = module.sensor_orientation_identity(); report("sensor_orientation", r, "stock indices=0,1,2 signs=1,1,1"); ok = static_cast<bool>(r); }
        if (ok) {
            std::uint64_t ticks = 0;
            const auto r = module.qtimer_ticks(ticks);
            report("qtimer_ticks", r, "raw_ticks=" + std::to_string(ticks) + " native sensor/pose use QTimer ns");
            ok = static_cast<bool>(r);
        }
        // QVR DSP start ELF0x7a3c only calls set_cvp_fds if camera_fds_len>0.
        // Supply the real calibration names/XML but no fabricated camera fds
        // or frames. No observed hard camera-buffer gate precedes tracker init.
        if (ok) {
            const auto r = module.start(config, calibration, static_cast<float>(requested_rate));
            report("start", r, std::string("stock flags=0 CVP_buffers=0 camera_input=") + (visual ? "inline_Y8" : "none") + " requested_hz=" + std::to_string(requested_rate));
            ok = static_cast<bool>(r);
        }
        if (ok && !interrupted) {
            const auto start_time = boottime_ns();
            ok = start_time != 0;
            if(ok && visual) {
                const controller::NativeHistoryQuery query=[&rings](std::uint64_t qt,controller::NativeHeadPose &out) {
                    return rings[2].query_history(qt,out);
                };
                visual->controller_export.set_native_history_query(query);
                visual->hand_export.set_native_history_query(query);
                visual->start();
            }
            if (ok) ok = feed(module, input, rings[2], stream, axes, start_time, visual.get());
        } else if (interrupted) {
            ok = false;
        }
        if (module.started())
            for (auto &ring : rings) ok = ring.summary("after_input_raw_snapshot") && ok;
    } catch (const std::exception &e) {
        report("imu_live_exception", failure(EIO), e.what());
        ok = false;
    }
    // EOF invalidates consumers before DSP stop/deinit and direct-stream closure.
    { std::lock_guard<std::mutex> log(camera::output_mutex); stream.close(); }
    // Stop and acknowledge the source before CDSP teardown. Imported fd
    // references remain in rings until every borrowed DSP mapping is released.
    const bool source_stopped=input.stop();
    report("native_source_stop",source_stopped ? Result{} : failure(EIO));
    ok=source_stopped && ok;
    input.close();
    if (visual) {
        visual->finish(); // Joins capture callbacks; main hand-frame caller already ended.
        visual->controller_export.set_native_history_query({});
        visual->hand_export.set_native_history_query({});
        ok = visual->result.load() == 0 && ok;
        visual->groups.summary();
    }
    // All cleanup is explicit so the log includes the real return values.
    if (module.stop_required()) {
        report("stop_begin", {}, "boottime_ns=" + std::to_string(boottime_ns()));
        const auto r = module.stop(); report("stop", r);
        safe_to_release = static_cast<bool>(r);
        ok = safe_to_release && ok;
    }
    if (initialized && safe_to_release)
        for (auto &ring : rings) ok = ring.summary("after_stop") && ok;
    if (initialized && safe_to_release)
        for (auto &ring : rings) ok = ring.save_stopped_snapshot() && ok;
    if (initialized && safe_to_release) {
        const auto r = module.deinit(); report("deinit", r);
        ok = static_cast<bool>(r) && ok;
    }
    if (opened && safe_to_release) {
        const auto r = module.close(message); report("close", r, message);
        safe_to_release = static_cast<bool>(r);
        ok = safe_to_release && ok;
    }
    std::uint64_t after[4]{};
    a081_apps_mem_counters(after);
    std::uint64_t owned[6]{};
    a081_apps_mem_lifetimes(owned);
    const bool balanced = after[2] == before[2] && after[0] - before[0] == after[1] - before[1];
    std::cout << "stage=imu_mapping_check rc=" << (balanced ? 0 : 1)
              << " maps=" << after[0] - before[0] << " unmaps=" << after[1] - before[1]
              << " live=" << after[2] << " errors=" << after[3] - before[3]
              << " scope=explicit_unmap" << std::endl;
    const auto borrowed_maps = (after[0] - before[0]) - (owned[0] - owned_before[0]);
    const auto borrowed_unmaps = (after[1] - before[1]) - (owned[1] - owned_before[1]);
    const bool fresh = before[0] == 0 && before[1] == 0 && before[2] == 0 && before[3] == 0 &&
                       std::all_of(std::begin(owned_before), std::end(owned_before),
                                   [](std::uint64_t value) { return value == 0; });
    const bool borrowed_balanced = fresh && after[2] >= owned[2] &&
        after[2] - owned[2] == 0 && borrowed_maps == borrowed_unmaps;
    // Only the exact original MemLogger request may cross the module boundary.
    // Exit3 means REQUIRED post-process proof, never a successful session.
    const bool pd_release_pending = owned[2] == 1 && owned[4] == 1 &&
        owned[3] == 2101248 && owned[5] == 2101248 && owned[0] == owned[1] + 1;
    const bool owned_balanced = owned[2] == 0 && owned[3] == 0 && owned[4] == 0 &&
        owned[5] == 0 && owned[0] == owned[1];
    std::cout << "stage=imu_module_mapping_check rc=" << (borrowed_balanced ? 0 : 1)
              << " borrowed_maps=" << borrowed_maps << " borrowed_unmaps=" << borrowed_unmaps
              << " total_live=" << after[2] << " owned_live=" << owned[2]
              << " owned_bytes=" << owned[3] << std::endl;
    safe_to_release = safe_to_release && borrowed_balanced && (owned_balanced || pd_release_pending);
    ok = safe_to_release && after[3] == before[3] && ok;
    for (auto &ring : rings) {
        if (safe_to_release) ok = ring.release() && ok;
        else ring.retain_until_process_exit();
    }
    if (!safe_to_release)
        report("ring_release_deferred", failure(EBUSY), "retain DMA mappings/fds until private plugin process exits and closes its FastRPC session");
    if (ok && pd_release_pending) {
        std::cout << "stage=imu_pd_release_required rc=3 owned_live=" << owned[2]
                  << " owned_bytes=" << owned[3]
                  << " final_reclamation=not_yet_observed" << std::endl;
        return 3;
    }
    return ok ? 0 : 1;
}
} // namespace a081::stock::imu_live
