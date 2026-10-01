// SPDX-License-Identifier: GPL-3.0
// Shared-memory "wire" between the SIM drivers (inside inference_node) and
// the MuJoCo simulator process (roboparty_sim/mujoco_sim.py).
//
// It replaces the CAN bus and the IMU serial cable of the real robot.
// Keep this layout in sync with roboparty_sim/shm_layout.py.
#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace rpo_sim {

constexpr const char* kShmPath = "/dev/shm/rpo_sim";
constexpr uint32_t kMagic = 0x52504F53;  // "RPOS"
constexpr uint32_t kVersion = 1;
constexpr int kMaxMotors = 32;

struct alignas(8) Shm {
    // Header, written by the simulator.
    uint32_t magic;
    uint32_t version;
    uint64_t heartbeat;  // simulator loop counter, also counts while paused
    uint32_t num_motors;
    uint32_t reserved;

    // Commands, written by SimMotorDriver (motor id N uses index N-1).
    // MIT mode: tau = kp * (p - q) + kd * (v - dq) + tau_ff
    float cmd_p[kMaxMotors];
    float cmd_v[kMaxMotors];
    float cmd_kp[kMaxMotors];
    float cmd_kd[kMaxMotors];
    float cmd_tau[kMaxMotors];
    uint8_t enabled[kMaxMotors];

    // Motor state, written by the simulator.
    float q[kMaxMotors];
    float dq[kMaxMotors];
    float tau[kMaxMotors];

    // IMU state, written by the simulator. Quaternion is w, x, y, z.
    float imu_quat[4];
    float imu_gyro[3];
    float imu_acc[3];
};

// Same numbers as the test in roboparty_sim/test/test_shm_layout.py.
static_assert(sizeof(Shm) == 1120, "Shm layout changed: update shm_layout.py");
static_assert(offsetof(Shm, enabled) == 664, "Shm layout changed: update shm_layout.py");
static_assert(offsetof(Shm, q) == 696, "Shm layout changed: update shm_layout.py");
static_assert(offsetof(Shm, imu_quat) == 1080, "Shm layout changed: update shm_layout.py");

// Maps the shared memory once per process. Returns nullptr until the
// simulator has created and initialized the file, like a cable that is not
// connected yet.
inline volatile Shm* connect() {
    static std::mutex mutex;
    static volatile Shm* shm = nullptr;
    std::lock_guard<std::mutex> lock(mutex);
    if (shm) {
        return shm;
    }
    int fd = ::open(kShmPath, O_RDWR);
    if (fd < 0) {
        return nullptr;
    }
    struct stat st {};
    if (::fstat(fd, &st) != 0 || st.st_size < static_cast<off_t>(sizeof(Shm))) {
        ::close(fd);
        return nullptr;
    }
    void* addr = ::mmap(nullptr, sizeof(Shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (addr == MAP_FAILED) {
        return nullptr;
    }
    auto* mapped = static_cast<volatile Shm*>(addr);
    if (mapped->magic != kMagic || mapped->version != kVersion) {
        ::munmap(addr, sizeof(Shm));
        return nullptr;
    }
    shm = mapped;
    return shm;
}

// Tracks if the simulator is alive. Each call that sees no new heartbeat for
// longer than one control period counts as a missed response, like a motor
// that does not answer on the CAN bus.
class Link {
   public:
    volatile Shm* get() {
        volatile Shm* shm = connect();
        const auto now = std::chrono::steady_clock::now();
        if (!shm) {
            ++missed_;
            return nullptr;
        }
        const uint64_t beat = shm->heartbeat;
        if (beat != last_beat_) {
            last_beat_ = beat;
            last_change_ = now;
            missed_ = 0;
        } else if (now - last_change_ > std::chrono::milliseconds(20)) {
            ++missed_;
        }
        return shm;
    }
    int missed() const { return missed_; }

   private:
    uint64_t last_beat_ = 0;
    std::chrono::steady_clock::time_point last_change_{};
    int missed_ = 0;
};

}  // namespace rpo_sim
