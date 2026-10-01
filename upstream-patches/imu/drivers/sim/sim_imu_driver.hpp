// SPDX-License-Identifier: GPL-3.0
// Simulated IMU for roboparty-mujoco-classroom.
//
// The real IMU sends orientation and spin over a serial cable. This driver
// reads the same values from the MuJoCo "imu" site through shared memory.
#pragma once

#include <string>
#include <vector>

#include "imu_driver.hpp"
#include "rpo_sim_shm.hpp"

class SimIMUDriver : public IMUDriver {
   public:
    explicit SimIMUDriver(uint16_t imu_id) { imu_id_ = imu_id; }

    std::vector<float> get_quat() override {
        volatile rpo_sim::Shm* shm = rpo_sim::connect();
        if (!shm) {
            return {1.f, 0.f, 0.f, 0.f};
        }
        return {shm->imu_quat[0], shm->imu_quat[1], shm->imu_quat[2], shm->imu_quat[3]};
    }
    std::vector<float> get_ang_vel() override { return read3(&rpo_sim::Shm::imu_gyro); }
    std::vector<float> get_lin_acc() override { return read3(&rpo_sim::Shm::imu_acc); }

   private:
    std::vector<float> read3(float (rpo_sim::Shm::*field)[3]) {
        volatile rpo_sim::Shm* shm = rpo_sim::connect();
        if (!shm) {
            return {0.f, 0.f, 0.f};
        }
        return {(shm->*field)[0], (shm->*field)[1], (shm->*field)[2]};
    }
};
