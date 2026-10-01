// SPDX-License-Identifier: GPL-3.0
// Simulated motor for roboparty-mujoco-classroom.
//
// A real DM motor gets MIT commands over CAN and runs this law on its own chip:
//     tau = kp * (p - q) + kd * (v - dq) + tau_ff
// This driver writes the same five numbers into shared memory. The MuJoCo
// simulator runs the law at 1 kHz and writes back the joint state.
#pragma once

#include <stdexcept>
#include <string>

#include "motor_driver.hpp"
#include "rpo_sim_shm.hpp"

class SimMotorDriver : public MotorDriver {
   public:
    SimMotorDriver(uint16_t motor_id, const std::string& interface) {
        if (motor_id < 1 || motor_id > rpo_sim::kMaxMotors) {
            throw std::runtime_error("SIM motor id must be 1.." + std::to_string(rpo_sim::kMaxMotors));
        }
        motor_id_ = motor_id;
        idx_ = motor_id - 1;
        can_interface_ = interface;
        comm_type_ = CommType::CAN;
    }

    void lock_motor() override { set_enabled(true); }
    void unlock_motor() override { set_enabled(false); }
    uint8_t init_motor() override {
        set_enabled(true);
        motor_control_mode_ = MIT;
        return 0;
    }
    void deinit_motor() override { set_enabled(false); }
    bool set_motor_zero() override { return true; }
    bool write_motor_flash() override { return true; }
    void get_motor_param(uint8_t) override {}

    void motor_pos_cmd(float pos, float, bool) override { motor_mit_cmd(pos, 0.f, 20.f, 1.f, 0.f); }
    void motor_spd_cmd(float spd) override { motor_mit_cmd(0.f, spd, 0.f, 1.f, 0.f); }
    void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override {
        volatile rpo_sim::Shm* shm = link_.get();
        if (!shm) {
            return;
        }
        shm->cmd_p[idx_] = f_p;
        shm->cmd_v[idx_] = f_v;
        shm->cmd_kp[idx_] = f_kp;
        shm->cmd_kd[idx_] = f_kd;
        shm->cmd_tau[idx_] = f_t;
    }
    void motor_mit_cmd(float*, float*, float*, float*, float*) override {}
    void set_motor_control_mode(uint8_t mode) override { motor_control_mode_ = mode; }

    int get_response_count() const override { return link_.missed(); }
    void refresh_motor_status() override { link_.get(); }
    void clear_motor_error() override { error_id_ = 0; }
    void set_motor_id(uint8_t, uint8_t) override {}
    void reset_motor_id() override {}

    float get_motor_pos() override { return read(&rpo_sim::Shm::q); }
    float get_motor_spd() override { return read(&rpo_sim::Shm::dq); }
    float get_motor_current() override { return read(&rpo_sim::Shm::tau); }

   private:
    int idx_ = 0;
    rpo_sim::Link link_;

    void set_enabled(bool on) {
        volatile rpo_sim::Shm* shm = link_.get();
        if (!shm) {
            return;
        }
        if (on) {
            // Hold the current pose with zero gains until the first real command.
            shm->cmd_kp[idx_] = 0.f;
            shm->cmd_kd[idx_] = 0.f;
            shm->cmd_tau[idx_] = 0.f;
        }
        shm->enabled[idx_] = on ? 1 : 0;
    }

    float read(float (rpo_sim::Shm::*field)[rpo_sim::kMaxMotors]) {
        volatile rpo_sim::Shm* shm = link_.get();
        if (!shm) {
            return 0.f;
        }
        return (shm->*field)[idx_];
    }
};
