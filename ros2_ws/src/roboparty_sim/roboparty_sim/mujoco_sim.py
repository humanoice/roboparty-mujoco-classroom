# SPDX-License-Identifier: GPL-3.0
"""MuJoCo "virtual body" for the RoboParty RPO.

The real inference_node sends motor commands through the SIM drivers into
shared memory. This node reads them, runs the motor law and the physics at
1 kHz, and writes back joint angles and IMU values. It also has a virtual
stand (an elastic band) that holds the robot up, like a real test gantry.
"""
import math
import os
import threading
import time

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, Trigger

from roboparty_sim import shm_layout

# Default standing pose in "URDF order" (from robots/rpo/configs/default.yaml).
DEFAULT_POSE = np.array([
    0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
    0.0, 0.0, -0.1, 0.3, -0.2, 0.0, 0.0,
    0.18, 0.06, 0.0, 0.78, 0.0,
    0.18, -0.06, 0.0, 0.78, 0.0])


class Floats:
    """numpy view of one float array in shared memory (no copy)."""

    def __init__(self, buf, field, count):
        self.array = np.frombuffer(buf, dtype=np.float32, count=count, offset=field.offset)


class MujocoSim(Node):

    def __init__(self):
        super().__init__('mujoco_sim')
        self.declare_parameter('mjcf', os.environ.get('RPO_MJCF', '/opt/rpo_description/mjcf/rpo.xml'))
        self.declare_parameter('shm_path', shm_layout.SHM_PATH)
        self.declare_parameter('viewer', True)
        # In the default pose the feet touch the floor at a base height of 0.747 m.
        self.declare_parameter('stand_height', 0.76)

        self.model = mujoco.MjModel.from_xml_path(self.get_parameter('mjcf').value)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep
        self.nu = self.model.nu
        # Actuator i drives motor id i+1. The MJCF actuator order is the URDF order.
        joint_ids = self.model.actuator_trnid[:, 0]
        self.qpos_adr = self.model.jnt_qposadr[joint_ids]
        self.dof_adr = self.model.jnt_dofadr[joint_ids]
        self.ctrl_range = self.model.actuator_ctrlrange.copy()
        self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        self.weight = float(self.model.body_mass.sum()) * -self.model.opt.gravity[2]
        self.sensor = {name: self._sensor_slice(name) for name in
                       ('orientation', 'angular-velocity', 'linear-acceleration')}

        self.shm, self.buf = shm_layout.create(self.nu, self.get_parameter('shm_path').value)
        S = shm_layout.Shm
        n = self.nu
        self.cmd_p = Floats(self.buf, S.cmd_p, n).array
        self.cmd_v = Floats(self.buf, S.cmd_v, n).array
        self.cmd_kp = Floats(self.buf, S.cmd_kp, n).array
        self.cmd_kd = Floats(self.buf, S.cmd_kd, n).array
        self.cmd_tau = Floats(self.buf, S.cmd_tau, n).array
        self.enabled = np.frombuffer(self.buf, dtype=np.uint8, count=n, offset=S.enabled.offset)
        self.out_q = Floats(self.buf, S.q, n).array
        self.out_dq = Floats(self.buf, S.dq, n).array
        self.out_tau = Floats(self.buf, S.tau, n).array
        self.out_quat = Floats(self.buf, S.imu_quat, 4).array
        self.out_gyro = Floats(self.buf, S.imu_gyro, 3).array
        self.out_acc = Floats(self.buf, S.imu_acc, 3).array

        self.lock = threading.Lock()
        self.stand_height = self.get_parameter('stand_height').value
        self.hold = True
        self.anchor = np.array([0.0, 0.0, self.stand_height])
        self.push_force = np.zeros(3)
        self.paused = False
        self.reset_world()

        self.create_service(SetBool, '/sim/hold', self.on_hold)
        self.create_service(Trigger, '/sim/reset', self.on_reset)
        self.create_service(Trigger, '/sim/push', self.on_push)
        self.pose_pub = self.create_publisher(PoseStamped, '/sim/base_pose', 10)
        self.rtf_pub = self.create_publisher(Float32, '/sim/real_time_factor', 10)
        self.real_time_factor = 1.0
        self.create_timer(1.0 / 30.0, self.publish_pose)
        self.create_timer(1.0, lambda: self.rtf_pub.publish(Float32(data=float(self.real_time_factor))))
        self.get_logger().info(
            f'MuJoCo ready: {self.nu} motors, dt={self.dt * 1000:.1f} ms, shared memory {shm_layout.SHM_PATH}')

    def _sensor_slice(self, name):
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        adr = self.model.sensor_adr[sid]
        return slice(adr, adr + self.model.sensor_dim[sid])

    # ----- services -------------------------------------------------------

    def on_hold(self, request, response):
        with self.lock:
            self.hold = request.data
            if self.hold:
                pos = self.data.xpos[self.base_id]
                self.anchor = np.array([pos[0], pos[1], self.stand_height])
        response.success = True
        response.message = 'Robot is on the stand' if request.data else 'Robot is free'
        return response

    def on_reset(self, request, response):
        with self.lock:
            self.reset_world()
        response.success = True
        response.message = 'Robot is back at the start, on the stand'
        return response

    def on_push(self, request, response):
        angle = np.random.uniform(0, 2 * math.pi)
        with self.lock:
            self.push_force = 60.0 * np.array([math.cos(angle), math.sin(angle), 0.0])
            self.push_until = self.data.time + 0.2
        response.success = True
        response.message = 'Push!'
        return response

    def reset_world(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = [0.0, 0.0, self.stand_height]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qpos[self.qpos_adr] = DEFAULT_POSE[:self.nu]
        self.hold = True
        self.anchor = np.array([0.0, 0.0, self.stand_height])
        self.push_until = 0.0  # mj_resetData sets the time back to 0
        mujoco.mj_forward(self.model, self.data)
        self.write_state()

    # ----- physics --------------------------------------------------------

    def apply_motor_law(self):
        q = self.data.qpos[self.qpos_adr]
        dq = self.data.qvel[self.dof_adr]
        tau = (self.cmd_kp * (self.cmd_p - q) + self.cmd_kd * (self.cmd_v - dq) + self.cmd_tau)
        tau *= self.enabled  # a disabled motor gives no torque: the joint is limp
        np.clip(tau, self.ctrl_range[:, 0], self.ctrl_range[:, 1], out=tau)
        self.data.ctrl[:] = tau

    def apply_stand_and_push(self):
        force = np.zeros(6)
        if self.hold:
            # Elastic band: a spring + damper on the base position and a soft
            # torque that keeps the body upright, like a rope on a gantry.
            pos = self.data.qpos[0:3]
            vel = self.data.qvel[0:3]
            force[0:3] = 1500.0 * (self.anchor - pos) - 150.0 * vel
            force[2] += self.weight  # the band carries the weight, so it does not sag
            quat = self.data.qpos[3:7]
            up_err = np.zeros(3)
            mujoco.mju_subQuat(up_err, np.array([1.0, 0.0, 0.0, 0.0]), quat)
            # up_err is in the body frame; rotate to world and remove yaw.
            rot = np.zeros(9)
            mujoco.mju_quat2Mat(rot, quat)
            up_err_world = rot.reshape(3, 3) @ up_err
            up_err_world[2] = 0.0
            force[3:6] = 200.0 * up_err_world - 20.0 * self.data.qvel[3:6]
        if self.data.time < self.push_until:
            force[0:3] += self.push_force
        self.data.xfrc_applied[self.base_id] = force

    def write_state(self):
        self.out_q[:] = self.data.qpos[self.qpos_adr]
        self.out_dq[:] = self.data.qvel[self.dof_adr]
        self.out_tau[:] = self.data.actuator_force
        self.out_quat[:] = self.data.sensordata[self.sensor['orientation']]
        self.out_gyro[:] = self.data.sensordata[self.sensor['angular-velocity']]
        self.out_acc[:] = self.data.sensordata[self.sensor['linear-acceleration']]

    def step(self):
        with self.lock:
            self.apply_motor_law()
            self.apply_stand_and_push()
            mujoco.mj_step(self.model, self.data)
            self.write_state()

    def run(self, viewer=None):
        """Run physics in real time. Call from the main thread."""
        wall_start = time.perf_counter()
        sim_start = self.data.time
        last_sync = 0.0
        rtf_wall, rtf_sim = wall_start, sim_start
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            self.shm.heartbeat += 1  # "I am alive", also while paused
            now = time.perf_counter()
            if self.paused:
                time.sleep(0.005)
                wall_start, sim_start = time.perf_counter(), self.data.time
                continue
            target = sim_start + (now - wall_start)
            if target - self.data.time > 0.1:
                # Too slow to keep up. Drop the lost time, do not try to catch up.
                sim_start, wall_start = self.data.time, now
                target = self.data.time
            while self.data.time < target:
                self.step()
            if now - rtf_wall >= 1.0:
                self.real_time_factor = (self.data.time - rtf_sim) / (now - rtf_wall)
                rtf_wall, rtf_sim = now, self.data.time
            if viewer is not None and now - last_sync > 1.0 / 30.0:
                with self.lock:
                    viewer.sync()
                last_sync = now
            sleep = (self.data.time - sim_start) - (time.perf_counter() - wall_start)
            if sleep > 0:
                time.sleep(sleep)

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        with self.lock:
            p = self.data.qpos[0:3].copy()
            q = self.data.qpos[3:7].copy()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
        (msg.pose.orientation.w, msg.pose.orientation.x,
         msg.pose.orientation.y, msg.pose.orientation.z) = map(float, q)
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    sim = MujocoSim()
    spinner = threading.Thread(target=rclpy.spin, args=(sim,), daemon=True)
    spinner.start()
    try:
        if sim.get_parameter('viewer').value:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(sim.model, sim.data, show_left_ui=False,
                                              show_right_ui=False,
                                              key_callback=lambda key: on_key(sim, key)) as viewer:
                viewer.cam.distance = 3.0
                viewer.cam.elevation = -15.0
                viewer.cam.azimuth = 135.0
                viewer.cam.trackbodyid = sim.base_id
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                sim.run(viewer)
        else:
            sim.run()
    except KeyboardInterrupt:
        pass
    finally:
        sim.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def on_key(sim, key):
    # Keys in the MuJoCo window. Space is already "pause" in the viewer.
    if key == 32:
        sim.paused = not sim.paused
    elif key == ord('H'):
        with sim.lock:
            sim.hold = not sim.hold
            sim.anchor = np.array([sim.data.qpos[0], sim.data.qpos[1], sim.stand_height])
    elif key == ord('P'):
        sim.on_push(None, Trigger.Response())


if __name__ == '__main__':
    main()
