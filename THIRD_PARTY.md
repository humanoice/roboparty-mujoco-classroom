# Third-party software and models

This repository does not copy third-party source code into git. The `Dockerfile` downloads it at
pinned commits during `docker compose build`.

| Component | Source | Revision | License | Use |
|---|---|---|---|---|
| roboparty_deploy | https://github.com/Roboparty/roboparty_deploy | `a8a0f1557cc5d085234b8bac248a8f543342f531` | GPL-3.0 | Deployment framework (pins the three below) |
| roboparty_inference | https://github.com/Roboparty/roboparty_inference | `ac448b12c382e3fab42c0e8467a6943987223c4b` | GPL-3.0 | `inference_node`, trained policies (`.onnx`), motions (`.npz`), configs. Not changed. |
| roboparty_motors | https://github.com/Roboparty/roboparty_motors | `118e569aa89ea0dff23219709320523526a6e8df` | GPL-3.0 | Motor drivers. Patched: `"SIM"` type added. |
| roboparty_imu | https://github.com/Roboparty/roboparty_imu | `7a6e6d2eeef8ff3513cee54a8fa833773efd42f3` | GPL-3.0 | IMU drivers. Patched: `"SIM"` type added. |
| rpo_description | https://github.com/Roboparty/rpo_description | `37aac9ca665e92731444a1618320078e7ba21569` | CERN-OHL-W-2.0 | MuJoCo model `mjcf/rpo.xml` and meshes. Not changed. |
| MuJoCo | https://github.com/google-deepmind/mujoco (PyPI `mujoco==3.3.7`) | 3.3.7 | Apache-2.0 | Physics simulator |
| ONNX Runtime | Bundled in roboparty_inference | 1.21.0 | MIT | Runs the policies |

The patches in `upstream-patches/` change GPL-3.0 code, so this repository uses GPL-3.0 (see `LICENSE`).

The robot-specific values in `ros2_ws/src/roboparty_sim/config/rpo_sim/robot.yaml` (gains and joint order)
come from `roboparty_inference/robots/rpo/robot.yaml`.
