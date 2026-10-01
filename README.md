# RoboParty MuJoCo Classroom

Drive a simulated RoboParty RPO humanoid with your keyboard. The robot walks, turns, waves and dances.

The "brain" is the **real RoboParty software** (`roboparty_inference` from
[roboparty_deploy](https://github.com/Roboparty/roboparty_deploy)), the same software that runs on the real robot.
Only the "body" is different: a MuJoCo physics simulation replaces the motors and the IMU.

Everything runs in Docker and shows in your web browser.

## Quick start

You need Docker Desktop (Mac, Windows) or Docker Engine (Linux). The image works on x86_64 and on arm64 (Apple Silicon).

```bash
docker compose up -d --build     # first build takes about 10-15 minutes
```

Open <http://localhost:6080/vnc.html?autoconnect=true&resize=scale>. You see three windows:

```
┌──────────────────────────────────┬──────────────────────┐
│                                  │   ROBOT REMOTE       │
│                                  │  [Q] [W] [E]         │
│      MuJoCo: the robot           │  [A] [S] [D]         │
│                                  │  Start Push Reset    │
│                                  │  Wave Dance Dance 2  │
│                                  ├──────────────────────┤
│                                  │  party terminal      │
│                                  │  RoboParty sim> _    │
└──────────────────────────────────┴──────────────────────┘
```

1. In the remote, click **Start**. The robot hangs on a virtual stand, moves to its start pose, then stands on its own.
2. Click the remote window, then **hold** a key:

   | Key | Action |
   |---|---|
   | W / S | walk forward / back |
   | A / D | turn left / right |
   | Q / E | step left / right |
   | Space | stop |

3. Click **Wave** or **Dance**. Click **Push** to push the robot. If it falls, click **Reset**.

In the MuJoCo window: left drag = rotate the camera, right drag = move, scroll = zoom.
Keys there: **H** = stand on/off, **P** = push, **Space** = pause.

## party commands

Type these in the party terminal (or in `docker compose exec sim bash`):

| Command | What it does |
|---|---|
| `party start` | motors on, start pose, brain on, off the stand |
| `party walk [speed] [seconds]` | walk forward (default 0.4 m/s for 3 s, max 0.6) |
| `party back [speed] [seconds]` | walk backward |
| `party turn left\|right [seconds]` | turn on the spot |
| `party side left\|right [seconds]` | step sideways |
| `party stop` | stand still |
| `party wave` | wave hello |
| `party dance [0\|1]` | dance |
| `party push` | push the robot |
| `party status` | where is the robot? did it fall? |
| `party reset` | put the robot back at the start and start again |
| `party sleep` | brain and motors off |

Each command prints the ROS topic or service that it uses. Read `ros2_ws/src/roboparty_sim/roboparty_sim/party.py`
to see the code. It is short.

## How it works

### The control loop

A walking robot does not follow a list of steps. A neural network (the **policy**) looks at the robot 50 times
each second and decides the next target angle for each of the 23 joints. RoboParty trained the policy in a
simulator (Isaac Lab) with thousands of robots over millions of tries.

```
            50 times per second                      1000 times per second
 ┌──────────────────────────────┐            ┌────────────────────────────┐
 │  POLICY (the "brain")         │  target    │  MOTOR (PD controller)     │
 │  neural network, .onnx file   │──angles──► │  torque = kp*(target-q)    │
 │                               │            │         + kd*(0 - dq)      │
 │  inputs:                      │            └─────────────┬──────────────┘
 │   - command (vx, vy, turn)  ◄─┼── /cmd_vel (keyboard!)   │ torque
 │   - body tilt + spin (IMU)    │                          ▼
 │   - joint angles + speeds     │◄──── sensors ───── BODY (MuJoCo physics)
 │   - last action               │
 └──────────────────────────────┘
```

- **kp** (stiffness) and **kd** (damping) make each motor act like a spring and a shock absorber.
- **IMU**: a sensor that measures how the body is tilted and how fast it spins.

### Real robot vs. this simulator

```
 REAL ROBOT                                   THIS CLASSROOM
 inference_node   (same code)                 inference_node   (same code)
   ├─ motor driver "DM" ── CAN bus ── motors    ├─ motor driver "SIM" ─┐
   └─ IMU driver "HIPNUC" ─ serial ── IMU       └─ IMU driver "SIM" ───┤ shared memory
                                                                       ▼
                                                        mujoco_sim: physics + motor law
```

The only file that is different is `config/rpo_sim/robot.yaml`. Compare it with the real one:

```bash
diff /opt/roboparty_deploy/install/roboparty_inference/share/roboparty_inference/robots/rpo/robot.yaml \
     /opt/ros2_ws/install/roboparty_sim/share/roboparty_sim/config/rpo_sim/robot.yaml
```

### Start sequence

The simulator does the same steps as the real robot:

| Step | Real robot | ROS service |
|---|---|---|
| 1 | Hang the robot on a stand | `/sim/hold` (virtual stand) |
| 2 | Turn on the motors | `/init_motors` |
| 3 | Move slowly to the start pose (4 s) | `/reset_joints` |
| 4 | Start the brain | `/start_inference` |
| 5 | Lower the robot to the floor | `/sim/hold` false |

## Exercises

1. **Topics.** Run `ros2 topic list`. Which node publishes `/joint_states`? (Use `ros2 topic info /joint_states --verbose`.)
2. **Drive by hand.** Send one velocity command:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
   ```
   Why does the robot continue to walk? How do you stop it?
3. **Two publishers.** Run `party walk 0.3 10` and hold **A** in the remote at the same time. What happens? Why?
4. **Services.** Run `ros2 service call /stop_inference std_srvs/srv/Trigger`, then push the robot. What happens? Why?
5. **Logs are a topic.** Run `ros2 topic echo /rosout`, then click **Wave**. `party` reads these messages to see
   which dance is selected.
6. **The brain's view.** Run `ros2 topic echo /imu --once`. Tilt is in `orientation`. Spin is in `angular_velocity`.
7. **Another brain.** Change `RPO_POLICY` in `compose.yaml` to `amp` and run `docker compose up -d`. Does it walk differently?

## Troubleshooting

| Problem | Fix |
|---|---|
| The robot falls when it walks | Run `docker compose exec sim /opt/ros2_ws/scripts/smoke_test.sh`. If the real-time factor is below 0.9, the computer is too slow. Close other programs. |
| Keys do nothing | Click the remote window first. Then click **Start**. |
| `service ... is not available` | Wait 10 s after `docker compose up`. Check `docker compose exec sim tail -50 /tmp/robot.log`. |
| `Physics paused for ... ms` in `/tmp/robot.log` | The computer is busy. A few short pauses (< 50 ms) are OK. Many long pauses can make the robot fall: close other programs (music and video apps use a lot of CPU). |
| `Failed to set realtime priority` in `/tmp/robot.log` | `compose.yaml` must have `cap_add: [SYS_NICE]` and `ulimits: rtprio: 99`. |
| The robot slides or acts strangely | `party reset` |

## For teachers

- `scripts/smoke_test.sh` checks the full chain: services, real-time speed, start, walk 5 s, did not fall.
- `upstream-patches/README.md` explains the SIM drivers and how to update to a newer `roboparty_deploy`.
- The build compiles with `-march=native` (upstream setting). Build the image on the computer that runs it.
- Your edits in `ros2_ws/src` are live in the container (bind mount). Restart with `docker compose restart`.
- `ROS_DOMAIN_ID` is 43, so this classroom does not mix with the RViz classroom (42) on one computer.
- Docker has no GPU, so the CPU draws the 3D view. To keep the fan quiet, the desktop is 1280x720 (720p), the
  MuJoCo window draws 15 frames per second, and shadows are off. To change it, edit the `mujoco_sim`
  parameters `viewer_fps`, `viewer_width` and `viewer_height` in `launch/sim.launch.py`, or set `viewer:=false`.
- Close the browser tab when nobody watches. The robot and `party` still work, and VNC uses less CPU.

## License

GPL-3.0, because the SIM drivers change GPL-3.0 code from RoboParty. See `LICENSE` and `THIRD_PARTY.md`.
