# Upstream patches

The Dockerfile builds the real RoboParty software from `roboparty_deploy` at a pinned commit.
This folder adds one new hardware type, `"SIM"`, to the motor and IMU driver libraries.
The `inference_node` code is not changed.

```
 inference_node (unchanged)
   └─ RobotInterface (unchanged)
        ├─ MotorDriver::create_motor("SIM") ─┐
        └─ IMUDriver::create_imu("SIM")   ───┤  /dev/shm/rpo_sim
                                             ▼
                              roboparty_sim/mujoco_sim.py
```

| File | Goes to | What it does |
|---|---|---|
| `common/rpo_sim_shm.hpp` | `motors/src/drivers/sim/`, `imu/src/drivers/sim/` | Shared-memory layout. Same layout as `roboparty_sim/shm_layout.py`. |
| `motors/drivers/sim/sim_motor_driver.hpp` | `motors/src/drivers/sim/` | `SimMotorDriver`: writes MIT commands, reads joint state. |
| `imu/drivers/sim/sim_imu_driver.hpp` | `imu/src/drivers/sim/` | `SimIMUDriver`: reads orientation and spin. |
| `motors/0001-add-sim-motor-type.patch` | `roboparty_motors` | Adds `"SIM"` to the motor factory (2 lines). |
| `imu/0001-add-sim-imu-type.patch` | `roboparty_imu` | Adds `"SIM"` to the IMU factory (2 lines). |

## Update to a newer roboparty_deploy

1. Change `DEPLOY_REF` in the `Dockerfile`.
2. Run `docker compose build`. If `git apply` fails, the factory code changed upstream:
   make the 2-line change again and save it with `git diff > ...patch`.
3. Run `scripts/smoke_test.sh`.

## Shared-memory layout change

If you change `rpo_sim_shm.hpp`, change `shm_layout.py` too. The C++ `static_assert`s and
`roboparty_sim/test/test_shm_layout.py` check the same offsets, so a mismatch stops the build or the test.

## License

These files change GPL-3.0 code, so they are GPL-3.0 too.
