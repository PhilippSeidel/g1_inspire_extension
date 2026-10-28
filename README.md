# Unitree G1 + Inspire hand extension for INCAR

INCAR robot interface that teleoperates one arm of the G1 ([unitree_g1_extension](https://github.com/PhilippSeidel/unitree_g1_extension)) and an Inspire hand ([inspire_extension](https://github.com/INCAR-Robotics/inspire_extension)) at the same time, by splitting the command hooks between the two. Just uses the gripper open/close signal for now instead of the full 6DoF hand teleoperation.

## Install

Requires an INCAR venv with `unitree_g1_extension` and `inspire_extension` installed. 


## Start

Set up the robot side as described in the `unitree_g1_extension` README, then source the venv, start the INCAR app as usual and start the interface:

```bash
python -m g1_inspire_extension.robot_interface.run --robot sim
python -m g1_inspire_extension.robot_interface.run --robot real
```

| Option | Values | Default | Description |
|---|---|---|---|
| `--robot` | `sim`, `real` | `sim` | Simulation or real robot |
| `--arm` | `right`, `left` | `right` | Arm to teleoperate |
| `--no-viewer` | flag | off | Simulation only: don't open the MuJoCo window, publish the sim cameras over ZMQ instead |
| `--hand-port` | path | `/dev/ttyUSB0` | Serial port of the Inspire hand |

If the hand can't be opened (e.g. no hand plugged in when testing in simulation), the hand process prints the error and stops; the arm keeps working.

## Behavior

- **Arm**: unchanged from `unitree_g1_extension`, driven by `right.commands.arm.ee.velocity`.
- **Hand**: driven by `right.commands.gripper.openclose`, the analog trigger of the right controller (0 released, 1 pressed). The hand fully closes when the trigger passes 0.75 and fully opens when it drops below 0.25. It only receives the command while the right side is engaged and holds its pose otherwise. It opens once the G1 is connected.
- The serial I/O of the hand runs in a separate process, so it doesn't stall the 10 ms control loop.
- The state is published every 10 ms: the arm as `right_arm` / `left_arm` (see `unitree_g1_extension`), the hand as `right_hand` / `left_hand` with the raw `angleAct` (0 closed – 1000 open) as `joint_pos` and `forceAct` as `joint_effort`.

No `command_processing` step is needed in the workspace config: INCAR sends `right.commands.gripper.openclose` by itself.
