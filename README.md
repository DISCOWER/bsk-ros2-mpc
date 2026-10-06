# Basilisk-ROS 2 MPC

A Model Predictive Controller (MPC) for spacecraft position and attitude tracking with the [Basilisk astrodynamics framework](https://hanspeterschaub.info/basilisk/) via the [Basilisk-ROS 2 Bridge](https://github.com/DISCOWER/bsk-ros2-bridge), built on the [acados](https://github.com/acados/acados) optimization framework.

The controller receives spacecraft states from a running Basilisk simulation over the bridge and publishes optimal thrust commands back in real time. It supports both direct thruster allocation and wrench-level (force/torque) control, and can optionally be operated interactively through RViz.

<img src="https://github.com/user-attachments/assets/958780b9-bff1-495b-803e-ab150b822049" width="1080"/>

_Leader-follower formation flying with three spacecraft controlled via identical ROS 2-based MPC, running in Basilisk simulation (left) and on the [ATMOS](https://atmos.discower.io/) free-flyers at KTH (right). See the [paper](https://arxiv.org/abs/2512.09833) for details._


## Setup

### Prerequisites

- [Basilisk-ROS 2 Messages](https://github.com/DISCOWER/bsk-msgs)
- [acados](https://docs.acados.org/installation/)
- casadi (`pip install casadi`)

### Install

```bash
cd your_ros2_workspace/src
git clone https://github.com/DISCOWER/bsk-msgs.git
git clone https://github.com/DISCOWER/bsk-ros2-mpc.git
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to bsk-ros2-mpc
source install/setup.bash
```

## Usage

Before launching any MPC controller, ensure the **Basilisk simulation** and the **Basilisk-ROS 2 Bridge** are both running (see [bridge Quick Start](https://github.com/DISCOWER/bsk-ros2-bridge#quick-start)).

### Single-Agent

```bash
ros2 launch bsk-ros2-mpc mpc.launch.py
```

#### Single-Agent Launch Options

| Argument | Default | Description |
|---|---|---|
| `namespace` | `bskSat` | ROS 2 namespace for the agent |
| `use_sim_time` | `False` | Synchronize with `/clock` topic |
| `type` | `wrench` | `wrench` (force/torque) or `da` (direct allocation) |
| `use_hill` | `True` | Use Hill frame for MPC (when in orbit) |
| `rviz_mode` | `setpoint` | RViz mode: `setpoint`, `viz`, or `off` |
| `name_others` | `""` | Space-separated namespaces of other agents for collision avoidance |
| `skip_build` | `False` | Skip acados solver codegen/build and reuse an existing compiled solver |

Notes:

- `name_others` enables multi-agent avoidance by passing other agents' positions into MPC constraints. Leave it empty for single-agent runs.
- `skip_build:=True` is useful for faster startup when the solver has already been generated for the same controller/model configuration.

Single-agent example with collision avoidance (avoid collision with bskSat1 and bskSat2)
```bash
ros2 launch bsk-ros2-mpc mpc.launch.py namespace:=bskSat0 name_others:="bskSat1 bskSat2"
```

### Multi-Agent

```bash
ros2 launch bsk-ros2-mpc multi_mpc.launch.py agents:="bskSat0 bskSat1 bskSat2"
```

This launches one MPC per namespace with a shared visualizer and RViz window.

#### Multi-Agent Launch Options

| Argument | Default | Description |
|---|---|---|
| `agents` | required | Space-separated spacecraft namespaces |
| `use_sim_time` | `False` | Synchronize with `/clock` topic |
| `type` | `wrench` | `wrench` (force/torque) or `da` (direct allocation) |
| `use_hill` | `True` | Use Hill frame for MPC (when in orbit) |
| `rviz_mode` | `setpoint` | RViz mode: `setpoint`, `viz`, or `off` |
| `skip_build` | `False` | Skip acados solver codegen/build for subsequent agents |

### Leader-Follower

```bash
ros2 launch bsk-ros2-mpc leader_followers_mpc.launch.py
```

By default, this launches `leaderSc` with followers `followerSc_1` and `followerSc_2`.
Override the leader and follower namespaces with `leader` and `followers`:

```bash
ros2 launch bsk-ros2-mpc leader_followers_mpc.launch.py leader:=leaderSc followers:="followerSc_1 followerSc_2"
```

The launch starts the leader MPC, follower MPCs, waypoint publishers, and one shared visualizer and RViz window. Use `rviz_mode:=off` to disable the visualizer and RViz.

#### Leader-Follower Launch Options

| Argument | Default | Description |
|---|---|---|
| `leader` | `leaderSc` | Namespace of the leader spacecraft |
| `followers` | `followerSc_1 followerSc_2` | Space-separated follower namespaces |
| `use_sim_time` | `False` | Synchronize with `/clock` topic |
| `type` | `wrench` | Leader controller type: `wrench` or `da` |
| `use_hill` | `True` | Use Hill frame for MPC (when in orbit) |
| `period` | `20.0` | Time in seconds spent at each leader waypoint |
| `rviz_mode` | `viz` | RViz mode: `viz` or `off` |

## Troubleshooting

**Missing `acados` dependencies**: If the launch fails during solver generation, ensure the Python package `acados_template` is installed and that `$ACADOS_SOURCE_DIR` is correctly exported in your `.bashrc`.

**Shared library error**: If you encounter `ImportError: libacados.so: cannot open shared object file`, ensure your `$LD_LIBRARY_PATH` includes `$ACADOS_SOURCE_DIR/lib`.

**Missing message types**: If you receive errors about unknown types, ensure `bsk_msgs` and `bsk_mpc_msgs` are built and your workspace is fully sourced.

## References

- [Basilisk-ROS 2 Bridge](https://github.com/DISCOWER/bsk-ros2-bridge)
- [Basilisk-ROS 2 Messages](https://github.com/DISCOWER/bsk-msgs)
- [Basilisk Astrodynamics Simulation](https://hanspeterschaub.info/basilisk/)
- [ROS 2 Documentation](https://www.ros.org/)
- [acados](https://docs.acados.org/)
