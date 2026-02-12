# Basilisk-ROS 2 MPC

A Model Predictive Controller (MPC) for spacecraft position and attitude tracking with the [Basilisk astrodynamics framework](https://hanspeterschaub.info/basilisk/) via the [Basilisk-ROS 2 Bridge](https://github.com/DISCOWER/bsk-ros2-bridge), built on the [acados](https://github.com/acados/acados) optimization framework.

The controller receives spacecraft states from a running Basilisk simulation over the bridge and publishes optimal thrust commands back in real time. It supports both direct thruster allocation and wrench-level (force/torque) control, and can optionally be operated interactively through RViz.

## Setup

### Prerequisites

- [Basilisk-ROS 2 Bridge](https://github.com/DISCOWER/bsk-ros2-bridge) (installed and sourced)
- [Basilisk-ROS 2 Messages](https://github.com/DISCOWER/bsk-msgs)
- [Acados](https://docs.acados.org/installation/)

### Install

```bash
cd your_ros2_workspace/src
git clone https://github.com/DISCOWER/bsk-ros2-mpc.git
cd ..
colcon build --packages-select bsk-ros2-mpc
source install/setup.bash
```

## Usage

Before launching any MPC controller, ensure the **Basilisk simulation** and the **Basilisk-ROS 2 Bridge** are both running (see [bridge Quick Start](https://github.com/DISCOWER/bsk-ros2-bridge#quick-start)).

### Single Agent

```bash
ros2 launch bsk-ros2-mpc mpc.launch.py
```

To use RViz visualization and interactive control:

```bash
ros2 launch bsk-ros2-mpc mpc.launch.py use_rviz:=True
```

### Launch Options

| Argument | Default | Description |
|---|---|---|
| `namespace` | - | ROS 2 namespace for the agent |
| `use_sim_time` | `False` | Synchronize with `/clock` topic |
| `type` | `wrench` | `wrench` (force/torque) or `da` (direct allocation) |
| `use_hill` | `True` | Use Hill frame for MPC (when in orbit) |
| `use_rviz` | `False` | Launch RViz for visualization and interactive control |

**Example:**

```bash
ros2 launch bsk-ros2-mpc mpc.launch.py namespace:=bskSat0 type:=wrench use_rviz:=True use_hill:=True
```

## References

- [Basilisk Astrodynamics Simulation](https://hanspeterschaub.info/basilisk/)
- [ROS 2 Documentation](https://www.ros.org/)
- [acados](https://docs.acados.org/)
- [Basilisk-ROS 2 Bridge](https://github.com/DISCOWER/bsk-ros2-bridge)
- [Basilisk-ROS 2 Messages](https://github.com/DISCOWER/bsk-msgs)