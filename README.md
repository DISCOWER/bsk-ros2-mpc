# BSK ROS2 MPC Controller (BSK-ROS2-MPC)

This package integrates a Model Predictive Controller (MPC) with [Basilisk astrodynamics simulator](https://hanspeterschaub.info/basilisk/) using [BSK-ROS2-Bridge](https://github.com/Thomas-Chan-2019/srl-ros2-BSK-bridge.git).

The MPC is implemented using the [acados framework](https://github.com/acados/acados).

## Overview

This package provides MPC controllers for spacecraft control in two main configurations:

1. **Single Agent MPC**: Individual spacecraft control with position and attitude tracking
2. **Leader-Follower MPC**: Multi-agent formation control where followers maintain relative positions to a leader spacecraft

## Setup

This package depends on `acados`. Follow the official [installation guide](https://docs.acados.org/installation/) to set it up.

Clone this repo and the following dependencies into your ROS 2 workspace:

* [bsk-msgs](https://github.com/E-Krantz/bsk-msgs.git)

Build the workspace:

```bash
colcon build --packages-up-to bsk-ros2-mpc
source install/local_setup.bash
```

## Prerequisites

Before launching the MPC controllers, ensure the following components are running:

1. **Basilisk Simulation**: The astrodynamics simulation must be started first
2. **BSK-ROS2-Bridge**: The bridge connecting Basilisk to ROS2 must be active

## Running the MPC Controllers

### Single Agent MPC

For individual spacecraft control with position and attitude tracking:

```bash
ros2 launch bsk-ros2-mpc bsk_mpc.launch.py
```

### Leader-Follower Formation Control

For multi-agent formation control, launch the controllers in sequence:

1. **Start the leader controller** (moves between waypoints):
```bash
ros2 launch bsk-ros2-mpc mpc_leader.launch.py
```

2. **Start the follower controllers** (maintains relative positions to leader):
```bash
ros2 launch bsk-ros2-mpc mpc_followers.launch.py
```

The follower launch file starts MPC controllers for both follower spacecraft simultaneously. 


### Launch File Options

The launch files support various configuration options:

#### `bsk_mpc.launch.py`
* `type`: Controller type (`da` for direct allocation, `wrench` for force/torque control)
* `namespace`: ROS namespace for the spacecraft

#### `mpc_leader.launch.py`
* Configures the leader spacecraft to follow waypoint trajectories
* Handles position and attitude reference tracking

#### `mpc_followers.launch.py`
* Launches MPC controllers for follower spacecraft
* Configures relative position maintenance with respect to the leader

### Examples

**Single Agent - Basic:**
```bash
ros2 launch bsk-ros2-mpc bsk_mpc.launch.py
```

**Single Agent - Direct Allocation MPC:**
```bash
ros2 launch bsk-ros2-mpc bsk_mpc.launch.py type:=da
```

**Single Agent - Wrench MPC:**
```bash
ros2 launch bsk-ros2-mpc bsk_mpc.launch.py type:=wrench namespace:=bskSat0
```

**Formation Control - Leader:**
```bash
ros2 launch bsk-ros2-mpc mpc_leader.launch.py
```

**Formation Control - Followers:**
```bash
ros2 launch bsk-ros2-mpc mpc_followers.launch.py
```

## References

- [Basilisk Astrodynamics Simulation](https://hanspeterschaub.info/basilisk/)
- [ROS 2 Documentation](https://www.ros.org/)
- [BSK-ROS2 Bridge](https://github.com/DISCOWER/bsk-ros2-bridge.git)

## License

This project is licensed under the BSD-3-Clause License - see the [LICENSE](LICENSE) file for details.

## Authors

**Elias Krantz**  
Email: eliaskra@kth.se