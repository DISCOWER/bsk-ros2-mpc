# BSK ROS2 MPC Controller (BSK-ROS2-MPC)

This package integrates a Model Predictive Controller (MPC) with [Basilisk astrodynamics simulator](https://hanspeterschaub.info/basilisk/) using [BSK-ROS2-Bridge](https://github.com/Thomas-Chan-2019/srl-ros2-BSK-bridge.git).

The MPC is implemented using the [acados framework](https://github.com/acados/acados).

## Setup

This package depends on `acados`. Follow the official [installation guide](https://docs.acados.org/installation/) to set it up.

Clone this repo and the following dependencies into your ROS 2 workspace:

* [bsk-msgs](https://github.com/E-Krantz/bsk-msgs.git)

Build the workspace:

```bash
colcon build --packages-up-to bsk-ros2-mpc
source install/local_setup.bash
```

## Running the MPC 


### Launch File Options

`bsk-mpc.launch.py` supports the following arguments:

* 

### Examples
Basic:

```bash
ros2 launch bsk-ros2-mpc bsk_mpc.launch.py
```

Direct allocation MPC:

```bash
ros2 launch bsk-ros2-mpc bsk_mpc.launch.py type:=da
```

Wrench MPC:

```bash
ros2 launch bsk-ros2-mpc bsk_mpc.launch.py type:=wrench namespace:=bskSat0
```

Follower Wrench MPC:

```bash
ros2 launch bsk-ros2-mpc bsk_mpc.launch.py type:=follower_wrench namespace:=bskSat1 name_leader:=bskSat0
```
