# BSK Controller

This package integrates a Model Predictive Controller (MPC) with [Basilisk astrodynamics simulator](https://hanspeterschaub.info/basilisk/) using [BSK-ROS2-Bridge](https://github.com/Thomas-Chan-2019/srl-ros2-BSK-bridge.git).

The MPC is implemented using the [acados framework](https://github.com/acados/acados).

## Citation

If you use this package in academic work, please cite the following paper:

TODO

## Setup

This package depends on `acados`. Follow the official [installation guide](https://docs.acados.org/installation/) to set it up.

Clone this repo and the following dependencies into your ROS 2 workspace:

* [bsk\_msgs](https://github.com/E-Krantz/bsk_msgs.git)

Build the workspace:

```bash
colcon build --packages-up-to bsk_controller
source install/local_setup.bash
```

## Running the MPC 


### Launch File Options

`bsk_mpc.launch.py` supports the following arguments:

* 

### Examples
Basic:

```bash
ros2 launch bsk_controller bsk_mpc.launch.py
```

Direct allocation MPC:

```bash
ros2 launch bsk_controller bsk_mpc.launch.py type:=da
```

Wrench MPC:

```bash
ros2 launch bsk_controller bsk_mpc.launch.py type:=wrench
```
