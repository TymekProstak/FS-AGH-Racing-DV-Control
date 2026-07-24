# AGH Racing DV Control

ROS 1 control stack for the AGH Racing Formula Student Driverless car. The
repository contains the controllers for trackdrive/autocross, skidpad and
acceleration together with their ROS messages, configuration and mathematical
documentation.

![Control pipeline](docs/control_pipeline.png)

## Applications

| Package | Mission | Launch file |
|---|---|---|
| `dv_control` | trackdrive and autocross | `control.launch` |
| `dv_skidpad_control` | skidpad | `skidpad_control.launch` |
| `dv_acc_launch_control` | acceleration, launch and braking | `dv_acc_launch_control.launch` |

All applications publish four wheel torques and steering angle on
`/dv_board/control`. Run only one control application at a time.

The lateral controller is the same unbounded LTV MPC in all three
applications. It uses a nonlinear bicycle model for online linearization and
a PT2 steering-actuator model closed with the physical encoder measurement.
The longitudinal reference is mission-specific:

- `dv_control` uses the forward-backward speed profile;
- `dv_skidpad_control` uses the segmented skidpad profile;
- `dv_acc_launch_control` uses the launch/brake map.

Every torque allocator uses the same relaxed wheel-load model. Its
first-order time constant is configured with
`model.mass_transfer.tau_load_s` and defaults to `0.05 s`. Skidpad enables the
four-wheel QP allocator by default.

In acceleration, lateral control becomes active only above
`lateral_control.activation_speed_mps` (`2.0 m/s` by default). Below that
speed the published `steeringAngle_rad` is exactly zero.

## Repository layout

```text
FS-AGH-Racing-DV-Control/
├── interfaces/dv_interfaces/   ROS messages used by control and simulator
├── common/                     shared wheel-load model and tests
├── dv_control/
├── dv_skidpad_control/
├── dv_acc_launch_control/
└── docs/                       equations, architecture and generated PDF
```

`interfaces/dv_interfaces` is a local catkin package containing only the
messages required by the controllers. No separate `dv_interfaces` repository
is required. `Control.msg`, `DV_board.msg` and `Imu.msg` are the wire
interface shared with
[`FS-AGH-Racing-DV-Simulator`](https://github.com/TymekProstak/FS-AGH-Racing-DV-Simulator).

## Requirements

- Ubuntu 20.04
- ROS Noetic
- `catkin_tools`
- a C++17 compiler
- Eigen 3
- nlohmann/json

Install the non-ROS build dependencies:

```bash
sudo apt update
sudo apt install python3-catkin-tools libeigen3-dev nlohmann-json3-dev
```

## Workspace setup

Clone the Control and Simulator repositories next to each other:

```bash
mkdir -p ~/agh-racing
cd ~/agh-racing

git clone https://github.com/TymekProstak/FS-AGH-Racing-DV-Simulator.git
git clone https://github.com/TymekProstak/FS-AGH-Racing-DV-Control.git
```

The resulting source layout is:

```text
~/agh-racing/
├── FS-AGH-Racing-DV-Simulator/
└── FS-AGH-Racing-DV-Control/
```

Build the Simulator as a base workspace:

```bash
mkdir -p ~/dv_sim_ws/src
ln -s ~/agh-racing/FS-AGH-Racing-DV-Simulator \
  ~/dv_sim_ws/src/FS-AGH-Racing-DV-Simulator

source /opt/ros/noetic/setup.bash
cd ~/dv_sim_ws
catkin init
rosdep install --from-paths src --ignore-src -r -y
catkin build lem_simulator
source devel/setup.bash
```

Build Control in an overlay workspace:

```bash
mkdir -p ~/dv_control_ws/src
ln -s ~/agh-racing/FS-AGH-Racing-DV-Control \
  ~/dv_control_ws/src/FS-AGH-Racing-DV-Control

source /opt/ros/noetic/setup.bash
source ~/dv_sim_ws/devel/setup.bash
cd ~/dv_control_ws
catkin init
catkin config --extend ~/dv_sim_ws/devel
rosdep install --from-paths src --ignore-src -r -y
catkin build dv_interfaces \
  dv_control dv_skidpad_control dv_acc_launch_control
source devel/setup.bash
```

The two workspaces keep message generation isolated while the Control overlay
can find `lem_simulator` and its launch files. In every new terminal load the
overlay before running a controller:

```bash
source ~/dv_control_ws/devel/setup.bash
```

## Run with the simulator

Each command starts the correct known-centerline Simulator scenario and one
control application:

```bash
# Trackdrive on FSG 2019
roslaunch dv_control simulator_fsg_2019.launch

# Skidpad
roslaunch dv_skidpad_control simulator_skidpad.launch

# Acceleration
roslaunch dv_acc_launch_control simulator_acc.launch
```

Pass a finite simulation time when required:

```bash
roslaunch dv_control simulator_fsg_2019.launch sim_time:=60
```

## Run a controller without the simulator

```bash
roslaunch dv_control control.launch
roslaunch dv_skidpad_control skidpad_control.launch
roslaunch dv_acc_launch_control dv_acc_launch_control.launch
```

A custom acceleration configuration can be selected at launch:

```bash
roslaunch dv_acc_launch_control dv_acc_launch_control.launch \
  config_path:=/absolute/path/to/params.json print_config:=true
```

## Main parameters

| Parameter | Default | Scope |
|---|---:|---|
| `model.mass_transfer.tau_load_s` | `0.05 s` | all applications |
| `model.mass_transfer.minimum_wheel_load_N` | `50 N` | all applications |
| `model.body.h1_roll` | `0.08 m` | all applications |
| `model.body.h2_roll` | `0.11 m` | all applications |
| `model.body.lambda_phi_elastic_lateral` | `0.479465...` | all applications |
| `model.ltv_mpc_unbounded.N` | `45` | all applications |
| `general.torque_allocation_lambda_factor` | `0.075` | skidpad QP |
| `general.torque_allocation_fx_ref` | `1000 N` | skidpad QP |
| `general.torque_allocation_mz_ref` | `10 N m` | skidpad QP |
| `lateral_control.activation_speed_mps` | `2.0 m/s` | acceleration |

Vehicle mass, geometry, wheel radius, gear ratio and torque limits must match
the active vehicle configuration before a test.

## Documentation and tests

The complete controller, wheel-load and QP allocator equations are available
in:

- [`docs/controllers_and_allocators.pdf`](docs/controllers_and_allocators.pdf)
- [`docs/controllers_and_allocators.tex`](docs/controllers_and_allocators.tex)

Run the standalone wheel-load test:

```bash
g++ -std=c++17 -O2 -Icommon/include \
  common/test/load_transfer_test.cpp \
  -o /tmp/load_transfer_test
/tmp/load_transfer_test
```

Build the PDF after changing its LaTeX source:

```bash
cd docs
latexmk -pdf controllers_and_allocators.tex
```

## Drive data

Store recordings and rosbags outside the Git repository and add only links:

| Mission | Recording | Rosbag / logs |
|---|---|---|
| Trackdrive / autocross | — | — |
| Skidpad | — | — |
| Acceleration | — | — |

## License

MIT
