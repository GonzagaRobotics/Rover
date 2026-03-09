# Rover

Monorepo for all core ROS2 packages that run on the Jetson Orin.

Requires Ubuntu 22.04, Python 3.10, and ROS2 Humble. Assumes CUDA is installed.

## Setup

### System Dependencies

- `ros-humble-rosbridge-suite`

### Python Dependencies

Run `pip install -r requirements.txt`.

### Setup

For serial comms, you may need to ensure your user is in the `dialout` group.

It is also recommended to add the ROS setup script to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

All commands to build and run assume ROS is sourced. Do not have any workspace sourced when building.

All scripts must be run from the root of the repo. Use `chmod u+x ./scripts/*.sh` to make them executable.

## Building

```bash
# Build all packages
./scripts/build.sh

# Build specific package(s) and any dependencies
./scripts/build.sh vision
./scripts/build.sh auto_nav pathfinder core
```

ROS generated files and artifacts can be removed with `./scripts/clean.sh`. It is a good troubleshooting step if you are having build issues.

## Running

To run a **single** node, a helper script to source the workspace and run the node is provided.

```bash
./scripts/run.sh drive
./scripts/run.sh vision --ros-args -p some_param:=value -p other_param:=value
```

Launch files provide a better way to orchestrate multiple nodes. They must be in the `launch` and have the `.launch.py` suffix.

```bash
./scripts/launch.sh all
./scripts/launch.sh teleop
./scripts/launch.sh jetson 
```