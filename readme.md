# gopigo_robot

This repository exists to do two things:

1. **Make a real [GoPiGo3](https://gopigo.io) work with ROS 2** — an `rclcpp` node
   subscribes to the same `/turtle1/cmd_vel` topic as `turtlesim` and turns each `Twist`
   into wheel speeds, so `turtle_teleop_key` on a laptop drives both the turtle and the
   robot.
2. **Show how to install ROS through Conan**, using
   [`ros-conan`](https://github.com/conan-io/ros-conan) (`ros-kilted`). There is no
   `apt install ros-…`, no `rosdep`, and no ROS distro on the Raspberry Pi: the runtime
   is a Conan package, pulled the same way as the GoPiGo3 C++ driver.

This repository is a [Conan workspace](https://docs.conan.io/2/tutorial/developing_packages/workspaces.html):
the driver and the ROS node are workspace packages (editables), built in dependency order
with `conan workspace build`. There is no `colcon` and no `ament_cmake`. Workspaces are
experimental; see the [Conan stability notes](https://docs.conan.io/2/introduction.html#stability).

| Directory                     | Contents                                                        |
| ----------------------------- | ---------------------------------------------------------------- |
| [`gopigo3/`](gopigo3)         | Conan recipe packaging the upstream GoPiGo3 C++ driver.          |
| [`gopigo3-ros/`](gopigo3-ros) | The ROS 2 node: translates `Twist` into robot motion.            |
| [`profiles/`](profiles)       | `ros` (native) and `rpi3-armv8` (cross to a Raspberry Pi 3).     |
| [`docker/`](docker)           | The build environment the Pi walkthrough runs inside.            |

```text
.
├── conanws.yml              # workspace root + package inventory
├── gopigo3/
│   ├── conanfile.py         # downloads DexterInd/GoPiGo3 and builds libgopigo3
│   └── test_package/
└── gopigo3-ros/
    ├── conanfile.py         # requires gopigo3 (workspace) and ros-kilted
    ├── CMakeLists.txt
    └── src/
```

## Prerequisites

- Python 3.12 and Conan 2.31+ (workspace `build` has had recent fixes).
- A C++17 compiler and CMake ≥ 3.22 (`profiles/ros` tool-requires `cmake/3.29.3`).
- A Conan remote that can resolve [`ros-kilted`](https://github.com/conan-io/ros-conan):

  ```bash
  git clone https://github.com/conan-io/ros-conan.git
  conan remote add ros-conan ./ros-conan --type=local-recipes-index
  conan profile detect --force
  ```

  The first `ros-kilted` build from source takes hours. Later builds reuse the Conan cache.

The driver talks to `/dev/spidev` through `<linux/spi/spidev.h>`, so **packages in this
workspace only build on Linux**.

## Native workspace build

From the repository root:

```bash
conan workspace source
conan workspace build --profile profiles/ros --build=missing
```

`gopigo3` is never looked up as a binary: it is in the workspace. `ros-kilted` and the rest
of the graph come from remotes/cache.

The node lands at `gopigo3-ros/build/Release/gopigo3_ros_node`. It will not talk to hardware
on a desktop; that is expected.

## Cross-build and deploy to the Raspberry Pi

A Raspberry Pi 3 Model B has 1 GB of RAM, so ROS 2 is built elsewhere and copied over; the Pi
ends up needing neither Conan nor a compiler. All you need is Docker — the build environment
comes from [`docker/Dockerfile`](docker/Dockerfile) — and a Pi running 64-bit Raspberry Pi OS.

### 1. Start the build container

From the repository root:

```bash
docker build -t gopigo-builder docker

docker run -it --rm -v "$PWD:/repo" -v gopigo-conan:/opt/conan2 gopigo-builder
```

The repository is shared at `/repo`, where the shell starts, and the `gopigo-conan` volume
keeps the Conan cache after `--rm` removes the container, so the hours step 2 costs are paid
once. **The remaining steps run at this prompt.**

### 2. Point Conan at `ros-kilted` and cross-build it for arm64

```bash
git clone --depth 1 https://github.com/conan-io/ros-conan.git /opt/ros-conan
conan remote add ros-conan /opt/ros-conan --type=local-recipes-index --force

conan install --requires=ros-kilted/2026.06.17 -pr:b=default -pr:h=profiles/rpi3-armv8 --build=missing
```

Hours of compiling, once. The default `core` variant carries `rclcpp` and `geometry_msgs`,
which is all the node needs.

### 3. Cross-build the workspace packages into the cache, then deploy

Workspace packages are editables. `full_deploy` copies **cache** packages, so create them
first, then install the node from a directory that is *not* this repository (otherwise Conan
walks up to `conanws.yml` and keeps `gopigo3` editable):

```bash
conan create gopigo3 -pr:b=profiles/ros -pr:h=profiles/rpi3-armv8 --build=missing

cd /tmp
conan install /repo/gopigo3-ros -pr:b=/repo/profiles/ros -pr:h=/repo/profiles/rpi3-armv8 \
    --build=never --deployer=full_deploy --deployer-folder=/repo/gopigo3-ros/deploy

cd /repo/gopigo3-ros
cmake --preset conan-release
cmake --build --preset conan-release
```

Seconds, not hours, after step 2. `--build=never` makes a profile mismatch fail loudly instead
of quietly trying to compile ROS for ARM. `full_deploy` gathers every dependency into
`deploy/`, around 370 MB, which is what frees the Pi from needing Conan.

### 4. Copy it to the robot

Still in `gopigo3-ros`. Check the robot answers before moving 370 MB:

```bash
ssh raspi@gopigo3.local true && echo reachable
```

The container reaches the LAN through Docker's NAT, but it has no mDNS resolver, so use the
robot's IP if `gopigo3.local` does not resolve. Then transfer:

```bash
rsync -az --relative --info=progress2 build/Release/generators build/Release/gopigo3_ros_node deploy relocate_env.sh raspi@gopigo3.local:~/gopigo3-ros/
```

`--relative` is required: the environment scripts reach the libraries through
`../../../deploy/...`, so `build/Release/generators` has to land three levels below `deploy/`,
exactly as it sits here. Later transfers sync only what changed.

### 5. Run it on the robot

```bash
ssh raspi@gopigo3.local
cd ~/gopigo3-ros
bash relocate_env.sh                        # once per transfer
. build/Release/generators/conanrun.sh
./build/Release/gopigo3_ros_node
```

[`relocate_env.sh`](gopigo3-ros/relocate_env.sh) rewrites the absolute path Conan bakes into
the first line of each environment script, which otherwise still points inside the container.
With the robot switched off, a healthy deployment gets through ROS start-up and then fails on
the hardware:

```text
[FATAL] [gopigo3_ros]: Cannot talk to the GoPiGo3 board (spi_setup error). Check that SPI is
enabled (raspi-config), that the robot is powered on, and that you can read /dev/spidev0.1.
```

Anything earlier, such as a missing `.so`, means the tree did not arrive intact.

## Drive it

The node needs two things on the Pi, both from the upstream GoPiGo3 install:

- **SPI enabled** (`sudo raspi-config` → *Interface Options* → *SPI*) and read/write access to
  `/dev/spidev0.1`. Adding your user to the `spi` group avoids running as root.
- **The `gopigo3_power` service.** It holds GPIO 23 to tell the board the Pi is up, and per the
  [upstream FAQ](https://github.com/DexterInd/GoPiGo3/blob/master/Installation_FAQ.md) motor
  control will not work without it, whatever drives the SPI bus. Check it with
  `sudo systemctl status gopigo3_power`; a power LED that keeps blinking means it is not
  running.

With the node running, start the turtle and the keyboard teleop on your laptop, which needs
`ros-kilted` with the `desktop` variant:

```bash
export ROS_DOMAIN_ID=42   # the same value on both machines, or DDS will not pair them
conan run "ros2 run turtlesim turtlesim_node"
conan run "ros2 run turtlesim turtle_teleop_key --ros-args -p scale_linear:=0.2 -p scale_angular:=1.0"
```

Arrow keys now move both the turtle and the robot. `turtle_teleop_key` defaults to 2 m/s, far
beyond what a GoPiGo3 can do; the scales above keep the two roughly in sync, and the node
clamps anything above `max_linear_speed` regardless.

### Parameters

| Parameter           | Default             | Description                                              |
| ------------------- | ------------------- | -------------------------------------------------------- |
| `cmd_vel_topic`     | `/turtle1/cmd_vel`  | Topic to subscribe to. Point it at any other teleop.     |
| `max_linear_speed`  | `0.3`               | Forward speed clamp, m/s.                                |
| `max_angular_speed` | `2.0`               | Yaw rate clamp, rad/s.                                   |
| `max_motor_dps`     | `500`               | Per-wheel clamp in degrees per second. Both wheels are scaled together so turns keep their shape. |
| `cmd_timeout`       | `0.5`               | Seconds without a command before the motors are stopped. |

```bash
./build/Release/gopigo3_ros_node --ros-args -p max_linear_speed:=0.15 -p cmd_timeout:=1.0
```

## How it works

[`gopigo3_driver.cpp`](gopigo3-ros/src/gopigo3_driver.cpp) wraps the upstream `GoPiGo3` class:
connect, set wheel speeds in degrees per second, stop, read the wheel geometry.
[`gopigo3_ros_node.cpp`](gopigo3-ros/src/gopigo3_ros_node.cpp) subscribes to
`geometry_msgs/msg/Twist` and applies differential-drive kinematics:

```text
v_left  = v - w * wheel_separation / 2
v_right = v + w * wheel_separation / 2
```

Wheel speeds are converted to shaft degrees per second, clamped, and written with
`set_motor_dps()`. The geometry comes from the board itself (`~/.gpg3_config.json` if you
calibrated it, upstream defaults otherwise). A watchdog stops the motors when no command
arrives for `cmd_timeout` seconds, because `turtle_teleop_key` publishes one `Twist` per
keystroke rather than a continuous stream.

The project only handles motion; the line follower, colour sensor and distance sensor are not
wired up. [`CMakeLists.txt`](gopigo3-ros/CMakeLists.txt) is plain CMake, no `colcon` and no
`ament_cmake`.

> [!NOTE]
> `GoPiGo3.h` defines globals and free functions at namespace scope, so it can only be included
> from one translation unit. That unit is `gopigo3_driver.cpp`; the node sees a forward
> declaration only.

## Notes

- A `.conanrc` in the repository root overrides the `CONAN_HOME` the image sets. Since the
  repository is mounted at `/repo`, a stray one sends the cache outside the `gopigo-conan`
  volume, or fails outright if it holds a Windows path.
- If HTTPS fails while building the image (typical of a TLS-inspecting proxy), put the proxy
  CA in `docker/extra-ca.crt`. That file is gitignored; `docker build` still picks it up when
  it is present.
- Do not replace [`rpi3-armv8`](profiles/rpi3-armv8) with `conan profile detect`.
  `compiler.version` and `python_version` must match what the prebuilt `ros-kilted` was built
  with, or Conan considers the binary missing and tries to build ROS from source for ARM.

## CI

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) runs the same commands as the sections
above, on Ubuntu 24.04 (the driver is Linux-only):

- **Native workspace build** — the [Prerequisites](#prerequisites) remote setup, then
  [Native workspace build](#native-workspace-build).
- **Cross-build to the Raspberry Pi** — [steps 1–3](#1-start-the-build-container) inside
  [`docker/Dockerfile`](docker/Dockerfile). `docker run` drops `-it` (no TTY in CI) and bind-mounts
  a cached directory at `/opt/conan2` instead of a Docker volume, so later runs reuse the
  Conan cache. Copying to the robot (steps 4–5) is skipped.

The first run of each job may compile `ros-kilted` from source (hours).

## Credits

The differential-drive handling is adapted from the pre-ROS
[danimtb/robotapp](https://github.com/danimtb/robotapp), and the Conan recipe from
[danimtb/conan-gopigo](https://github.com/danimtb/conan-gopigo), updated to Conan 2.
