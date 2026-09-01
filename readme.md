# gopigo-ros-robot

Drive a [GoPiGo3](https://gopigo.io) with ROS 2 (`turtle_teleop_key` → `/turtle1/cmd_vel` → wheels).
ROS comes from Conan via [`ros-conan`](https://github.com/conan-io/ros-conan) (`ros-kilted`): no
`apt`, no `rosdep`, no ROS distro on the Pi.

Linux only. Docker on the laptop; 64-bit Raspberry Pi OS on the robot. The first `ros-kilted`
build takes hours; later runs reuse the `gopigo-conan` volume.

## 1. Build container

```bash
docker build -t gopigo-builder docker
docker run -it --rm -v "$PWD:/repo" -v gopigo-conan:/opt/conan2 gopigo-builder
```

Everything below runs **inside** this shell (`/repo`).

## 2. ROS for arm64

```bash
git clone --depth 1 --branch danimtb/fix-vcs https://github.com/conan-io/ros-conan.git /opt/ros-conan
conan remote add ros-conan /opt/ros-conan --type=local-recipes-index --force
conan install --requires=ros-kilted/2026.06.17 -pr:b=default -pr:h=profiles/rpi3-armv8 --build=missing
```

## 3. Node + libraries

```bash
conan create gopigo3 -pr:b=profiles/ros -pr:h=profiles/rpi3-armv8 --build=missing

cd /tmp
conan install /repo/gopigo3-ros -pr:b=/repo/profiles/ros -pr:h=/repo/profiles/rpi3-armv8 \
    --build=never --deployer=full_deploy --deployer-folder=/repo/gopigo3-ros/deploy

cd /repo/gopigo3-ros
cmake --preset conan-release
cmake --build --preset conan-release
```

`conan install` is run from `/tmp` so Conan does not treat `gopigo3` as a workspace editable.
`--build=never` fails instead of compiling ROS for ARM again.

## 4. Copy to the Pi

Use the robot IP if `gopigo3.local` does not resolve (the container has no mDNS).

```bash
ssh raspi@gopigo3.local true
rsync -az --relative --info=progress2 \
    build/Release/generators build/Release/gopigo3_ros_node deploy relocate_env.sh \
    raspi@gopigo3.local:~/gopigo3-ros/
```

`--relative` keeps `generators` three levels below `deploy/`, as the env scripts expect.

## 5. Run on the Pi

SPI on (`raspi-config`), user in group `spi`, `gopigo3_power` running
([FAQ](https://github.com/DexterInd/GoPiGo3/blob/master/Installation_FAQ.md)).

```bash
ssh raspi@gopigo3.local
cd ~/gopigo3-ros
bash relocate_env.sh
. build/Release/generators/conanrun.sh
export ROS_DOMAIN_ID=42
./build/Release/gopigo3_ros_node
```

`relocate_env.sh` once per transfer. A good deploy reaches ROS start-up, then fails on SPI if
the board is off. A missing `.so` means the tree did not copy intact.

## 6. Teleop on the laptop

Same `ROS_DOMAIN_ID`. Needs `ros-kilted` with `variant=desktop`:

```bash
export ROS_DOMAIN_ID=42
conan run "ros2 run turtlesim turtlesim_node"
conan run "ros2 run turtlesim turtle_teleop_key --ros-args -p scale_linear:=0.2 -p scale_angular:=1.0"
```

| Parameter | Default |
| --- | --- |
| `cmd_vel_topic` | `/turtle1/cmd_vel` |
| `max_linear_speed` | `0.3` m/s |
| `max_angular_speed` | `2.0` rad/s |
| `max_motor_dps` | `500` |
| `cmd_timeout` | `0.5` s without a command, then stop |

```bash
./build/Release/gopigo3_ros_node --ros-args -p max_linear_speed:=0.15
```

## Desktop compile (no robot)

Python 3.12, Conan 2.31+, Linux:

```bash
git clone --branch danimtb/fix-vcs https://github.com/conan-io/ros-conan.git
conan remote add ros-conan ./ros-conan --type=local-recipes-index
conan profile detect --force
conan workspace source
conan workspace build --profile profiles/ros --build=missing
```

The binary is `gopigo3-ros/build/Release/gopigo3_ros_node`. It will not talk to hardware.

CI runs these same commands (cross skips the copy to the Pi).

## Notes

- `ros-conan` is cloned from the `danimtb/fix-vcs` branch: its `vcs import` passes
  `--retry 5 --workers 5`, without which the ROS source clones fail. Back to `main` once
  that branch is merged.
- Building ROS from source needs tens of GB. GitHub runners guarantee only 14 GB, so CI
  deletes their preinstalled toolchains first and drops the ROS source and build trees once
  the package is in the cache.
- Do not put a `.conanrc` in the repo root: the container would miss the `gopigo-conan` volume.
- HTTPS errors building the image: add the proxy CA as gitignored `docker/extra-ca.crt`.
- Keep [`profiles/rpi3-armv8`](profiles/rpi3-armv8); `conan profile detect` will rebuild ROS for ARM.
- Workspace: [`gopigo3/`](gopigo3) driver, [`gopigo3-ros/`](gopigo3-ros) node. Plain CMake, no `colcon`.

Credits: kinematics from [danimtb/robotapp](https://github.com/danimtb/robotapp), recipe from
[danimtb/conan-gopigo](https://github.com/danimtb/conan-gopigo).
