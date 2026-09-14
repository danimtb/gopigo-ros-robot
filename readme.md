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
git clone --depth 1 https://github.com/conan-io/ros-conan.git /opt/ros-conan
conan remote add ros-conan /opt/ros-conan --type=local-recipes-index --force
conan install --requires=ros-kilted/2026.06.17 -pr:b=default -pr:h=profiles/rpi3-armv8 --build=missing
```

## 3. Node + libraries

```bash
cd /tmp
conan create /repo/gopigo3 -pr:b=/repo/profiles/ros -pr:h=/repo/profiles/rpi3-armv8 --build=missing

conan install /repo/gopigo3-ros -pr:b=/repo/profiles/ros -pr:h=/repo/profiles/rpi3-armv8 --build=never --deployer=full_deploy --deployer-folder=/repo/gopigo3-ros/deploy

cd /repo/gopigo3-ros
cmake --preset conan-release
cmake --build --preset conan-release
```

Both run from `/tmp` so Conan does not treat `gopigo3` as a workspace editable: from `/repo`
`conan create` builds nothing and its `test_package` fails on the missing library. Run from
`/repo`, `conan install` resolves `gopigo3` to the editable instead of the cache package and
`cmake --preset` then fails with `Library 'gopigo3' not found in package`.
`--build=never` fails instead of compiling ROS for ARM again.

## 4. Copy to the Pi

Use the robot IP.

```bash
rsync -az --relative --info=progress2 build/Release/generators build/Release/gopigo3_ros_node deploy relocate_env.sh raspi@192.168.1.136:~/gopigo3-ros/
rsync -az ../scripts/run_robot.sh raspi@192.168.1.136:~/gopigo3-ros/run_robot.sh
```

`--relative` keeps `generators` three levels below `deploy/`, as the env scripts expect.

## 5. Run on the Pi

This SSH is a real session on the robot (the node must keep running there). SPI on
(`raspi-config`), user in group `spi`, `gopigo3_power` running
([FAQ](https://github.com/DexterInd/GoPiGo3/blob/master/Installation_FAQ.md)).

```bash
ssh raspi@192.168.1.136
cd ~/gopigo3-ros
bash relocate_env.sh
. build/Release/generators/conanrun.sh
export ROS_DOMAIN_ID=42
./build/Release/gopigo3_ros_node --ros-args -r __ns:=/gopigo_a -p line_follow:=true
# or the convoy demo (first robot without a leader elects itself):
bash run_robot.sh a
bash run_robot.sh b
```

Use a different `__ns` on the second robot (`/gopigo_b`). Same `ROS_DOMAIN_ID` so the laptop
sees both. Topic names are relative (`cmd_vel`, `color`, `led/eyes`, …) and become
`/gopigo_a/cmd_vel`, `/gopigo_a/color`, … The convoy bus is global: `/convoy/command`
and `/convoy/peer`. They do not drive until you publish `START` on `/convoy/run`.

`relocate_env.sh` once per transfer. A good deploy reaches ROS start-up, then fails on SPI if
the board is off. A missing `.so` means the tree did not copy intact.

## 6. Teleop on the laptop

Same `ROS_DOMAIN_ID`. Needs `ros-kilted` with `variant=desktop`. Remap turtlesim onto the robot
namespace:

```bash
export ROS_DOMAIN_ID=42
conan run "ros2 run turtlesim turtlesim_node"
conan run "ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/gopigo_a/cmd_vel -p scale_linear:=0.2 -p scale_angular:=1.0"
```

| Parameter | Default |
| --- | --- |
| `cmd_vel_topic` | `cmd_vel` |
| `max_linear_speed` | `0.3` m/s |
| `max_angular_speed` | `4.0` rad/s |
| `max_motor_dps` | `700` |
| `cmd_timeout` | `0.5` s without a command, then stop |
| `line_follower_port` | `I2C` (`AD1`, `AD2`, or `off`) |
| `line_follower_topic` | `line_follower` |
| `line_follow` | `false` (set `true` to drive from the sensor) |
| `line_follow_speed` | `0.10` m/s |
| `line_follow_kp` | `8.0` |
| `line_follow_kd` | `0.08` |
| `line_follow_slowdown` | `0.35` of the speed dropped in a full turn |
| `line_threshold` | `0.10` (below it a sensor is on the line) |
| `line_search_timeout` | `1.5` s turning to find a lost line, then stop |
| `color_sensor_port` | `I2C` (`AD1`, `AD2`, or `off`) |
| `color_sensor_topic` | `color` |
| `color_led` | `true` (board LED for reflected colour) |
| `led_topic_prefix` | `led` |
| `robot_id` | `a` (startup only; use `b` on the second robot) |
| `convoy_enable` | `false` |
| `convoy_role` | `follower` (auto-elects if no leader; `ros2 param set` still works) |
| `cruise_speed` | `0.10` m/s (green card) |
| `turbo_speed` | `0.15` m/s (yellow card) |
| `sync_pause` | `1.2` s stopped on both robots before a speed or role change (`0.0` disables) |
| `leader_timeout` | `3.0` s without a leader heartbeat → this robot leads |
| `handover_turn` | `true` (both robots spin half a turn when they swap roles) |
| `turn_speed` | `1.5` rad/s for that spin (~2 s for half a turn) |
| `color_min_saturation` | `0.25` (ignore floor / grey) |
| `color_min_clear` | `0.05` |
| `color_debounce` | `2` matching colour reads |
| `color_cooldown` | `3.0` s before the same card fires again (matches the eye flash) |

Do not put a leading `/` on those topic parameters: that would make them global and both robots
would share the same names.

`line_follow` and every number above can be set while the node drives, so switching between
following and waiting for teleop is a parameter set from the laptop (the node is
`gopigo3_ros` inside the namespace):

```bash
conan run "ros2 param set /gopigo_a/gopigo3_ros line_follow true"
conan run "ros2 param set /gopigo_a/gopigo3_ros line_follow false"
conan run "ros2 param set /gopigo_a/gopigo3_ros line_follow_kp 6.0"
conan run "ros2 param get /gopigo_a/gopigo3_ros line_follow"
```

Switching it off stops the wheels and leaves the robot waiting for `cmd_vel`; switching it on
starts following `cmd_timeout` after the last teleop command, which is also how a teleop key
takes the robot over mid-line without touching the parameter. Write the decimal point on the
numbers (`700.0`, not `700`) or the value goes out as an integer and is refused. Topics, ports,
`color_led` and `robot_id` are wired up at start-up, so setting those is refused too, rather
than accepted and ignored.

The Dexter line follower (black board, 6 IR; red board, 5 IR) is read over I2C at `0x06`.
Values are `0` (black) … `1` (white), left → right with the board arrow forward. On the Grove
I2C port that is the Pi's `/dev/i2c-1`, so `i2cdetect -y 1` must list `0x06`. Its ATtiny
stretches the clock, so each register write and read goes out as its own transfer with 10 ms in
between: a combined write+read (repeated start) is what `di_i2c` avoids on this bus, and it
does fail here.

Following runs on its own 20 ms timer (colour polls at 100 ms so it does not steal the bus).
`line_threshold` is what makes the steering firm: sensors above it are treated as bare surface
and carry no weight, so a line under one end of the board gives the full error instead of an
average watered down by the five sensors looking at the floor. Read
`line_follower` on a plain floor and put the threshold below those values. When the board sees
only white, the last turn is held for `line_search_timeout` to bring the line back in view.

The wheels get `v ± ω · 0.0585`, so on a 66.5 mm wheel each rad/s of `ω` adds ~100 dps to one
wheel and takes it off the other. Turning comes out of `line_follow_kp`, not out of braking:
with a big `line_follow_slowdown` the outer wheel ends up no faster than when going straight
and the robot pivots on the inner wheel instead of driving the curve. The node warns when the
turn it asks for is being capped by `max_angular_speed`.

The Dexter Light & Color Sensor (TCS34725 at `0x29`) can share the same Grove I2C bus as the
line follower. `color` is `std_msgs/ColorRGBA` (`r,g,b` plus `a` = clear/intensity, 0…1).
`color/name` is the nearest of black, white, red, green, blue, yellow, cyan, fuchsia.

Board LEDs (red GoPiGo3): Dexter eyes (RGB) and red blinkers. Colour channels and blinker
brightness are 0…1. Names below are under the robot namespace (example: `/gopigo_a`).

| Topic | Type |
| --- | --- |
| `cmd_vel` | `geometry_msgs/Twist` |
| `line_follower` | `std_msgs/Float32MultiArray` |
| `line_follower/position`, `line_follower/state` | `Float32`, `String` |
| `color`, `color/name` | `ColorRGBA`, `String` |
| `led/eyes` | `std_msgs/ColorRGBA` (both eyes) |
| `led/eye/left`, `led/eye/right` | `std_msgs/ColorRGBA` |
| `led/blinkers` | `std_msgs/Float32` (both red blinkers) |
| `led/blinker/left`, `led/blinker/right` | `std_msgs/Float32` |
| `/convoy/command` | `std_msgs/String`: `STOP`, `CRUISE`, `TURBO`, `HANDOVER` (global) |
| `/convoy/peer` | `std_msgs/String` CSV `id,role,cmd,v_mps,seq,term` (global) |
| `/convoy/run` | `std_msgs/String`: `START` or `STOP` (global; robots stay still until `START`) |

```bash
./build/Release/gopigo3_ros_node --ros-args -r __ns:=/gopigo_a -p max_linear_speed:=0.15
./build/Release/gopigo3_ros_node --ros-args -r __ns:=/gopigo_a -p line_follow:=true
```

On the laptop, same `ROS_DOMAIN_ID`:

```bash
conan run "ros2 topic echo /gopigo_a/line_follower"
conan run "ros2 topic echo /gopigo_a/line_follower/state"
conan run "ros2 topic echo /gopigo_a/color"
conan run "ros2 topic echo /gopigo_a/color/name"
conan run "ros2 topic pub /gopigo_a/led/eyes std_msgs/msg/ColorRGBA '{r: 0.0, g: 0.4, b: 1.0, a: 1.0}'"
conan run "ros2 topic pub /gopigo_a/led/blinker/left std_msgs/msg/Float32 '{data: 1.0}'"
```

## Convoy demo

Two robots on the same `ROS_DOMAIN_ID`, each with its own namespace. Start both the
same way (`run_robot.sh a` / `run_robot.sh b`). Each begins as follower and becomes
leader if no leader appears on `/convoy/peer` within `leader_timeout` (3 s). Only the
leader applies colour cards; the follower copies `/convoy/command`.

Both robots run the same speed off the same order, and neither measures the gap. Before a
speed or role change they both stop for `sync_pause`, blink the card colour as a countdown
and leave together, so the leader cannot pull away while the order is still in flight. The
leader waits out the pause too: it publishes the order first and applies it at the end, like
the follower. A red card skips the pause, since a stop needs no countdown.

They stay still until `START` on `/convoy/run`. Place them on the line with the gap
you want before that. Nothing holds that gap: wheel slip drifts it, and there is no
ultrasonic. `v_mps` on `/convoy/peer` is the number to echo when checking that both robots
agree on speed.

A role swap reverses the convoy: once the pause is over, both robots spin half a turn in
place and carry on driving the other way down the line, so the robot that just took the
lead ends up at the head instead of trailing the one that gave it away. Leave enough gap
for both of them to spin without touching. There is no driving backwards to be had here:
the sensor board is at the front, and a line follower with the board behind the wheels
runs away from the line instead of onto it. The spin is measured on the wheel encoders
and finished off on the line itself, which crosses the point the robot spins about and so
comes back square under the board half a turn later. `handover_turn:=false` keeps the old
behaviour, where the roles swap but neither robot moves out of place. An election or a
term clash does not turn anybody around: those happen when a robot is missing, not when
two robots agree to hand over.

Cards: red = stop, yellow = turbo, green = cruise, blue = swap roles (no overtake).
Blue with no peer is ignored (red flash). If the follower disappears, the leader
keeps going. If the leader is silent for 3 s, the other robot takes over with a
higher `term`; when the old leader reconnects it yields (lower term).

Eyes (low brightness) are the stand UI:

| Pattern | Meaning |
| --- | --- |
| Slow white blink | Waiting for `/convoy/run START` |
| Fast blink then solid (yellow / green / blue) | Sync pause before a speed or role change; both leave on the solid |
| Solid blue after that countdown | Spinning half a turn to reverse the convoy after a role swap |
| Flash red (~3 s) | Red card (both stop), follower saw a card, or blue with no peer |
| Flash yellow / green (~3 s) | Card read, but that speed was already in effect |
| Both eyes on | Leader, peer present |
| Both eyes off | Follower |
| Left eye on | Leader, one robot (never saw a peer) |
| Left eye blinking | Leader, peer lost (WiFi) |
| Flash white | `START`, or peer back / yielded the lead |

Stand transport is Fast DDS (the Kilted default already used with `ROS_DOMAIN_ID`).
Zenoh is optional: only if your Conan `ros-kilted` tree contains `rmw_zenoh_cpp`, set
`RMW_IMPLEMENTATION=rmw_zenoh_cpp` on every robot and start `rmw_zenohd` once.

```bash
# laptop, same ROS_DOMAIN_ID. Start both Pi nodes first, then:
conan run "ros2 topic pub --once /convoy/run std_msgs/msg/String '{data: START}'"
conan run "ros2 topic pub --once /convoy/run std_msgs/msg/String '{data: STOP}'"
conan run "ros2 topic echo /convoy/command"
conan run "ros2 topic echo /convoy/peer"
conan run "ros2 param set /gopigo_a/gopigo3_ros cruise_speed 0.12"
```

Resilience check: start both nodes, `START`, then power off one robot. The other
should keep answering red / yellow / green without a restart.

Enable I2C in `raspi-config` if a sensor is on the I2C Grove. Use `-p line_follower_port:=AD1`
or `-p color_sensor_port:=AD1` when that sensor is on Grove AD1. `-p color_sensor_port:=off`
skips the colour probe.


## Desktop compile (no robot)

Python 3.12, Conan 2.31+, Linux:

```bash
git clone https://github.com/conan-io/ros-conan.git
conan remote add ros-conan ./ros-conan --type=local-recipes-index
conan profile detect --force
conan workspace source
conan workspace build --profile profiles/ros --build=missing
```

The binary is `gopigo3-ros/build/Release/gopigo3_ros_node`. It will not talk to hardware.

CI runs these same commands (cross skips the copy to the Pi).

## Notes

- Do not put a `.conanrc` in the repo root: the container would miss the `gopigo-conan` volume.
- HTTPS errors building the image: add the proxy CA as gitignored `docker/extra-ca.crt`.
- Keep [`profiles/rpi3-armv8`](profiles/rpi3-armv8); `conan profile detect` will rebuild ROS for ARM.
- Workspace: [`gopigo3/`](gopigo3) driver, [`gopigo3-ros/`](gopigo3-ros) node. Plain CMake, no `colcon`.

Credits: kinematics from [danimtb/robotapp](https://github.com/danimtb/robotapp), recipe from
[danimtb/conan-gopigo](https://github.com/danimtb/conan-gopigo).
