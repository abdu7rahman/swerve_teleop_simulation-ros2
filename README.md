# swerve_teleop_simulation — ROS 2

ROS 2 port of [swerve_teleop_simulation](https://github.com/abdu7rahman/swerve_teleop_simulation):
a four-module swerve base in Gazebo, driven from `/cmd_vel`.

## Layout

```
swerve_description/   URDF, meshes, ros2_control block, rviz display
swerve_bringup/       kinematics node, controller config, Gazebo launch
```

## Build

```bash
mkdir -p ~/ros2_ws/src && cp -r swerve_description swerve_bringup ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Run

Simulation with controllers and kinematics:

```bash
ros2 launch swerve_bringup gazebo.launch.py
```

Then drive it:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Model only, no physics:

```bash
ros2 launch swerve_description display.launch.py
```

## Joint roles

| Joint | Role | Interface |
|---|---|---|
| `Rev1`, `Rev3`, `Rev5`, `Rev7` | steering | position |
| `Rev2`, `Rev4`, `Rev6`, `Rev8` | wheel drive | velocity |

Module order throughout — controller joint lists and `module_positions` — is
front-left, front-right, rear-right, rear-left, matching the ROS 1
`holders: ['Rev3','Rev5','Rev1','Rev7']` / `wheels: ['Rev4','Rev6','Rev2','Rev8']`
pairing against `positions: [[-0.3,-0.3], [-0.3,0.3], [0.3,-0.3], [0.3,0.3]]`.

## What changed from ROS 1

| ROS 1 | ROS 2 |
|---|---|
| catkin | `ament_cmake` + `ament_python` |
| `<transmission>` blocks (`swerve.trans`) | `<ros2_control>` block |
| `libgazebo_ros_control.so` | `gz_ros2_control` |
| Gazebo Classic + `empty_world.launch` | Gazebo Sim + `ros_gz_sim` |
| `gazebo_ros spawn_model` | `ros_gz_sim create` |
| `joint_state_controller/JointStateController` | `joint_state_broadcaster/JointStateBroadcaster` |
| `effort_controllers/JointPositionController` ×8 | one `JointGroupPositionController` |
| `swerve_steering_controller` | `swerve_kinematics` node |
| `.launch` XML | Python launch with event handlers |
| bundled `teleop_twist_keyboard.py` | upstream `teleop_twist_keyboard` package |

### The steering controller had to be rebuilt

ROS 1 used `swerve_steering_controller/SwerveSteeringController`, a third-party
`ros_control` plugin. It has no ROS 2 port, and `ros2_controllers` ships
diff-drive, tricycle and Ackermann steering but nothing for four independently
steered modules.

Rather than leave a dangling dependency, the kinematics moved into
`swerve_bringup/swerve_kinematics.py`: `/cmd_vel` in, steering positions and
wheel velocities out to two stock group controllers. The maths is the standard
`v_module = v_chassis + omega × r`, with the module geometry, radii and joint
limits read from parameters carrying the same values the ROS 1
`ros_controllers.yaml` specified.

Two behaviours the port adds because a hand-written controller can afford them:

- **Shortest-path steering.** A module pointed 180° away spinning backwards is
  the same motion. If flipping is the shorter turn, it flips and negates the
  wheel speed instead of swinging most of a half turn.
- **Hold heading at rest.** With no velocity demand the modules keep their last
  angle rather than snapping to zero, so centring the stick does not make them
  twitch.

### Other fixes

- **The two controller configs disagreed and neither was complete.** The
  top-level `ros_controllers.yaml` configured `swerve_steering_controller` while
  `swerve_description/launch/controller.yaml` configured eight separate
  `effort_controllers/JointPositionController` instances under a
  `swerve_controller` namespace — including position controllers on the four
  *wheel* joints, which should be velocity-driven. One config now, with the
  joint roles matching what the joints actually do.
- **Transmission interfaces contradicted the controllers.** `swerve.trans`
  declared `PositionJointInterface` on Rev1/3/5/7 and `VelocityJointInterface`
  on Rev2/4/6/8 — correct — but `controller.yaml` then loaded position
  controllers on all eight. The `ros2_control` block and the controller config
  agree.
- **No command timeout.** Nothing stopped the wheels if `/cmd_vel` went quiet.
  The kinematics node zeroes wheel velocity after 0.5 s while leaving the
  steering where it is.
- **The PID gains were commented out.** `gazebo_ros_control/pid_gains` in
  `ros_controllers.yaml` was entirely commented, so Gazebo fell back to defaults.
  The group controllers do not need them; if you move to effort control, that is
  where they would go.

The URDF geometry, inertials, Gazebo material and friction settings carry over
unchanged.
