# Legged Robot State Estimator

Shared ROS 2 Jazzy `ros2_control` floating-base estimators for legged and wheeled-biped robots.
Both OCS2 and TurboMPC controllers can consume the same estimate interfaces without compiling their own filters.


## Estimators

| Plugin | Model | Attitude |
| --- | --- | --- |
| `legged_robot_state_estimator/HumanoidInEkfController` | Contact-aided right-invariant EKF, using the `legged_state_estimator` library | Estimated jointly with position, velocity, and optional IMU biases |
| `legged_robot_state_estimator/HumanoidLinearKfController` | Two-stage linear KF, with point or rolling-wheel contacts | Supplied by the IMU quaternion, with optional mounting correction |

The package has no OCS2, TurboMPC, MuJoCo, or robot-description dependency.
It uses Pinocchio and the robot URDF supplied at runtime.
The InEKF backend is the separate [`legged_state_estimator`](https://github.com/wei-hsuan-cheng/legged_state_estimator) library; it is not another controller plugin.


## Build

In a ROS 2 Jazzy workspace, clone this repository under `src/`.
Import the InEKF backend only if it is not already present in the workspace or underlay:

```bash
mkdir -p ~/estimator_ws/src
cd ~/estimator_ws
git clone https://github.com/wei-hsuan-cheng/legged_robot_state_estimator.git src/legged_robot_state_estimator
vcs import src < src/legged_robot_state_estimator/legged_robot_state_estimator.repos
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to legged_robot_state_estimator --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Do not import a second copy of either package into a workspace that already contains it.
Both MPC repositories include this package and the InEKF backend in their dependency manifests.


## Controller Configuration

Keep the controller instance names and robot-specific tuning in the application's YAML:

```yaml
controller_manager:
  ros__parameters:
    humanoid_linear_kf:
      type: legged_robot_state_estimator/HumanoidLinearKfController
    humanoid_inekf:
      type: legged_robot_state_estimator/HumanoidInEkfController
```

The parameter schemas are [linear KF](config/humanoid_linear_kf_parameter.yaml) and [InEKF](config/humanoid_inekf_parameter.yaml).
Supply `urdfFile`, `jointNames`, `baseFrame`, `imuInterfaceName`, `imuFrame`, and `contactFrames` for the robot.
The existing MPC launches keep the `floatingBaseEstimator` argument and robot configuration values unchanged.
The H2 TurboMPC configuration also provides an optional `humanoid_inekf` instance using the H2 OCS2 estimator settings.
The default remains `humanoid_linear_kf`.
The wheeled-biped configuration uses the linear KF's `rolling_wheel` model; the extracted InEKF does not implement that rolling-contact model.
Use `contact.source: scheduled` for the linear KF; its no-contact-input overload assumes every contact is planted rather than detecting contact from torque.
The rolling-wheel height observation still assumes a ground plane and does not estimate an unknown terrain map.


## Consumer Interface

Consumers depend on `legged_robot_state_estimator` and include:

```cpp
#include <legged_robot_state_estimator/estimator_state_interfaces.hpp>
```

The public namespaces are `legged_robot_state_estimator::state_estimation` and `legged_robot_state_estimator::estimator_interfaces`.
The estimator exports pose, world linear velocity, body angular velocity, validity, readiness, epoch, and diagnostics through state interfaces.
The consumer writes two scheduled-contact flags and an initialization epoch through reference interfaces.
Odometry is published separately on the configured topic, normally `/humanoid/state_estimate/odom`.
Control does not need to subscribe to that topic.

Rebuild both the estimator and its consumers after migrating.
The shared diagnostic interface includes the full OCS2 estimator diagnostics, so an old TurboMPC binary must not be mixed with the new interface headers.


## Tests

```bash
colcon test --packages-select legged_robot_state_estimator --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover stationary point contacts, rolling-wheel velocity and height, stationary InEKF updates, interface packing, plugin discovery/loading, and invalid contact models.
These checks do not replace robot-specific closed-loop acceptance tests.


## Provenance

The ROS controller integration and filters were extracted from `legged_robot_mpc_controller` at `e7aa9ce0f91cae410dcca0016f8ccdab6a603baa`.
The linear KF duplicated in `turbompc_controllers` at `c86b1c048335ff40eb784da0a923f46a992fbaa2` used the same filtering equations but fewer diagnostic fields.
This extraction preserves the OCS2 implementation and adds no estimator tuning or filtering algorithms.
The linear filter follows the two-stage Cheetah 3 design and its `legged_control` port; the invariant filter uses the existing contact-aided InEKF backend.
See [handoff](docs/handoff.md) for migration and validation status.
