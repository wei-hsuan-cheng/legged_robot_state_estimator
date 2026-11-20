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
Both MPC repositories list only this shared estimator in their dependency manifests and import its internal dependencies through this package's own manifest.
To share one local checkout across workspaces, symlink that checkout into each workspace's `src/` instead of importing another copy.
For example, from the workspace root:

```bash
ESTIMATOR_SOURCE=/absolute/path/to/legged_robot_state_estimator
ln -s "$ESTIMATOR_SOURCE" src/legged_robot_state_estimator
vcs import src < "$ESTIMATOR_SOURCE/legged_robot_state_estimator.repos"
```

Skip the symlink or dependency import when the corresponding package is already present.
`colcon build --packages-up-to <consumer_package>` then discovers and builds the estimator through its declared dependencies.
An external checkout that is neither under `src/` nor symlinked there is not discovered by the normal workspace build.
The current ROS controller reference interface exposes two support flags, matching the G1, H2, and Upkie integrations.


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
These build-time schemas belong to this package and are passed to `generate_parameter_library` in [CMakeLists.txt](CMakeLists.txt).
They generate the typed parameter libraries during the build; they are not runtime controller configurations.
The complete H2-based runtime example is [config/ros2_controllers.yaml](config/ros2_controllers.yaml), installed under this package's share directory.
Replace its absolute URDF placeholder and adapt the joint, IMU, and contact-frame names for another robot.
It registers both plugins, but the application should activate only its selected estimator because both publish the same odometry and base TF.
Loading this YAML alone does not start estimation: the consumer must write a positive `initialize_epoch` and, for scheduled contacts, the two support flags through the reference interfaces.
Supply `urdfFile`, `jointNames`, `baseFrame`, `imuInterfaceName`, `imuFrame`, and `contactFrames` for the robot.
The MPC launches select the estimator with the `floatingBaseEstimator` argument.
H2 defaults to `humanoid_linear_kf` and also supports `humanoid_inekf`.
The wheeled-biped configuration uses the linear KF's `rolling_wheel` model; the InEKF does not implement that rolling-contact model.
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


## Tests

```bash
colcon test --packages-select legged_robot_state_estimator --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover stationary point contacts, rolling-wheel velocity and height, stationary InEKF updates, interface packing, plugin discovery/loading, and invalid contact models.
These checks do not replace robot-specific closed-loop acceptance tests.


## Models

The linear filter follows the two-stage Cheetah 3 design and its `legged_control` port.
The invariant filter uses the contact-aided InEKF backend.
