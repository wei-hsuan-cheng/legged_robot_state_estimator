# Shared Estimator Extraction Handoff

Date: 2026-11-20.


## Scope

Extract both estimator controllers from `legged_robot_mpc_controller` and remove the duplicated linear KF from `turbompc_controllers`.
The shared package owns the plugin library, filter wrappers, parameter schemas, and exported state/reference interface definitions.
Robot descriptions, launch arguments, and estimator tuning remain with the MPC applications.
The InEKF backend remains an external `legged_state_estimator` dependency.
No estimator process or observation equation is intentionally changed.


## Compatibility

Plugin prefixes change to `legged_robot_state_estimator/`.
Controller instance names stay `humanoid_inekf` and `humanoid_linear_kf`.
The OCS2 diagnostic superset is now shared with TurboMPC, so both consumers must be rebuilt.
The added TurboMPC H2 InEKF configuration copies the OCS2 H2 settings and is optional.
Upkie continues to use the rolling-wheel linear KF.
Do not select the point-contact InEKF as a rolling-wheel estimator.


## Validation

The shared package builds in both the `ros_jazzy` and `turbompc_jazzy` containers.
All six estimator tests pass in both environments, including loading both plugins and a stationary InEKF update with scheduled contacts.
Structured YAML comparisons confirm the existing G1, H2, and Upkie estimator parameters are unchanged in both consumers.
The optional TurboMPC H2 InEKF block matches the OCS2 H2 block.
Pending: finish compiling both MPC consumers against the shared package.
Robot walking, driving, and push-recovery performance are not established by the extraction unit tests.


## Future Work

- Keep one shared diagnostic interface instead of reintroducing per-MPC copies.
- Add dedicated estimator regression recordings for H2, G1, and Upkie.
- Do not change robot noise tuning as part of a packaging-only migration.
- Track InEKF and rolling-wheel model improvements separately from this extraction.
