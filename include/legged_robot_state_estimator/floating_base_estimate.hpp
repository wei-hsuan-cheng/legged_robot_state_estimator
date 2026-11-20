#ifndef LEGGED_ROBOT_STATE_ESTIMATOR__FLOATING_BASE_ESTIMATE_HPP_
#define LEGGED_ROBOT_STATE_ESTIMATOR__FLOATING_BASE_ESTIMATE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace legged_robot_state_estimator::state_estimation
{

/// Floating-base state produced by a state estimator, in the same conventions the
/// MPC observation expects: world-frame position/orientation/linear velocity and
/// body-local angular velocity.
struct FloatingBaseEstimate
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};                  //!< base position in world
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};    //!< base orientation (world<-body)
  Eigen::Vector3d linear_velocity_world{Eigen::Vector3d::Zero()};    //!< base linear velocity in world
  Eigen::Vector3d angular_velocity_local{Eigen::Vector3d::Zero()};   //!< base angular velocity in body frame
  bool valid{false};                                                 //!< false until the estimator is initialized
};

}  // namespace legged_robot_state_estimator::state_estimation

#endif  // LEGGED_ROBOT_STATE_ESTIMATOR__FLOATING_BASE_ESTIMATE_HPP_
