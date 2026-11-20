#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

#include "legged_robot_state_estimator/estimator_state_interfaces.hpp"
#include "legged_robot_state_estimator/inekf_floating_base_estimator.hpp"
#include "legged_robot_state_estimator/linear_kf_floating_base_estimator.hpp"

namespace se = legged_robot_state_estimator::state_estimation;
namespace interfaces = legged_robot_state_estimator::estimator_interfaces;

namespace
{
se::EstimatorSettings settings(const std::string & model)
{
  se::EstimatorSettings s;
  s.urdf_path = TEST_URDF;
  s.controller_joint_names = {"right_wheel_joint", "left_wheel_joint"};
  s.imu_frame = "imu";
  s.contact_frames = {"left_wheel", "right_wheel"};
  s.contact_foot_indices = {0, 1};
  s.lkf_contact_model = model;
  s.lkf_wheel_radius = 0.055;
  s.lkf_wheel_axle_axis = Eigen::Vector3d::UnitY();
  s.lkf_imu_extrinsics = true;
  return s;
}
}  // namespace

TEST(Estimator, PointContactRemainsStationary)
{
  se::LinearKfFloatingBaseEstimator filter(settings("point"));
  const std::vector<double> zero(2, 0.0);
  filter.initializeFromStance(Eigen::Quaterniond::Identity(), zero);
  se::FloatingBaseEstimate estimate;
  for (int i = 0; i < 500; ++i) {
    estimate = filter.update(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.81), zero, zero, zero, {true, true});
  }
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.position.z(), 0.25, 1e-6);
  EXPECT_LT(estimate.linear_velocity_world.norm(), 1e-6);
  EXPECT_TRUE(filter.diagnostics().position_std.allFinite());
  filter.reset();
  EXPECT_FALSE(filter.initialized());
}

TEST(Estimator, RollingWheelUsesTreadVelocity)
{
  const auto s = settings("rolling_wheel");
  se::LinearKfFloatingBaseEstimator filter(s);
  const std::vector<double> zero(2, 0.0);
  filter.initializeFromStance(Eigen::Quaterniond::Identity(), zero);
  constexpr double speed = 0.2;
  const std::vector<double> velocity(2, speed / s.lkf_wheel_radius);
  se::FloatingBaseEstimate estimate;
  for (int i = 0; i < 2000; ++i) {
    const std::vector<double> position(2, i * s.sampling_time * velocity[0]);
    estimate = filter.update(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.81), position, velocity, zero, {true, true});
  }
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.position.z(), 0.305, 1e-6);
  EXPECT_NEAR(estimate.linear_velocity_world.x(), speed, 1e-5);
  EXPECT_NEAR(estimate.linear_velocity_world.y(), 0.0, 1e-6);
  EXPECT_TRUE(filter.diagnostics().kinematic_height_valid);
  EXPECT_NEAR(filter.diagnostics().reported_height, estimate.position.z(), 1e-12);
}

TEST(Estimator, SharedInterfaceRoundTrip)
{
  se::FloatingBaseEstimate estimate;
  estimate.position = Eigen::Vector3d(1.0, 2.0, 0.3);
  estimate.linear_velocity_world = Eigen::Vector3d(0.2, 0.1, -0.1);
  estimate.valid = true;
  se::EstimatorDiagnostics diagnostics;
  diagnostics.nis = 3.0;
  diagnostics.reported_height = 0.3;
  interfaces::EstimatorStatus status;
  status.ready = true;
  status.epoch = 3;
  std::vector<double> values(interfaces::names().size());
  interfaces::pack(estimate, diagnostics, status, values);
  se::FloatingBaseEstimate result;
  se::EstimatorDiagnostics result_diagnostics;
  interfaces::EstimatorStatus result_status;
  interfaces::unpack(values, result, result_diagnostics, result_status);
  EXPECT_TRUE(result.position.isApprox(estimate.position));
  EXPECT_TRUE(result.linear_velocity_world.isApprox(estimate.linear_velocity_world));
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result_status.ready);
  EXPECT_EQ(result_status.epoch, 3u);
  EXPECT_DOUBLE_EQ(result_diagnostics.nis, 3.0);
  EXPECT_DOUBLE_EQ(result_diagnostics.reported_height, 0.3);
}

TEST(Estimator, InEkfRemainsStationaryWithScheduledContacts)
{
  auto s = settings("point");
  s.contact_source = "scheduled";
  s.contact_frames = {"left_heel", "left_toe", "right_heel", "right_toe"};
  s.contact_foot_indices = {0, 0, 1, 1};
  s.contact_beta0.assign(4, -1.0);
  s.contact_beta1.assign(4, 1.0);
  se::InekfFloatingBaseEstimator filter(s);
  const std::vector<double> zero(2, 0.0);
  filter.initializeFromStance(Eigen::Quaterniond::Identity(), zero);
  se::FloatingBaseEstimate estimate;
  for (int i = 0; i < 500; ++i) {
    estimate = filter.update(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.81), zero, zero, zero, {true, true});
  }
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.position.z(), 0.305, 1e-4);
  EXPECT_LT(estimate.linear_velocity_world.norm(), 1e-4);
  EXPECT_TRUE(filter.diagnostics().position_std.allFinite());
}

TEST(Estimator, BothPluginsLoadIndependentlyOfMpc)
{
  pluginlib::ClassLoader<controller_interface::ChainableControllerInterface> loader(
    "controller_interface", "controller_interface::ChainableControllerInterface");
  for (const auto & name : {
    "legged_robot_state_estimator/HumanoidLinearKfController",
    "legged_robot_state_estimator/HumanoidInEkfController"})
  {
    EXPECT_TRUE(loader.isClassAvailable(name));
    EXPECT_NE(loader.createSharedInstance(name), nullptr);
  }
}

TEST(Estimator, RejectsUnknownContactModel)
{
  EXPECT_THROW(se::LinearKfFloatingBaseEstimator filter(settings("unknown")), std::invalid_argument);
}
