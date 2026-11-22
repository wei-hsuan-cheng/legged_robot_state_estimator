// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#include "legged_robot_state_estimator/humanoid_linear_kf.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "legged_robot_state_estimator/humanoid_linear_kf_parameters.hpp"

namespace legged_robot_state_estimator
{

struct HumanoidLinearKf::Params
{
  std::shared_ptr<humanoid_linear_kf::ParamListener> listener;
  humanoid_linear_kf::Params values;
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node;
};

namespace
{

// geometryFile: the wheel radius, tire centre and axle shared with the MPC and the offline tools. The file is the
// single source, so the same values in the parameter file are refused rather than silently overridden.
void applyGeometryFile(humanoid_linear_kf::Params & p, rclcpp_lifecycle::LifecycleNode & node)
{
  const auto overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  for (const char * name : {"wheel.radius", "wheel.centerOffset", "wheel.axleAxis"}) {
    if (overrides.count(name) > 0) {
      throw std::invalid_argument(std::string(name) + " comes from geometryFile; remove it from the parameter file");
    }
  }
  const YAML::Node wheel = YAML::LoadFile(p.geometryFile)["geometry"]["wheel"];
  if (!wheel) {
    throw std::invalid_argument("geometry " + p.geometryFile + " has no 'geometry.wheel' map");
  }
  p.wheel.radius = wheel["radius"].as<double>();
  p.wheel.centerOffset = wheel["center_offset"].as<std::vector<double>>();
  p.wheel.axleAxis = wheel["axle_axis"].as<std::vector<double>>();
  if (!(p.wheel.radius > 0.0) || p.wheel.centerOffset.size() != 3 || p.wheel.axleAxis.size() != 3) {
    throw std::invalid_argument(
      "geometry " + p.geometryFile + ": wheel needs radius > 0 and three-element center_offset and axle_axis");
  }
}

}  // namespace

HumanoidLinearKf::HumanoidLinearKf() : HumanoidStateEstimator("humanoid_linear_kf") {}
HumanoidLinearKf::~HumanoidLinearKf() = default;

bool HumanoidLinearKf::declareParameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  try {
    params_ = std::make_unique<Params>();
    params_->listener = std::make_shared<humanoid_linear_kf::ParamListener>(node);
    params_->node = node;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] declaring parameters failed: %s", name().c_str(), e.what());
    return false;
  }
  return true;
}

bool HumanoidLinearKf::readParameters(
  EstimatorCommonSettings & common, state_estimation::EstimatorSettings & filter)
{
  if (!params_ || !params_->listener) {
    return false;
  }
  params_->values = params_->listener->get_params();
  if (!params_->values.geometryFile.empty()) {
    const auto node = params_->node.lock();
    if (!node) {
      return false;
    }
    try {
      applyGeometryFile(params_->values, *node);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "[%s] %s", name().c_str(), e.what());
      return false;
    }
    RCLCPP_INFO(
      node->get_logger(), "[%s] wheel geometry from %s: radius %.4f m", name().c_str(),
      params_->values.geometryFile.c_str(), params_->values.wheel.radius);
  }
  const auto & p = params_->values;

  common.urdf_file = p.urdfFile;
  common.joint_names = p.jointNames;
  common.base_frame = p.baseFrame;
  common.imu_interface_name = p.imuInterfaceName;
  common.imu_frame = p.imuFrame;
  common.contact_frames = p.contactFrames;
  common.odom_topic = p.odomTopic;
  common.odom_frame = p.odomFrame;
  common.publish_rate = p.publishRate;
  common.contact_source = p.contact.source;
  common.ready_min_seconds = p.readiness.minSeconds;
  common.ready_roll_pitch_std_max = p.readiness.rollPitchStdDevMax;
  common.ready_height_std_max = p.readiness.heightStdDevMax;
  common.ready_velocity_std_max = p.readiness.velocityStdDevMax;

  filter.contact_foot_indices = p.contact.footIndices;

  filter.num_feet = static_cast<int>(p.numFeet);
  filter.lkf_foot_radius = p.footRadius;
  filter.lkf_imu_process_noise_position = p.imuProcessNoisePosition;
  filter.lkf_imu_process_noise_velocity = p.imuProcessNoiseVelocity;
  filter.lkf_foot_process_noise_position = p.footProcessNoisePosition;
  filter.lkf_foot_sensor_noise_position = p.footSensorNoisePosition;
  filter.lkf_foot_sensor_noise_velocity = p.footSensorNoiseVelocity;
  filter.lkf_foot_height_sensor_noise = p.footHeightSensorNoise;
  filter.lkf_swing_noise_multiplier = p.swingNoiseMultiplier;
  filter.lkf_contact_model = p.contactModel;
  filter.lkf_wheel_radius = p.wheel.radius;
  filter.lkf_wheel_center_offset = Eigen::Vector3d(p.wheel.centerOffset[0], p.wheel.centerOffset[1], p.wheel.centerOffset[2]);
  filter.lkf_wheel_axle_axis = Eigen::Vector3d(p.wheel.axleAxis[0], p.wheel.axleAxis[1], p.wheel.axleAxis[2]);
  filter.lkf_imu_extrinsics = p.imuExtrinsics;
  return true;
}

}  // namespace legged_robot_state_estimator
