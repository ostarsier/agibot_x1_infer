#pragma once
#include <yaml-cpp/yaml.h>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "my_ros2_proto/msg/joint_command.hpp"

#include "control_module/utilities.h"

namespace xyber_x1_infer::rl_control_module {

class ControllerBase {
 public:
  ControllerBase(const bool use_sim_handles);
  virtual ~ControllerBase() = default;

  virtual void Init(const YAML::Node &cfg_node) = 0;
  virtual void RestartController() = 0;
  virtual void SetCmdData(const geometry_msgs::msg::Twist joy_data);
  virtual void SetImuData(const sensor_msgs::msg::Imu imu_data);
  virtual void SetJointStateData(const sensor_msgs::msg::JointState joint_state_data, const std::unordered_map<std::string, int> &joint_state_index_map_);
  virtual std::vector<std::string> GetJointList();
  virtual void Update() = 0;
  virtual my_ros2_proto::msg::JointCommand GetJointCmdData() = 0;

 protected:
  bool use_sim_handles_;
  struct JointConf {
    vector_t init_state;
    vector_t velocity;
    vector_t effort;
    vector_t stiffness;
    vector_t damping;
  } joint_conf_;
  std::vector<std::string> joint_names_;
  // from ros2 topic
  mutable std::shared_mutex joy_mutex_;
  mutable std::shared_mutex imu_mutex_;
  mutable std::shared_mutex joint_state_mutex_;
  geometry_msgs::msg::Twist joy_data_;
  sensor_msgs::msg::Imu imu_data_;
  sensor_msgs::msg::JointState joint_state_data_;
  // // other
  // bool is_first_frame_{true};
};

}  // namespace xyber_x1_infer::rl_control_module
