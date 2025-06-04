// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "dcu_driver_module/dcu_driver_module.h"

#include <ctime>
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "my_ros2_proto/msg/joy_stick_data.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace xyber;
using namespace std::chrono_literals;

namespace xyber_x1_infer::dcu_driver_module {

bool DcuDriverModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Load cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    YAML::Node cfg_node = YAML::LoadFile(file_path.data());
    imu_dcu_name_ = cfg_node["imu_dcu_name"].as<std::string>();
    actuator_debug_ = cfg_node["actuator_debug"].as<bool>();
    enable_actuator_ = cfg_node["enable_actuator"].as<bool>();
    publish_frequecy_ = cfg_node["publish_frequecy"].as<double>();
    joint_name_list_ = cfg_node["joint_list"].as<std::vector<std::string>>();
    AIMRT_INFO("Initialized joint_name_list_ with {} joints:", joint_name_list_.size());
    for (const auto& joint_name : joint_name_list_) {
      AIMRT_INFO("- {}", joint_name);
    }
    actuator_name_list_ = cfg_node["actuator_list"].as<std::vector<std::string>>();

    // Init DCU SDK
    InitDcu(cfg_node);

    // Init Transmission
    InitTransmission(cfg_node);

    // Prepare publisher
    pub_imu_ = core_.GetChannelHandle().GetPublisher("/imu/data");
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::Imu>(pub_imu_);

    pub_joint_state_ = core_.GetChannelHandle().GetPublisher("/joint_states");
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::JointState>(pub_joint_state_);

    pub_actuator_cmd_ = core_.GetChannelHandle().GetPublisher("/actuator_cmd");
    aimrt::channel::RegisterPublishType<my_ros2_proto::msg::JointCommand>(pub_actuator_cmd_);

    pub_actuator_state_ = core_.GetChannelHandle().GetPublisher("/actuator_states");
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::JointState>(pub_actuator_state_);

    // Initialize subscriber
    sub_joint_cmd_ = core_.GetChannelHandle().GetSubscriber("/joint_cmd");
    aimrt::channel::Subscribe<my_ros2_proto::msg::JointCommand>(
        sub_joint_cmd_, std::bind(&DcuDriverModule::JointCmdCallback, this, std::placeholders::_1));

    AIMRT_INFO("Init succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  return true;
}

bool DcuDriverModule::Start() {
  is_running_ = true;
  publish_thread_ = std::thread(&DcuDriverModule::PublishLoop, this);

  return true;
}

void DcuDriverModule::Shutdown() {
  is_running_ = false;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }

  xyber_ctrl_->DisableAllActuator();
  xyber_ctrl_->Stop();
}

bool DcuDriverModule::InitDcu(YAML::Node& cfg_node) {
  // read config param
  ecat_cfg_ = cfg_node["ethercat"].as<YAML::EthercatConfig>();
  dcu_network_cfg_ = cfg_node["dcu_network"].as<YAML::DcuNetworkConfig>();

  // create XyberController
  xyber_ctrl_.reset(XyberController::GetInstance());

  // create dcu and attch actuators to it
  for (const auto& dcu_cfg : dcu_network_cfg_) {
    if (!dcu_cfg.enable) continue;
    bool ret = xyber_ctrl_->CreateDcu(dcu_cfg.name, dcu_cfg.ecat_id);
    AIMRT_CHECK_ERROR_THROW(ret, "DCU {} register failed.", dcu_cfg.name);

    for (size_t ch = 0; ch <= (size_t)CtrlChannel::CTRL_CH3; ch++) {
      for (const auto& actr : dcu_cfg.ch[ch]) {
        if (std::find(actuator_name_list_.begin(), actuator_name_list_.end(), actr.name) ==
            actuator_name_list_.end()) {
          AIMRT_WARN("Actuator {} not found in actuator_list. ignored.", actr.name);
          continue;
        }
        ret = xyber_ctrl_->AttachActuator(dcu_cfg.name, (CtrlChannel)ch, StringToType(actr.type),
                                          actr.name, actr.can_id);
        AIMRT_CHECK_ERROR_THROW(ret, "Actuator {} register failed.", actr.name);
      }
    }
  }

  // setup ethercat stuff
  xyber_ctrl_->SetRealtime(ecat_cfg_.rt_priority, ecat_cfg_.bind_cpu);
  bool ret = xyber_ctrl_->Start(ecat_cfg_.ifname, ecat_cfg_.cycle_time_ns, ecat_cfg_.enable_dc);
  if (!ret) {
    AIMRT_ERROR_THROW("XyberController start failed.");
    return false;
  }

  // enable actuator and imu
  if (enable_actuator_) {
    for (const auto name : actuator_name_list_) {
      if (!xyber_ctrl_->EnableActuator(name)) {
        ret = false;
      }
    }
    if (!ret) {
      AIMRT_ERROR_THROW("EnableActuator failed.");
      return false;
    }

    // update actuator cmd cache
    for (auto& name : actuator_name_list_) {
      xyber_ctrl_->SetMitCmd(name, 0, 0, 0, 0, 0);
    }
  }
  for (const auto& dcu_cfg : dcu_network_cfg_) {
    if (!dcu_cfg.imu_enable) continue;
    AIMRT_DEBUG("DCU {} imu enabled.", dcu_cfg.name);
    xyber_ctrl_->ApplyDcuImuOffset(dcu_cfg.name);
  }

  return true;
}

bool DcuDriverModule::InitTransmission(YAML::Node& cfg_node) {
  auto trans_node = cfg_node["transmission"];

  // create data space
  for (const auto& name : joint_name_list_) {
    joint_data_space_[name];
  }
  for (const auto& name : actuator_name_list_) {
    actuator_data_space_[name];
  }

  // bind joint and actuator
  for (const auto& node : trans_node) {
    std::string name = node["name"].as<std::string>();
    std::string type = node["type"].as<std::string>();

    if (type == "SimpleTransmission") {
      double direction = node["direction"].as<double>();
      std::string joint = node["joint"].as<std::string>();
      std::string actuator = node["actuator"].as<std::string>();

      const auto& joint_it = joint_data_space_.find(joint);
      if (joint_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint);
        continue;
      }
      const auto& actuator_it = actuator_data_space_.find(actuator);
      if (actuator_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator);
        continue;
      }

      JointHandle joint_handle;
      joint_handle.handle = &joint_it->second;

      ActuatorHandle actuator_handle;
      actuator_handle.direction = direction;
      actuator_handle.handle = &actuator_it->second;

      auto trans = new SimpleTransmission(name, actuator_handle, joint_handle);
      transmission_.RegisterTransmission(trans);

      // AIMRT_DEBUG("Add Transmission {}, type {}, bind actuator {}, joint {} .", name, type,
      //             actuator, joint);

    } else if (type == "LeftAnkleParallelTransmission" ||
               type == "RightAnkleParallelTransmission") {
      double direction_left = node["direction_left"].as<double>();
      double direction_right = node["direction_right"].as<double>();
      std::string param_path = node["param_path"].as<std::string>();
      std::string joint_pitch = node["joint_pitch"].as<std::string>();
      std::string joint_roll = node["joint_roll"].as<std::string>();
      std::string actuator_left = node["actuator_left"].as<std::string>();
      std::string actuator_right = node["actuator_right"].as<std::string>();

      const auto& joint_pitch_it = joint_data_space_.find(joint_pitch);
      if (joint_pitch_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_pitch);
        continue;
      }
      const auto& joint_roll_it = joint_data_space_.find(joint_roll);
      if (joint_roll_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_roll);
        continue;
      }
      const auto& actuator_left_it = actuator_data_space_.find(actuator_left);
      if (actuator_left_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_left);
        continue;
      }
      const auto& actuator_right_it = actuator_data_space_.find(actuator_right);
      if (actuator_right_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_right);
        continue;
      }

      JointHandle joint_pitch_data;
      joint_pitch_data.handle = &joint_pitch_it->second;

      JointHandle joint_roll_data;
      joint_roll_data.handle = &joint_roll_it->second;

      ActuatorHandle actr_left_data;
      actr_left_data.direction = direction_left;
      actr_left_data.handle = &actuator_left_it->second;

      ActuatorHandle actr_right_data;
      actr_right_data.direction = direction_right;
      actr_right_data.handle = &actuator_right_it->second;

      if (type == "LeftAnkleParallelTransmission") {
        auto trans = new LeftAnkleParallelTransmission(
            name, param_path, actr_left_data, actr_right_data, joint_pitch_data, joint_roll_data);
        transmission_.RegisterTransmission(trans);
      } else {
        auto trans = new RightAnkleParallelTransmission(
            name, param_path, actr_left_data, actr_right_data, joint_pitch_data, joint_roll_data);
        transmission_.RegisterTransmission(trans);
      }
      // AIMRT_DEBUG(
      //     "Add Transmission {}, type {}, bind actr left {}, actr right {}, joint pitch {},
      //     joint roll {} .", name, type, actuator_left, actuator_right, joint_pitch,
      //     joint_roll);
    } else if (type == "LeftWristParallelTransmission" ||
               type == "RightWristParallelTransmission") {
      double direction_left = node["direction_left"].as<double>();
      double direction_right = node["direction_right"].as<double>();
      std::string param_path = node["param_path"].as<std::string>();
      std::string joint_pitch = node["joint_pitch"].as<std::string>();
      std::string joint_roll = node["joint_roll"].as<std::string>();
      std::string actuator_left = node["actuator_left"].as<std::string>();
      std::string actuator_right = node["actuator_right"].as<std::string>();

      const auto& joint_pitch_it = joint_data_space_.find(joint_pitch);
      if (joint_pitch_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_pitch);
        continue;
      }
      const auto& joint_roll_it = joint_data_space_.find(joint_roll);
      if (joint_roll_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_roll);
        continue;
      }
      const auto& actuator_left_it = actuator_data_space_.find(actuator_left);
      if (actuator_left_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_left);
        continue;
      }
      const auto& actuator_right_it = actuator_data_space_.find(actuator_right);
      if (actuator_right_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_right);
        continue;
      }

      JointHandle joint_pitch_data;
      joint_pitch_data.handle = &joint_pitch_it->second;

      JointHandle joint_roll_data;
      joint_roll_data.handle = &joint_roll_it->second;

      ActuatorHandle actr_left_data;
      actr_left_data.direction = direction_left;
      actr_left_data.handle = &actuator_left_it->second;

      ActuatorHandle actr_right_data;
      actr_right_data.direction = direction_right;
      actr_right_data.handle = &actuator_right_it->second;

      if (type == "LeftWristParallelTransmission") {
        auto trans = new LeftWristParallelTransmission(
            name, param_path, actr_left_data, actr_right_data, joint_pitch_data, joint_roll_data);
        transmission_.RegisterTransmission(trans);
      } else {
        auto trans = new RightWristParallelTransmission(
            name, param_path, actr_left_data, actr_right_data, joint_pitch_data, joint_roll_data);
        transmission_.RegisterTransmission(trans);
      }
    } else if (type == "LumbarParallelTransmission") {
      double direction_left = node["direction_left"].as<double>();
      double direction_right = node["direction_right"].as<double>();
      std::string param_path = node["param_path"].as<std::string>();
      std::string joint_pitch = node["joint_pitch"].as<std::string>();
      std::string joint_roll = node["joint_roll"].as<std::string>();
      std::string actuator_left = node["actuator_left"].as<std::string>();
      std::string actuator_right = node["actuator_right"].as<std::string>();

      const auto& joint_pitch_it = joint_data_space_.find(joint_pitch);
      if (joint_pitch_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_pitch);
        continue;
      }
      const auto& joint_roll_it = joint_data_space_.find(joint_roll);
      if (joint_roll_it == joint_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find joint {}, ignored.", name, joint_roll);
        continue;
      }
      const auto& actuator_left_it = actuator_data_space_.find(actuator_left);
      if (actuator_left_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_left);
        continue;
      }
      const auto& actuator_right_it = actuator_data_space_.find(actuator_right);
      if (actuator_right_it == actuator_data_space_.end()) {
        AIMRT_WARN("Transmission {} can`t find actuator {}, ignored.", name, actuator_right);
        continue;
      }

      JointHandle joint_pitch_data;
      joint_pitch_data.handle = &joint_pitch_it->second;

      JointHandle joint_roll_data;
      joint_roll_data.handle = &joint_roll_it->second;

      ActuatorHandle actr_left_data;
      actr_left_data.direction = direction_left;
      actr_left_data.handle = &actuator_left_it->second;

      ActuatorHandle actr_right_data;
      actr_right_data.direction = direction_right;
      actr_right_data.handle = &actuator_right_it->second;

      auto trans = new LumbarParallelTransmission(name, param_path, actr_left_data, actr_right_data,
                                                  joint_pitch_data, joint_roll_data);
      transmission_.RegisterTransmission(trans);
    } else {
      AIMRT_ERROR_THROW("Transmission {} type {} not support.", name, type);
      return false;
    }
  }

  return true;
}

void DcuDriverModule::PublishLoop() {
  // create publish proxy
  aimrt::channel::PublisherProxy<sensor_msgs::msg::Imu> pub_imu(pub_imu_);
  aimrt::channel::PublisherProxy<sensor_msgs::msg::JointState> pub_joint_state(pub_joint_state_);
  aimrt::channel::PublisherProxy<sensor_msgs::msg::JointState> pub_actuator_state(
      pub_actuator_state_);
  aimrt::channel::PublisherProxy<my_ros2_proto::msg::JointCommand> pub_actuator_cmd(
      pub_actuator_cmd_);

  sensor_msgs::msg::Imu imu_msg;
  sensor_msgs::msg::JointState js_msg;
  // for (const auto& [name, data] : joint_data_space_) {
  //   js_msg.name.push_back(name);
  // }
  js_msg.name = joint_name_list_;
  js_msg.effort.resize(js_msg.name.size());
  js_msg.velocity.resize(js_msg.name.size());
  js_msg.position.resize(js_msg.name.size());

  sensor_msgs::msg::JointState actr_state;
  actr_state.name = actuator_name_list_;
  actr_state.effort.resize(actr_state.name.size());
  actr_state.velocity.resize(actr_state.name.size());
  actr_state.position.resize(actr_state.name.size());

  my_ros2_proto::msg::JointCommand actr_cmd;
  actr_cmd.name = actuator_name_list_;
  actr_cmd.effort.resize(actr_cmd.name.size());
  actr_cmd.velocity.resize(actr_cmd.name.size());
  actr_cmd.position.resize(actr_cmd.name.size());
  actr_cmd.stiffness.resize(actr_cmd.name.size());
  actr_cmd.damping.resize(actr_cmd.name.size());

  auto period = std::chrono::nanoseconds((uint64_t)(1 / publish_frequecy_ * 1000000000));
  auto next_loop_time = std::chrono::steady_clock::now();
  while (is_running_) {
    // get time
    timeval now;
    gettimeofday(&now, NULL);

    builtin_interfaces::msg::Time stamp;
    stamp.sec = now.tv_sec;
    stamp.nanosec = now.tv_usec * 1000;

    // refresh actuator state data
    for (auto& [name, data] : actuator_data_space_) {
      data.state.effort = xyber_ctrl_->GetEffort(name);
      data.state.velocity = xyber_ctrl_->GetVelocity(name);
      data.state.position = xyber_ctrl_->GetPosition(name);
    }
    // transmission
    {
      std::lock_guard<std::mutex> lock(rw_mtx_);
      transmission_.TransformActuatorToJoint();
    }
    // publish joint state
    // uint32_t i = 0;
    // for (const auto& [name, data] : joint_data_space_) { // TODO: RL module needs fixed js
    //   js_msg.effort[i] = data.state.effort;
    //   js_msg.velocity[i] = data.state.velocity;
    //   js_msg.position[i] = data.state.position;
    //   i++;
    // }
    for (size_t i = 0; i < joint_name_list_.size(); i++) {
      const auto& it = joint_data_space_.find(js_msg.name[i]);
      js_msg.effort[i] = it->second.state.effort;
      js_msg.velocity[i] = it->second.state.velocity;
      js_msg.position[i] = it->second.state.position;
    }
    js_msg.header.stamp = stamp;
    pub_joint_state.Publish(js_msg);

    // pub actuator data for debug
    if (actuator_debug_) {
      for (size_t i = 0; i < actuator_name_list_.size(); i++) {
        auto it = actuator_data_space_.find(actuator_name_list_[i]);

        actr_state.effort[i] = it->second.state.effort;
        actr_state.velocity[i] = it->second.state.velocity;
        actr_state.position[i] = it->second.state.position;

        actr_cmd.effort[i] = it->second.cmd.effort;
        actr_cmd.velocity[i] = it->second.cmd.velocity;
        actr_cmd.position[i] = it->second.cmd.position;
        actr_cmd.stiffness[i] = it->second.cmd.kp;
        actr_cmd.damping[i] = it->second.cmd.kd;
      }
      pub_actuator_cmd.Publish(actr_cmd);
      pub_actuator_state.Publish(actr_state);
    }

    // publish imu
    DcuImu imu = xyber_ctrl_->GetDcuImuData(imu_dcu_name_);
    imu_msg.angular_velocity.x = imu.gyro[0] / 180 * M_PI;
    imu_msg.angular_velocity.y = imu.gyro[1] / 180 * M_PI;
    imu_msg.angular_velocity.z = imu.gyro[2] / 180 * M_PI;
    imu_msg.linear_acceleration.x = imu.acc[0];
    imu_msg.linear_acceleration.y = imu.acc[1];
    imu_msg.linear_acceleration.z = imu.acc[2];
    imu_msg.orientation.w = imu.quat[0];
    imu_msg.orientation.x = imu.quat[1];
    imu_msg.orientation.y = imu.quat[2];
    imu_msg.orientation.z = imu.quat[3];
    imu_msg.header.stamp = stamp;
    pub_imu.Publish(imu_msg);

    next_loop_time += period;
    std::this_thread::sleep_until(next_loop_time);
  }
}

void DcuDriverModule::JointCmdCallback(
    const std::shared_ptr<const my_ros2_proto::msg::JointCommand>& msg) {
  AIMRT_INFO("Received joint cmd data: {}", my_ros2_proto::msg::to_yaml(*msg));
  if (!is_running_) return;

  // cache cmd data
  for (size_t i = 0; i < msg->name.size(); i++) {
    auto it = joint_data_space_.find(msg->name[i]);
    if (it == joint_data_space_.end()) {
      AIMRT_WARN("JointCmdCallback joint {} not found.", msg->name[i]);
      AIMRT_WARN("num {}", i);

      continue;
    }
    it->second.cmd.effort = msg->effort[i];
    it->second.cmd.velocity = msg->velocity[i];
    it->second.cmd.position = msg->position[i];
    it->second.cmd.kp = msg->stiffness[i];
    it->second.cmd.kd = msg->damping[i];
  }
  // transmission
  {
    std::lock_guard<std::mutex> lock(rw_mtx_);
    transmission_.TransformJointToActuator();
  }
  // send to actuator
  for (const auto& [name, data] : actuator_data_space_) {
    // AIMRT_DEBUG("Send actuator {} pos {} vel {} tor {} kp {} kd {}.", name, data.cmd.position,
    //             data.cmd.velocity, data.cmd.effort, data.cmd.kp, data.cmd.kd);
    xyber_ctrl_->SetMitCmd(name, data.cmd.position, data.cmd.velocity, data.cmd.effort, data.cmd.kp,
                           data.cmd.kd);
  }
}

}  // namespace xyber_x1_infer::dcu_driver_module
