#include "control_module/vla_controller.h"
#include <iostream>
#include <fstream>
#include <hiredis/hiredis.h>
#include <nlohmann/json.hpp>

// 使用nlohmann/json库
using json = nlohmann::json;

namespace xyber_x1_infer::rl_control_module {

VLAController::~VLAController() {
  if (redis_ctx_) {
    redisFree(redis_ctx_);
    redis_ctx_ = nullptr;
  }
}

VLAController::VLAController(const bool use_sim_handles)
    : ControllerBase(use_sim_handles) {}

void VLAController::Init(const YAML::Node &cfg_node) {
  // 初始化 joint_names_ - 只使用右臂关节
  joint_names_.clear();
  joint_names_.push_back("right_shoulder_pitch_joint");
  joint_names_.push_back("right_shoulder_roll_joint");
  joint_names_.push_back("right_shoulder_yaw_joint");
  joint_names_.push_back("right_elbow_pitch_joint");
  joint_names_.push_back("right_elbow_yaw_joint");
  joint_names_.push_back("right_wrist_pitch_joint");
  joint_names_.push_back("right_wrist_roll_joint");
  
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.resize(joint_names_.size(), 0.0);
  joint_state_data_.velocity.resize(joint_names_.size(), 0.0);
  joint_state_data_.effort.resize(joint_names_.size(), 0.0);
  
  // 初始化 joint_conf_
  joint_conf_.init_state = Eigen::Map<vector_t>(cfg_node["init_state"].as<std::vector<double>>().data(), cfg_node["init_state"].as<std::vector<double>>().size());
  
  // 读取velocity和effort值
  if (cfg_node["velocity"]) {
    joint_conf_.velocity = Eigen::Map<vector_t>(cfg_node["velocity"].as<std::vector<double>>().data(), cfg_node["velocity"].as<std::vector<double>>().size());
  } else {
    // 如果不存在，初始化为零
    joint_conf_.velocity = Eigen::VectorXd::Zero(joint_names_.size());
  }
  
  if (cfg_node["effort"]) {
    joint_conf_.effort = Eigen::Map<vector_t>(cfg_node["effort"].as<std::vector<double>>().data(), cfg_node["effort"].as<std::vector<double>>().size());
  } else {
    // 如果不存在，初始化为零
    joint_conf_.effort = Eigen::VectorXd::Zero(joint_names_.size());
  }
  
  joint_conf_.stiffness = Eigen::Map<vector_t>(cfg_node["stiffness"].as<std::vector<double>>().data(), cfg_node["stiffness"].as<std::vector<double>>().size());
  joint_conf_.damping = Eigen::Map<vector_t>(cfg_node["damping"].as<std::vector<double>>().data(), cfg_node["damping"].as<std::vector<double>>().size());
  
  // 从配置中获取Redis配置
  if (cfg_node["redis_conf"]) {
    if (cfg_node["redis_conf"]["host"]) {
      redis_host_ = cfg_node["redis_conf"]["host"].as<std::string>();
    }
    if (cfg_node["redis_conf"]["port"]) {
      redis_port_ = cfg_node["redis_conf"]["port"].as<int>();
    }
    if (cfg_node["redis_conf"]["key"]) {
      redis_key_ = cfg_node["redis_conf"]["key"].as<std::string>();
    }
    if (cfg_node["redis_conf"]["timeout"]) {
      redis_timeout_ = cfg_node["redis_conf"]["timeout"].as<int>();
    }
  }
  
  // 初始化Redis连接
  struct timeval timeout = {redis_timeout_, 0}; // 设置超时时间
  redis_ctx_ = redisConnectWithTimeout(redis_host_.c_str(), redis_port_, timeout);
  if (redis_ctx_ == nullptr || redis_ctx_->err) {
    if (redis_ctx_) {
      std::cerr << "Redis连接初始化失败: " << redis_ctx_->errstr << std::endl;
      redisFree(redis_ctx_);
      redis_ctx_ = nullptr;
    } else {
      std::cerr << "Redis连接初始化失败: 无法分配redis上下文" << std::endl;
    }
  } else {
    std::cout << "Redis连接初始化成功: " << redis_host_ << ":" << redis_port_ << std::endl;
  }
}

void VLAController::RestartController() {
  // 简单的重启逻辑，不需要太多操作
}

void VLAController::Update() {
  // 直接返回，不执行任何操作
  return;
}

my_ros2_proto::msg::JointCommand VLAController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand joint_cmd;
  joint_cmd.name = joint_names_;
  joint_cmd.position.resize(joint_names_.size(), 0.0);
  joint_cmd.velocity.resize(joint_names_.size(), 0.0);
  joint_cmd.effort.resize(joint_names_.size(), 0.0);
  joint_cmd.damping.resize(joint_names_.size(), 0.0);
  joint_cmd.stiffness.resize(joint_names_.size(), 0.0);
  
  // 从配置文件中读取刚度、阻尼、速度和力矩值
  for (size_t ii = 0; ii < joint_names_.size(); ii++) {
    joint_cmd.stiffness[ii] = joint_conf_.stiffness(ii);
    joint_cmd.damping[ii] = joint_conf_.damping(ii);
    joint_cmd.velocity[ii] = joint_conf_.velocity(ii);
    joint_cmd.effort[ii] = joint_conf_.effort(ii);
  }
  
  try {
    // 使用初始化时创建的Redis连接获取数据
    if (!redis_ctx_ || redis_ctx_->err) {
      // 如果Redis连接不存在或有错误，尝试重新创建
      if (redis_ctx_) {
        redisFree(redis_ctx_);
      }
      struct timeval timeout = {1, 0}; // 设置超时时间为1秒
      redis_ctx_ = redisConnectWithTimeout(redis_host_.c_str(), redis_port_, timeout);
      if (redis_ctx_ == nullptr || redis_ctx_->err) {
        if (redis_ctx_) {
          std::cerr << "Redis重连失败: " << redis_ctx_->errstr << std::endl;
          redisFree(redis_ctx_);
          redis_ctx_ = nullptr;
        } else {
          std::cerr << "Redis重连失败: 无法分配redis上下文" << std::endl;
        }
      }
    }
    
    std::string joint_data;
    bool has_data = false;
    
    // 使用BLPOP从列表中获取数据，超时时间为1秒
    if (redis_ctx_ && !redis_ctx_->err) {
      redisReply* reply = (redisReply*)redisCommand(redis_ctx_, "BLPOP %s 1", redis_key_.c_str());
      if (reply != nullptr) {
        // BLPOP返回一个包含两个元素的数组：键名和值
        if (reply->type == REDIS_REPLY_ARRAY && reply->elements == 2) {
          joint_data = reply->element[1]->str; // 第二个元素是值
          has_data = true;
          std::cout << "Redis获取到数据: " << joint_data << std::endl;
        }
        freeReplyObject(reply);
      } else {
        std::cerr << "Redis BLPOP错误: " << (redis_ctx_->errstr ? redis_ctx_->errstr : "未知错误") << std::endl;
      }
    }
    
    if (has_data) {
      // 解析Redis中的数据，格式为JSON对象
      try {
        // 使用nlohmann/json解析JSON数据
        json joint_json = json::parse(joint_data);
        
        // 遍历JSON对象中的所有键值对
        for (auto it = joint_json.begin(); it != joint_json.end(); ++it) {
          // 获取关节名称
          std::string joint_name = it.key();
          
          // 获取关节位置值
          double position_value = it.value().get<double>();
          
          // 输出调试信息
          std::cout << "Redis获取关节数据: " << joint_name << " = " << position_value << std::endl;
          
          // 查找关节在joint_names_中的索引
          auto joint_it = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
          if (joint_it != joint_names_.end()) {
            size_t index = std::distance(joint_names_.begin(), joint_it);
            joint_cmd.position[index] = position_value;
          }
        }
      } catch (const std::exception& e) {
        std::cerr << "Redis数据解析错误: " << e.what() << std::endl;
        // 解析错误时，使用默认值
        for (size_t ii = 0; ii < joint_names_.size(); ii++) {
          joint_cmd.position[ii] = joint_conf_.init_state(ii);
        }
      }
    } else {
      // 如果Redis中没有找到键，使用默认值
      for (size_t ii = 0; ii < joint_names_.size(); ii++) {
        joint_cmd.position[ii] = joint_conf_.init_state(ii);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Redis连接错误: " << e.what() << std::endl;
    
    // 连接错误时使用默认值
    for (size_t ii = 0; ii < joint_names_.size(); ii++) {
      joint_cmd.position[ii] = joint_conf_.init_state(ii);
    }
  }
  
  return joint_cmd;
}

}  // namespace xyber_x1_infer::rl_control_module
