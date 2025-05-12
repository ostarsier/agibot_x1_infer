#include "control_module/vla_controller.h"
#include <iostream>
#include <fstream>
#include <hiredis/hiredis.h>
#include <nlohmann/json.hpp>
#include "aimrt_module_cpp_interface/logger/logger.h"
#include "control_module/global.h"

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
  
  // 只保留position数据的初始化，其他的velocity、effort、stiffness、damping都删除
  
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
          AIMRT_INFO("Redis重连失败: {}", redis_ctx_->errstr);
          redisFree(redis_ctx_);
          redis_ctx_ = nullptr;
        } else {
          AIMRT_INFO("Redis重连失败: 无法分配redis上下文");
        }
      }
    }
    
    std::string joint_data;
    bool has_data = false;
    
    // 使用RPOP从列表中获取数据（非阻塞）
    if (redis_ctx_ && !redis_ctx_->err) {
      redisReply* reply = (redisReply*)redisCommand(redis_ctx_, "RPOP %s", redis_key_.c_str());
      if (reply != nullptr) {
        // RPOP直接返回值，如果列表为空则返回nil
        if (reply->type == REDIS_REPLY_STRING) {
          joint_data = reply->str;
          has_data = true;
          AIMRT_INFO("Redis获取到数据: {}", joint_data);
          freeReplyObject(reply);
        } else if (reply->type == REDIS_REPLY_NIL) {
          freeReplyObject(reply);
          joint_cmd.name.clear(); // 将 name 设置为空列表,就不会覆盖掉对应关节的值 
          return joint_cmd; // 返回默认值初始化的命令
        } else {
          // 处理其他可能的回复类型
          freeReplyObject(reply);
        }
      } else {
        AIMRT_INFO("Redis RPOP错误: {}", (redis_ctx_->errstr ? redis_ctx_->errstr : "未知错误"));
      }
    }
    
    if (has_data) {
      // 解析Redis中的数据，格式为JSON数组，顺序与joint_list一致
      try {
        // 使用nlohmann/json解析JSON数据
        json joint_array = json::parse(joint_data);
        
        // 确保解析的数据是一个数组
        if (!joint_array.is_array()) {
          std::cerr << "Redis数据格式错误: 期望JSON数组，实际获取: " << joint_data << std::endl;
          // 使用默认值
          for (size_t ii = 0; ii < joint_names_.size(); ii++) {
            joint_cmd.position[ii] = joint_conf_.init_state(ii);
          }
          return joint_cmd;
        }
        
        // 检查数组大小
        if (joint_array.size() != joint_names_.size()) {
          std::cerr << "Redis数据大小不匹配: 期望 " << joint_names_.size() 
                    << " 个元素，实际获取 " << joint_array.size() << " 个元素" << std::endl;
          // 使用默认值
          for (size_t ii = 0; ii < joint_names_.size(); ii++) {
            joint_cmd.position[ii] = joint_conf_.init_state(ii);
          }
          return joint_cmd;
        }
        
        // 遍历JSON数组，按照joint_list的顺序解析数据
        for (size_t i = 0; i < joint_array.size(); ++i) {
          // 获取关节名称
          std::string joint_name = joint_names_[i];
          
          // 获取关节位置值
          double position_value = joint_array[i].get<double>();
          
          // 输出调试信息
          std::cout << "Redis获取关节数据: " << joint_names_[i] << " = " << position_value << std::endl;
          
          // 直接使用数组索引i作为关节索引
          joint_cmd.position[i] = position_value;
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
