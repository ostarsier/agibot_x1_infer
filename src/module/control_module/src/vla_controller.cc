#include "control_module/vla_controller.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
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
  // 从配置文件中获取 joint_names_
  joint_names_.clear();
  for (const auto& joint : cfg_node["joint_list"]) {
    joint_names_.push_back(joint.as<std::string>());
  }
  AIMRT_INFO("VLAController 从配置文件加载关节列表，共 {} 个关节", joint_names_.size());
  
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.resize(joint_names_.size(), 0.0);
  joint_state_data_.velocity.resize(joint_names_.size(), 0.0);
  joint_state_data_.effort.resize(joint_names_.size(), 0.0);

  // 从配置文件中获取 stiffness 和 damping
  stiffness_.clear();
  damping_.clear();
  for (const auto& stiff : cfg_node["stiffness"]) {
    stiffness_.push_back(stiff.as<double>());
  }
  for (const auto& damp : cfg_node["damping"]) {
    damping_.push_back(damp.as<double>());
  }
  AIMRT_INFO("VLAController 从配置文件加载刚度阻尼参数，共 {} 个刚度和 {} 个阻尼", 
             stiffness_.size(), damping_.size());
  
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
      AIMRT_INFO("Redis连接初始化失败: {} {}:{}", redis_ctx_->errstr, redis_host_, redis_port_);
      redisFree(redis_ctx_);
      redis_ctx_ = nullptr;
    } else {
      AIMRT_INFO("Redis连接初始化失败: 无法分配redis上下文");
    }
  } else {
    AIMRT_INFO("Redis连接初始化成功: {}:{}", redis_host_, redis_port_);
  }
}

void VLAController::RestartController() {
  // 重置Redis连接
  if (redis_ctx_) {
    redisFree(redis_ctx_);
    redis_ctx_ = nullptr;
  }
  
  // 重新初始化Redis连接
  struct timeval timeout = {redis_timeout_, 0};
  redis_ctx_ = redisConnectWithTimeout(redis_host_.c_str(), redis_port_, timeout);
  if (redis_ctx_ == nullptr || redis_ctx_->err) {
    AIMRT_INFO("重启控制器时Redis连接初始化失败");
  } else {
    AIMRT_INFO("重启控制器时Redis连接初始化成功");
  }
}

void VLAController::Update() {
  return;
}

my_ros2_proto::msg::JointCommand VLAController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand joint_cmd;
  // 默认为空列表，只有成功获取数据时才设置为joint_names_
  joint_cmd.name.clear();
  joint_cmd.position.resize(joint_names_.size(), 0.0);
  joint_cmd.velocity.resize(joint_names_.size(), 0.0);
  joint_cmd.effort.resize(joint_names_.size(), 0.0);
  // 使用已加载的刚度阻尼参数作为默认值
  joint_cmd.stiffness = stiffness_;
  joint_cmd.damping = damping_;
  
  bool success = false;
  std::string joint_data;
  
  try {
    // 检查Redis连接是否可用
    if (redis_ctx_ == nullptr || redis_ctx_->err) {
      AIMRT_INFO("Redis连接不可用，无法获取关节数据");
      return joint_cmd;
    }
    
    // 使用GET命令获取键值数据
    redisReply* reply = (redisReply*)redisCommand(redis_ctx_, "GET %s", redis_key_.c_str());
    if (reply != nullptr) {
      // GET直接返回键值，如果键不存在则返回nil
      if (reply->type == REDIS_REPLY_STRING) {
        joint_data = reply->str;
        success = true;
        AIMRT_INFO("Redis获取到数据: {}", joint_data);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      freeReplyObject(reply);
    } else {
      AIMRT_INFO("Redis GET错误: {}", (redis_ctx_->errstr ? redis_ctx_->errstr : "未知错误"));
    }
    
    if (success) {
      // 解析Redis中的数据，格式为JSON数组，顺序与joint_list一致
      try {
        // 使用nlohmann/json解析JSON数据
        json joint_array = json::parse(joint_data);
        
        // 确保解析的数据是一个数组
        if (!joint_array.is_array()) {
          AIMRT_INFO("Redis数据格式错误: 期望JSON数组，实际获取: {}", joint_data);
        } else if (joint_array.size() != joint_names_.size()) {
          // 检查数组大小
          AIMRT_INFO("Redis数据大小不匹配: 期望 {} 个元素，实际获取 {} 个元素", joint_names_.size(), joint_array.size());
        } else {
          // 清除原始的joint_cmd数据，只添加有值的关节
          joint_cmd.name.clear();
          joint_cmd.position.clear();
          joint_cmd.velocity.clear();
          joint_cmd.effort.clear();
          
          // 遍历JSON数组，按照joint_list的顺序解析数据
          for (size_t i = 0; i < joint_array.size(); ++i) {
            // 检查关节值是否为null或空字符串
            if (joint_array[i].is_null() || 
                (joint_array[i].is_string() && joint_array[i].get<std::string>().empty())) {
              // 跳过没有值的关节
              continue;
            }
            
            try {
              // 获取关节位置值
              double position_value = joint_array[i].get<double>();
              
              // 只添加有值的关节
              joint_cmd.name.push_back(joint_names_[i]);
              joint_cmd.position.push_back(position_value);
              joint_cmd.velocity.push_back(0.0);  // 速度设为0作为安全默认值
              joint_cmd.effort.push_back(0.0);    // 力矩设为0作为安全默认值
            } catch (const std::exception& e) {
              AIMRT_INFO("解析关节 {} 的值失败: {}", joint_names_[i], e.what());
              // 解析失败则跳过该关节
            }
          }
          // 设置刚度和阻尼参数
          // 根据实际添加的关节，设置对应的刚度和阻尼参数
          joint_cmd.stiffness.clear();
          joint_cmd.damping.clear();
          
          if(stiffness_.size() == joint_names_.size() && damping_.size() == joint_names_.size()) {
            // 只为添加到joint_cmd的关节设置刚度和阻尼
            for (size_t i = 0; i < joint_cmd.name.size(); ++i) {
              // 查找关节在原始joint_names_中的位置
              auto it = std::find(joint_names_.begin(), joint_names_.end(), joint_cmd.name[i]);
              if (it != joint_names_.end()) {
                size_t idx = std::distance(joint_names_.begin(), it);
                joint_cmd.stiffness.push_back(stiffness_[idx]);
                joint_cmd.damping.push_back(damping_[idx]);
              }
            }
          } else {
            AIMRT_INFO("刚度阻尼参数数量与关节数量不匹配");
          }
        }
      } catch (const std::exception& e) {
        AIMRT_INFO("Redis数据解析错误: {}", e.what());
      }
    } 
  } catch (const std::exception& e) {
    AIMRT_INFO("Redis连接错误: {}", e.what());
  }
  
  // 只有在success=true时才打印joint_cmd的日志
  if (success) {
    // 创建JSON对象来格式化joint_cmd
    json joint_data_json;
    joint_data_json["name"] = joint_cmd.name;
    joint_data_json["position"] = joint_cmd.position;
    joint_data_json["stiffness"] = joint_cmd.stiffness;
    joint_data_json["damping"] = joint_cmd.damping;
    joint_data_json["velocity"] = joint_cmd.velocity;
    joint_data_json["effort"] = joint_cmd.effort;
    AIMRT_INFO("发送 joint_cmd {}", joint_data_json.dump());
  }
  
  return joint_cmd;
}

}  // namespace xyber_x1_infer::rl_control_module
