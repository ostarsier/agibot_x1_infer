#pragma once
#include <memory>
#include <hiredis/hiredis.h>
#include "control_module/controller_base.h"

namespace xyber_x1_infer::rl_control_module {

class VLAController : public ControllerBase {
 public:
  VLAController(const bool use_sim_handles);
  ~VLAController();

  void Init(const YAML::Node &cfg_node) override;
  void RestartController() override;
  void Update() override;
  my_ros2_proto::msg::JointCommand GetJointCmdData() override;

 private:
  // Redis配置
  std::string redis_key_ = "joint_position";
  std::string redis_host_ = "127.0.0.1";
  int redis_port_ = 6379;
  int redis_timeout_ = 0; // blpop超时时间，0表示无限等待
  
  // Redis连接对象
  redisContext* redis_ctx_{nullptr};
};

}  // namespace xyber_x1_infer::rl_control_module
