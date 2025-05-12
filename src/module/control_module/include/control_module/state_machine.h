#ifndef CONTROL_MODULE_STATE_MACHINE_H_
#define CONTROL_MODULE_STATE_MACHINE_H_

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
// #include <mutex>
#include <shared_mutex>

namespace xyber_x1_infer::rl_control_module {

struct State {
  State(const std::string &state_name, const YAML::Node &node) {
    state_name_ = state_name;
    trigger_topic_ = node["trigger_topic"].as<std::string>();
    pre_states_ = node["pre_states"].as<std::vector<std::string>>();
    controllers_ = node["controllers"].as<std::vector<std::string>>();
  }

  std::string state_name_;
  std::string trigger_topic_;
  std::vector<std::string> pre_states_;
  std::vector<std::string> controllers_;
};

class StateMachine {
 public:
  StateMachine() = default;
  ~StateMachine() = default;

  void Init(const YAML::Node &node) {
    trigger_state_map_.clear();
    for (auto iter = node.begin(); iter != node.end(); iter++) {
      State state(iter->first.as<std::string>(), iter->second);
      trigger_state_map_[iter->second["trigger_topic"].as<std::string>()].push_back(state);
    }

    // 初始化状态，将 yaml 中第一个状态作为初始状态
    current_state_name_ = node.begin()->first.as<std::string>();
    controllers_ = trigger_state_map_[node.begin()->second["trigger_topic"].as<std::string>()][0].controllers_;
  }

  bool OnEvent(const std::string &trigger_topic) {
    for (const auto& to_trans_state : trigger_state_map_[trigger_topic]) {
      auto iter = std::find(to_trans_state.pre_states_.begin(), to_trans_state.pre_states_.end(), current_state_name_);

      if (iter != to_trans_state.pre_states_.end()) {
        current_state_name_ = to_trans_state.state_name_;
        std::unique_lock<std::shared_mutex> unique_lock(controllers_mutex_);
        controllers_ = to_trans_state.controllers_;
        return true;
      }
    }
    return false;
  }

  std::string GetCurrentState() {
    return current_state_name_;
  }

  std::vector<std::string> GetCurrentControllerNames() {
    std::shared_lock<std::shared_mutex> shared_lock(controllers_mutex_);
    return controllers_;
  }

 private:
  std::string current_state_name_;
  std::map<std::string, std::string> state_trigger_map_;
  std::map<std::string, std::vector<State>> trigger_state_map_;
  mutable std::shared_mutex controllers_mutex_;
  std::vector<std::string> controllers_;
};

}  // namespace xyber_x1_infer::rl_control_module

#endif  // CONTROL_MODULE_STATE_MACHINE_H_
