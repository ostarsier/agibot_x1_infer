# VLA控制器使用说明

## 1. 基本概念
VLA控制器（VLAController）是专门用于控制机器人右臂关节的控制器，它通过Redis获取关节位置数据，并从配置文件中获取其他参数，实现对右臂关节的精确控制。

## 2. 支持的关节
VLA控制器控制以下右臂关节：
- right_shoulder_pitch_joint
- right_shoulder_roll_joint
- right_shoulder_yaw_joint
- right_elbow_pitch_joint
- right_elbow_yaw_joint
- right_wrist_pitch_joint
- right_wrist_roll_joint

## 3. 状态切换
要切换到VLA控制模式，需要按以下步骤操作：

### 3.1 从idle状态切换到stand_&_vla状态
```bash
# 1. 先发布到zero_mode
ros2 topic pub /zero_mode std_msgs/msg/Empty "{}"

# 2. 然后发布到stand_mode
ros2 topic pub /stand_mode std_msgs/msg/Empty "{}"

# 3. 最后发布到vla_mode
ros2 topic pub /vla_mode std_msgs/msg/Empty "{}"
```

### 3.2 状态切换条件
根据配置文件，stand_&_vla状态的切换条件是：
```yaml
stand_&_vla:
  trigger_topic: /vla_mode
  pre_states: [stand, stand_&_vla]
  controllers: [pd_zero, pd_stand, vla]
```

## 4. Redis配置
VLA控制器通过Redis的list使用blpop获取关节位置数据，Redis配置如下：

### 4.1 默认配置
```cpp
// Redis配置
std::string redis_key_ = "joint_position";
std::string redis_host_ = "127.0.0.1";
int redis_port_ = 6379;
int redis_timeout_ = 1; // blpop超时时间，单位为秒
```

### 4.2 配置文件中的设置
在rl_x1_sim.yaml中，Redis配置如下：
```yaml
vla:
  redis_conf:
    host: "127.0.0.1"
    port: 6379
    key: "joint_position"
    timeout: 1
```

## 5. Redis数据格式
Redis中的数据格式为JSON对象，其中key是关节名称，value是关节位置值：

```json
{
  "right_shoulder_pitch_joint": 0.1,
  "right_shoulder_roll_joint": 0.2,
  "right_shoulder_yaw_joint": 0.3,
  "right_elbow_pitch_joint": 0.4,
  "right_elbow_yaw_joint": 0.5,
  "right_wrist_pitch_joint": 0.6,
  "right_wrist_roll_joint": 0.7
}
```

## 6. 控制器参数
VLA控制器的参数从配置文件中获取，包括初始状态、速度、力矩、刚度和阻尼值：

```yaml
vla:
  joint_list: [right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint, right_elbow_pitch_joint, right_elbow_yaw_joint, right_wrist_pitch_joint, right_wrist_roll_joint]
  init_state: [0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0]    # 初始位置
  velocity:   [0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0]    # 速度
  effort:     [0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0]    # 力矩
  stiffness:  [300.0, 300.0, 30.0,  300.0, 30.0,  30.0,  30.0]   # 刚度
  damping:    [0.6,   0.6,   0.1,   0.6,   0.1,   0.1,   0.1]    # 阻尼
```

其中，关节位置值从 Redis 中获取，其他参数从配置文件中获取。

## 7. 故障处理
1. 如果Redis连接失败，控制器会使用默认参数
2. 如果Redis键不存在，控制器会使用默认参数
3. 如果Redis数据格式错误，控制器会使用默认参数

## 8. 修改说明

### 8.1 修改Redis配置
如果需要修改Redis配置，可以在rl_x1_sim.yaml中修改：
```yaml
vla:
  redis_conf:
    host: "新的主机地址"
    port: 新的端口号
    key: "新的键名"
```

### 8.2 修改控制器参数
如果需要修改控制器参数，可以在rl_x1_sim.yaml中修改：
```yaml
vla:
  init_state: [新的初始位置参数]
  velocity: [新的速度参数]
  effort: [新的力矩参数]
  stiffness: [新的刚度参数]
  damping: [新的阻尼参数]
```

### 8.3 修改Redis数据格式
如果需要修改Redis数据格式，需要同时修改：
1. VLAController::GetJointCmdData函数中的JSON解析逻辑
2. Redis数据的写入格式

## 9. 使用示例

### 9.1 发布Redis数据
```python
import redis
import json

# 连接Redis
r = redis.Redis(host='127.0.0.1', port=6379)

# 准备JSON格式的关节数据
joint_data = {
    "right_shoulder_pitch_joint": 0.1,
    "right_shoulder_roll_joint": 0.2,
    "right_shoulder_yaw_joint": 0.3,
    "right_elbow_pitch_joint": 0.4,
    "right_elbow_yaw_joint": 0.5,
    "right_wrist_pitch_joint": 0.6,
    "right_wrist_roll_joint": 0.7
}

# 将字典转换为JSON字符串并写入Redis列表
r.rpush('joint_position', json.dumps(joint_data))
```

### 9.2 切换到VLA模式
```bash
# 1. 先发布到zero_mode
ros2 topic pub /zero_mode std_msgs/msg/Empty "{}"

# 2. 然后发布到stand_mode
ros2 topic pub /stand_mode std_msgs/msg/Empty "{}"

# 3. 最后发布到vla_mode
ros2 topic pub /vla_mode std_msgs/msg/Empty "{}"
```

## 10. 注意事项
1. 确保Redis服务正常运行
2. 确保Redis配置正确
3. 确保Redis数据格式正确
4. 在切换状态时，确保机器人处于安全位置
5. 在修改参数时，确保参数值合理
