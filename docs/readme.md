

cd /home/yonsvm/code/robot-infer
./build.sh
cd build/
./run_sim.sh

---
redis-cli -h 192.168.110.91

---
cd ~/dev_ws
colcon build

ros2 run vla_x1 topic_jointstate_sub
ros2 run vla_x1 topic_jointcommand_pub


---





robot-infer/src/module/dcu_driver_module/src/dcu_driver_module.cc

line 52:
sub_joint_cmd_ = core_.GetChannelHandle().GetSubscriber("/joint_cmd");
use call back fun:

JointCmdCallback  add log


## ControlModule::MainLoop()
robot-infer/src/module/control_module/src/control_module.cc
ControlModule::MainLoop()
184  aimrt::channel::Publish zhushi

## redis send test

robot-infer/src/module/control_module/src/control_module.cc
my thread start:
redis_thread_ = std::thread(&ControlModule::RedisToRosThread, this);

void ControlModule::RedisToRosThread()
thread shi xian.

cd build
nohup ./run_sim.sh > 1.log  2>&1 &

rpush joint_cmd '{"name":["left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint"],"position":[0.55555,0.55555,0.55555],"velocity":[0.0,0.0,0.0],"effort":[0.1,0.1,0.1],"stiffness":[],"damping":[]}'

# zhushi mainloop publish and send cmd:
rpush joint_cmd '{"name":["right_shoulder_pitch_joint","right_shoulder_yaw_joint","right_elbow_pitch_joint"],"position":[0.0,0.0,200.0],"velocity":[1.0,1.0,1.0],"effort":[1.0,1.0,1.0],"stiffness":[300,30,300],"damping":[0.6,0.1,0.6]}'



## joint_cmd values
demo value :robot-infer/src/module/control_module/cfg/rl_x1_sim.yaml


robot-infer/doc/dcu_driver_module/dcu_driver_module.zh_CN.md:

/joint_cmd 使用关节名称作为索引，其数据类型为`力位混合控制`专用，所有的旋转关节都支持此种控制类型。

***请注意：对于夹爪关节，只有 position 和 effort 字段生效，其他字段需要保持为0。***

- 下行控制：position 单位为`百分比`，数值范围 0 - 1.0，0代表完全夹紧；effort 代表最大电流的百分比，数值范围 0 - 1，最大值于夹爪中配置。

- 上行状态：position 和 effort 意义保持不变。velocity 为最大速度的`百分比`，数值范围 0 - 1.0。


# 夹爪 key
仿真环境state中没有夹爪
robot-infer/src/module/dcu_driver_module/cfg/dcu_x1.yaml