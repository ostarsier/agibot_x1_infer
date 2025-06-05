/home/yonsvm/code/agibot_x1_infer/src/module/dcu_driver_module/cfg/dcu_x1.yaml

顶部关节列表中有夹爪关节名字，项目中搜索夹爪有对应值如何设置


# 上传
scp -r /home/yonsvm/code/agibot_x1_infer/src robot@192.168.110.247:/home/robot/cjd/agibot_x1_infer_vla/

ssh robot@192.168.110.247



/joint_cmd 使用关节名称作为索引，其数据类型为`力位混合控制`专用，所有的旋转关节都支持此种控制类型。

***请注意：对于夹爪关节，只有 position 和 effort 字段生效，其他字段需要保持为0。***

- 下行控制：position 单位为`百分比`，数值范围 0 - 1.0，0代表完全夹紧；effort 代表最大电流的百分比，数值范围 0 - 1，最大值于夹爪中配置。

- 上行状态：position 和 effort 意义保持不变。velocity 为最大速度的`百分比`，数值范围 0 - 1.0。

