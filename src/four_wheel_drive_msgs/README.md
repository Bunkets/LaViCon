# 四轮四驱底盘 相关 ROS 包

ROS包 消息文件说明

## 消息结构

```
four_wheel_drive_base_msg
├── float32 front_left_current
├── float32 front_right_current
├── float32 back_left_current
├── float32 back_right_current
├── float32 front_voltage
└── float32 back_voltage

four_wheel_drive_full_msg
├── struct four_wheel_drive_fb_lr_msg front_left
│   ├── struct four_wheel_drive_error_msg error
│   │   ├── bool under_voltage
│   │   ├── bool position_fault
│   │   ├── bool hall_fault
│   │   ├── bool overcurrent
│   │   ├── bool overload
│   │   ├── bool overheating
│   │   ├── bool speed_deviation
│   │   └── bool free_wheeling_fault
│   ├── struct four_wheel_drive_state_msg state
│   │   ├── bool start
│   │   ├── bool running
│   │   ├── bool speed_reach
│   │   ├── bool position_reach
│   │   ├── bool braking_output
│   │   ├── bool exceeded_overload
│   │   ├── bool error_warning
│   │   ├── bool reverse_stall
│   │   └── bool forward_stall
│   ├── float32 current
│   └── float32 max_current
├── struct four_wheel_drive_fb_lr_msg front_right
│   ├── struct four_wheel_drive_error_msg error
│   │   ├── bool under_voltage
│   │   ├── bool position_fault
│   │   ├── bool hall_fault
│   │   ├── bool overcurrent
│   │   ├── bool overload
│   │   ├── bool overheating
│   │   ├── bool speed_deviation
│   │   └── bool free_wheeling_fault
│   ├── struct four_wheel_drive_state_msg state
│   │   ├── bool start
│   │   ├── bool running
│   │   ├── bool speed_reach
│   │   ├── bool position_reach
│   │   ├── bool braking_output
│   │   ├── bool exceeded_overload
│   │   ├── bool error_warning
│   │   ├── bool reverse_stall
│   │   └── bool forward_stall
│   ├── float32 current
│   └── float32 max_current
├── struct four_wheel_drive_fb_lr_msg back_left
│   ├── struct four_wheel_drive_error_msg error
│   │   ├── bool under_voltage
│   │   ├── bool position_fault
│   │   ├── bool hall_fault
│   │   ├── bool overcurrent
│   │   ├── bool overload
│   │   ├── bool overheating
│   │   ├── bool speed_deviation
│   │   └── bool free_wheeling_fault
│   ├── struct four_wheel_drive_state_msg state
│   │   ├── bool start
│   │   ├── bool running
│   │   ├── bool speed_reach
│   │   ├── bool position_reach
│   │   ├── bool braking_output
│   │   ├── bool exceeded_overload
│   │   ├── bool error_warning
│   │   ├── bool reverse_stall
│   │   └── bool forward_stall
│   ├── float32 current
│   └── float32 max_current
├── struct four_wheel_drive_fb_lr_msg back_right
│   ├── struct four_wheel_drive_error_msg error
│   │   ├── bool under_voltage
│   │   ├── bool position_fault
│   │   ├── bool hall_fault
│   │   ├── bool overcurrent
│   │   ├── bool overload
│   │   ├── bool overheating
│   │   ├── bool speed_deviation
│   │   └── bool free_wheeling_fault
│   ├── struct four_wheel_drive_state_msg state
│   │   ├── bool start
│   │   ├── bool running
│   │   ├── bool speed_reach
│   │   ├── bool position_reach
│   │   ├── bool braking_output
│   │   ├── bool exceeded_overload
│   │   ├── bool error_warning
│   │   ├── bool reverse_stall
│   │   └── bool forward_stall
│   ├── float32 current
│   └── float32 max_current
├── struct four_wheel_drive_fb_msg front
│   ├── float32 temperature
│   └── float32 voltage
└── struct four_wheel_drive_fb_msg back
    ├── float32 temperature
    └── float32 voltage
```

## four_wheel_drive_base_msg.msg

四驱底盘 驱动器 基础数据包

| 类型     | 名称                   | 取值     | 单位 | 备注  |
|---------|-----------------------|----------|----|------|
| float32 | front\_left\_current  | \-\-\-\- |  A | 前左电流 |
| float32 | front\_right\_current | \-\-\-\- |  A | 前右电流 |
| float32 | back\_left\_current   | \-\-\-\- |  A | 后左电流 |
| float32 | back\_right\_current  | \-\-\-\- |  A | 后右电流 |
| float32 | front\_voltage        | \-\-\-\- |  V | 前电压  |
| float32 | back\_voltage         | \-\-\-\- |  V | 后电压  |

## four_wheel_drive_full_msg.msg

四驱底盘 驱动器 完整数据包

| 类型                                    | 名称          | 取值     | 单位      | 备注 |
|----------------------------------------|--------------|----------|----------|----|
| struct four\_wheel\_drive\_fb\_lr\_msg | front\_left  | \-\-\-\- | \-\-\-\- | 前左 |
| struct four\_wheel\_drive\_fb\_lr\_msg | front\_right | \-\-\-\- | \-\-\-\- | 前右 |
| struct four\_wheel\_drive\_fb\_lr\_msg | back\_left   | \-\-\-\- | \-\-\-\- | 后左 |
| struct four\_wheel\_drive\_fb\_lr\_msg | back\_right  | \-\-\-\- | \-\-\-\- | 后右 |
| struct four\_wheel\_drive\_fb\_msg     | front        | \-\-\-\- | \-\-\-\- | 前  |
| struct four\_wheel\_drive\_fb\_msg     | back         | \-\-\-\- | \-\-\-\- | 后  |

## four_wheel_drive_fb_lr_msg.msg

四驱底盘 驱动器 前左 前右 后左 后右 数据包

| 类型                                   | 名称          | 取值     | 单位      | 备注   |
|---------------------------------------|--------------|----------|----------|------|
| struct four\_wheel\_drive\_error\_msg | error        | \-\-\-\- | \-\-\-\- | 错误   |
| struct four\_wheel\_drive\_state\_msg | state        | \-\-\-\- | \-\-\-\- | 状态   |
| float32                               | current      | \-\-\-\- | A        | 电流   |
| float32                               | max\_current | \-\-\-\- | A        | 最大电流 |

## four_wheel_drive_fb_msg.msg

四驱底盘 驱动器 前 后 数据包

| 类型      | 名称          | 取值       | 单位 | 备注 |
|---------|-------------|----------|----|----|
| float32 | temperature | \-\-\-\- | ℃  | 温度 |
| float32 | voltage     | \-\-\-\- | V  | 电压 |

## four_wheel_drive_error_msg.msg

四驱底盘 驱动器 错误 数据包

| 类型   | 名称                    | 取值           | 单位       | 备注   |
|------|-----------------------|--------------|----------|------|
| bool | under\_voltage        | true / false | \-\-\-\- | 低压故障 |
| bool | position\_fault       | true / false | \-\-\-\- | 位置故障 |
| bool | hall\_fault           | true / false | \-\-\-\- | 霍尔故障 |
| bool | overcurrent           | true / false | \-\-\-\- | 过流故障 |
| bool | overload              | true / false | \-\-\-\- | 过载故障 |
| bool | overheating           | true / false | \-\-\-\- | 过热故障 |
| bool | speed\_deviation      | true / false | \-\-\-\- | 速度超差 |
| bool | free\_wheeling\_fault | true / false | \-\-\-\- | 飞车故障 |

## four_wheel_drive_state_msg.msg

四驱底盘 驱动器 状态 数据包

| 类型   | 名称                 | 取值           | 单位       | 备注   |
|------|--------------------|--------------|----------|------|
| bool | start              | true / false | \-\-\-\- | 伺服启动 |
| bool | running            | true / false | \-\-\-\- | 伺服运行 |
| bool | speed\_reach       | true / false | \-\-\-\- | 速度到达 |
| bool | position\_reach    | true / false | \-\-\-\- | 位置到达 |
| bool | braking\_output    | true / false | \-\-\-\- | 制动输出 |
| bool | exceeded\_overload | true / false | \-\-\-\- | 过载门槛 |
| bool | error\_warning     | true / false | \-\-\-\- | 错误警告 |
| bool | reverse\_stall     | true / false | \-\-\-\- | 反向堵转 |
| bool | forward\_stall     | true / false | \-\-\-\- | 正向堵转 |
