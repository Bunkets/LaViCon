# 四轮四驱底盘 相关 ROS 包

ROS 包 消息文件说明

## four_wheel_pt_msg.msg

四轮四驱底盘 原始数据

uint8[] cmd_81_state : 总压 电流 SOC

uint8[] cmd_87_software : 单体最高最低电压

uint8[] cmd_88_hardware : 单体最高最低温度

uint8[] cmd_89_param : 充放电 MOS管状态

uint8[] cmd_8A_date : 状态信息

uint8[] cmd_B0_drive_full : 电池故障状态

uint8[] cmd_B1_drive_base : 电池故障状态
