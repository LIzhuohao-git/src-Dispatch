#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""小车参数配置中心（Web 端"参数配置"页的后端）。

架构：
  - 参数统一存放在 ~/.originbot_delivery/config/robot_params.json（用户可导出/导入）；
  - 每个参数组有 schema（中文标签/说明/类型/取值范围），Web 端据此渲染表单；
  - 保存时把参数"生成"为各子系统真正消费的配置文件，写入
    ~/.originbot_delivery/config/generated/：
      lidar.json         雷达参数 + 安装位姿（ldlidar/launch/stl06n.launch.py 启动时读取）
      imu.json           IMU 安装位姿（originbot_base/launch/robot.launch.py 读取）
      chassis.json       底盘速度标定（originbot_base/launch/robot.launch.py 读取）
      camera.json        相机安装位姿（originbot_delivery/launch/aruco.launch.py 读取）
      ekf.yaml           robot_localization EKF（odom_ekf.launch.py 读取）
      slam_toolbox.yaml  slam_toolbox 建图（delivery_server 启动时经 slam_params_file 注入）
      nav2_params.yaml   Nav2 全套（含 AMCL，启动时经 params_file 注入）
      cartographer/lds_2d.lua  Cartographer 建图（经 cartographer_config_dir 注入）
  - launch 文件每次启动都会重新读取生成文件 → 保存后"重启对应子系统"即生效，
    开机自启动也自动加载；
  - ArUco 规格（字典/边长）特殊：直接写入 markers.json，aruco_localization
    约 1 秒内热加载，无需重启。
"""

import copy
import json
import os
from pathlib import Path

import yaml

# 参数 schema。type: bool / int / float / enum
#   label  表单里显示的中文名称
#   note   中文说明（生效机理、调参建议）
#   group  同一参数组内的子分组标题（可选）
#   path   生成 yaml 时写入的目标位置（点分路径，仅 yaml 类参数组需要）
#   legacy 旧版键名（可选）。改键名后用它把用户已保存的旧值迁移过来，
#          避免升级后参数悄悄回到默认值；仅在 robot_params.json 里生效
SCHEMA = [
    {
        "id": "lidar",
        "title": "激光雷达（STL06N）",
        "apply": "保存后需重启底盘生效；开机自启动自动加载",
        "params": [
            # key 必须与 ldlidar 驱动声明的 ROS 参数名一致（见 ldlidar/include/
            # ros2_api.h 与 launch/stl06n.launch.py 读取的键名），否则 Web 端
            # 写出的 lidar.json 会被 launch 静默忽略——历史上 scan_dir /
            # enable_angle_crop 两个键就是这样一直不生效的。
            {"key": "laser_scan_dir", "type": "bool", "default": True,
             "legacy": "scan_dir",
             "label": "雷达安装方向（正装）",
             "note": "True=正装（默认，逆时针扫描）；False=倒装（吊装/倒置安装时勾选去掉）。"
                     "本项决定雷达自身角度系与车体系的对应关系，"
                     "也会改变角度裁剪的语义——改了它务必用 lidar_crop_check.py 复核"
                     "裁剪区间（加 --dir-check 时该工具会自行驱动小车转 90° 实测方向）"},
            {"key": "enable_angle_crop_func", "type": "bool", "default": False,
             "legacy": "enable_angle_crop",
             "label": "启用角度裁剪",
             "note": "把 [裁剪起始角, 裁剪结束角] 区间内的测量置 0，用于屏蔽车身/支架遮挡"},
            {"key": "angle_crop_min", "type": "float", "default": 95.0,
             "min": 0.0, "max": 360.0, "step": 1.0,
             "label": "裁剪起始角（度）",
             "note": "与结束角构成闭区间；0° 为雷达正前方，逆时针为正"},
            {"key": "angle_crop_max", "type": "float", "default": 265.0,
             "min": 0.0, "max": 360.0, "step": 1.0,
             "label": "裁剪结束角（度）",
             "note": "示例 [95,265] 表示裁掉正后方扇区"},
            {"key": "min_range", "type": "float", "default": 0.1,
             "min": 0.02, "max": 2.0, "step": 0.01,
             "label": "最小检测距离（米）",
             "note": "小于该距离的点被丢弃；STL06N 出厂 0.1，勿小于雷达盲区"},
            {"key": "max_range", "type": "float", "default": 20.0,
             "min": 0.5, "max": 30.0, "step": 0.5,
             "label": "最大检测距离（米）",
             "note": "远场噪点多时可调小（STL06N 实测有效距离约 12m），建图/避障更干净"},
            # ---- 安装位姿 ----
            # 注意：本组数值是 base_link -> laser_link 静态 TF 的唯一来源。
            # 雷达装歪/挪位只需改这里，重启底盘即可生效（老版本散落在
            # stl06n.launch.py 的硬编码 + runtime_selfcheck.py 的期望值里，
            # 三处不一致，已统一到此处）。
            {"key": "mount_x", "type": "float", "default": 0.15,
             "min": -1.0, "max": 1.0, "step": 0.01, "group": "安装位姿",
             "label": "安装位置 X（米）",
             "note": "雷达相对 base_link 的前向偏移（前方为正）。"
                     "实测确认：钢尺量雷达光心到两轮轴心的水平距离。"
                     "本项也是 odom_calib.py 旋转标定的模型输入——原地旋转时雷达"
                     "会绕 base_link 画圈，这个偏移填错会让测出的真实转角偏掉"},
            {"key": "mount_y", "type": "float", "default": 0.0,
             "min": -1.0, "max": 1.0, "step": 0.01, "group": "安装位姿",
             "label": "安装位置 Y（米）",
             "note": "左向偏移（左为正）；居中安装填 0"},
            {"key": "mount_z", "type": "float", "default": 0.08,
             "min": -1.0, "max": 1.0, "step": 0.01, "group": "安装位姿",
             "label": "安装位置 Z（米）",
             "note": "相对 base_link 的高度（base_link 距地 0.13325m）。"
                     "填错只影响 3D 可视化，不影响 2D 建图/导航"},
            {"key": "mount_roll", "type": "float", "default": 0.0,
             "min": -3.2, "max": 3.2, "step": 0.01, "group": "安装位姿",
             "label": "安装横滚（弧度）",
             "note": "绕 X 轴旋转；水平安装填 0"},
            {"key": "mount_pitch", "type": "float", "default": 0.0,
             "min": -3.2, "max": 3.2, "step": 0.01, "group": "安装位姿",
             "label": "安装俯仰（弧度）",
             "note": "绕 Y 轴旋转；水平安装填 0"},
            {"key": "mount_yaw", "type": "float", "default": 0.0,
             "min": -3.2, "max": 3.2, "step": 0.01, "group": "安装位姿",
             "label": "安装偏航（弧度）",
             "note": "绕 Z 轴旋转，0=雷达 0° 方向朝车头正前方。"
                     "配套正装（laser_scan_dir=True）时雷达 0° 即车头；"
                     "若雷达 0° 指向车尾，应同时把本项设为 3.1416 "
                     "（或改 laser_scan_dir）——用 lidar_crop_check.py 实测确认。"
                     "注意本项非 0 时标定工具会一并代入模型，无需手工换算"},
        ],
    },
    {
        "id": "imu",
        "title": "IMU 安装位姿",
        "apply": "保存后需重启底盘生效；开机自启动自动加载",
        "params": [
            {"key": "x", "type": "float", "default": -0.05,
             "min": -1.0, "max": 1.0, "step": 0.01,
             "label": "安装位置 X（米）",
             "note": "IMU 相对 base_link 的前向偏移，出厂 -0.05（前方为正）"},
            {"key": "y", "type": "float", "default": 0.10,
             "min": -1.0, "max": 1.0, "step": 0.01,
             "label": "安装位置 Y（米）",
             "note": "左向偏移，出厂 0.10（左为正）"},
            {"key": "z", "type": "float", "default": 0.05,
             "min": -1.0, "max": 1.0, "step": 0.01,
             "label": "安装位置 Z（米）",
             "note": "高度偏移，出厂 0.05"},
            {"key": "roll", "type": "float", "default": 3.1416,
             "min": -6.3, "max": 6.3, "step": 0.01,
             "label": "安装横滚 roll（弧度）",
             "note": "出厂 3.1416 表示芯片上下翻转安装"},
            {"key": "pitch", "type": "float", "default": 0.0,
             "min": -6.3, "max": 6.3, "step": 0.01,
             "label": "安装俯仰 pitch（弧度）",
             "note": "出厂 0"},
            {"key": "yaw", "type": "float", "default": 1.5708,
             "min": -6.3, "max": 6.3, "step": 0.01,
             "label": "安装偏航 yaw（弧度）",
             "note": "出厂 1.5708 表示芯片顺时针转了 90° 安装"},
        ],
    },
    {
        "id": "camera",
        "title": "相机与 ArUco 规格",
        "apply": "ArUco 字典/边长保存后立即热生效；安装位姿需重启 ArUco 子系统",
        "params": [
            {"key": "x", "type": "float", "default": 0.12,
             "min": -1.0, "max": 1.0, "step": 0.01,
             "label": "相机安装位置 X（米）",
             "note": "相机相对 base_footprint 的前向偏移，出厂 0.12"},
            {"key": "y", "type": "float", "default": 0.0,
             "min": -1.0, "max": 1.0, "step": 0.01,
             "label": "相机安装位置 Y（米）",
             "note": "左向偏移"},
            {"key": "z", "type": "float", "default": 0.25,
             "min": -1.0, "max": 1.0, "step": 0.01,
             "label": "相机安装位置 Z（米）",
             "note": "高度，出厂 0.25"},
            {"key": "yaw", "type": "float", "default": 0.0,
             "min": -3.2, "max": 3.2, "step": 0.01,
             "label": "相机安装偏航（弧度）",
             "note": "绕竖直轴的偏转角，0=光轴朝正前方；斜装时按实际角度填"},
            {"key": "dictionary", "type": "enum", "default": "DICT_6X6_1000",
             "options": ["DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250",
                         "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250",
                         "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250",
                         "DICT_6X6_1000", "DICT_7X7_50", "DICT_7X7_100",
                         "DICT_7X7_250"],
             "label": "ArUco 字典",
             "note": "必须与打印的标志位字典一致；保存后写入 markers.json 热生效"},
            {"key": "marker_size", "type": "float", "default": 0.07,
             "min": 0.01, "max": 1.0, "step": 0.005,
             "label": "ArUco 标志位边长（米）",
             "note": "务必实测打印边长；差 1cm 距离解算约偏 6%。保存后热生效"},
        ],
    },
    {
        "id": "slam_toolbox",
        "title": "SLAM Toolbox 建图参数",
        "apply": "保存后重启建图生效（重启会丢弃未保存的建图进度，先保存地图）",
        "params": [
            {"key": "resolution", "type": "float", "default": 0.10,
             "min": 0.02, "max": 0.2, "step": 0.01,
             "label": "地图分辨率（米/格）",
             "note": "越小越精细越吃内存/CPU；建议与 Nav2 代价地图一致（0.05~0.10）"},
            {"key": "min_laser_range", "type": "float", "default": 0.1,
             "min": 0.02, "max": 2.0, "step": 0.01,
             "label": "激光最小距离（米）",
             "note": "与雷达驱动的 min_range 保持一致，忽略盲区点"},
            {"key": "max_laser_range", "type": "float", "default": 20.0,
             "min": 0.5, "max": 30.0, "step": 0.5,
             "label": "激光最大距离（米）",
             "note": "STL06N 实测约 12m；远场噪声多时下调到 12.0，回环质量更好"},
            {"key": "minimum_travel_distance", "type": "float", "default": 0.15,
             "min": 0.02, "max": 2.0, "step": 0.01,
             "label": "新增帧的最小平移（米）",
             "note": "越小建图越密、回环越好，但更吃 CPU；小空间建议 0.1~0.2"},
            {"key": "minimum_travel_heading", "type": "float", "default": 0.2,
             "min": 0.02, "max": 3.0, "step": 0.01,
             "label": "新增帧的最小转角（弧度）",
             "note": "小空间建议 0.1~0.3"},
            {"key": "map_update_interval", "type": "float", "default": 2.0,
             "min": 0.5, "max": 10.0, "step": 0.5,
             "label": "地图刷新周期（秒）",
             "note": "/map 栅格刷新间隔；调小 Web 端看到建图更实时，更耗 CPU"},
            {"key": "loop_match_minimum_chain_size", "type": "int", "default": 5,
             "min": 2, "max": 30, "step": 1,
             "label": "回环最小链长",
             "note": "默认 10；小空间调小（如 5）更容易触发回环闭合"},
            {"key": "loop_search_maximum_distance", "type": "float", "default": 3.0,
             "min": 0.5, "max": 20.0, "step": 0.5,
             "label": "回环搜索最大距离（米）",
             "note": "在此范围内寻找历史帧做回环；大场景可调大"},
            {"key": "transform_publish_period", "type": "float", "default": 0.02,
             "min": 0.005, "max": 0.2, "step": 0.005,
             "label": "map->odom TF 发布周期（秒）",
             "note": "0 则永不发布，AMCL 交接会失效；保持 0.02 即可"},
            {"key": "use_scan_matching", "type": "bool", "default": True,
             "label": "启用扫描匹配",
             "note": "关闭后退化为纯里程计建图（不推荐，除非调试里程计）"},
        ],
    },
    {
        "id": "cartographer",
        "title": "Cartographer 建图参数",
        "apply": "保存后重启建图生效（重启会丢弃未保存的建图进度，先保存地图）",
        "params": [
            {"key": "min_range", "type": "float", "default": 0.1,
             "min": 0.02, "max": 2.0, "step": 0.01,
             "label": "激光最小距离（米）",
             "note": "与雷达驱动 min_range 保持一致"},
            {"key": "max_range", "type": "float", "default": 20.0,
             "min": 0.5, "max": 30.0, "step": 0.5,
             "label": "激光最大距离（米）",
             "note": "远场噪声多时可下调到 12.0"},
            {"key": "missing_data_ray_length", "type": "float", "default": 0.5,
             "min": 0.1, "max": 5.0, "step": 0.1,
             "label": "无效数据射线长度（米）",
             "note": "无回波时按该长度做 free-space 画线"},
            {"key": "use_imu_data", "type": "bool", "default": True,
             "label": "使用 IMU 数据",
             "note": "当前底盘 IMU 标定偏大/噪声，实测关闭更稳定；改回 true 需重测"},
            {"key": "use_online_correlative_scan_matching", "type": "bool", "default": True,
             "label": "启用 CSM 实时匹配",
             "note": "里程计漂移大时建议开启，能显著改善局部建图一致性"},
            {"key": "motion_filter_max_angle_deg", "type": "float", "default": 0.03,
             "min": 0.01, "max": 45.0, "step": 0.01,
             "label": "运动滤波最小转角（度）",
             "note": "两帧间转角小于该值不新增节点；越小采样越密越吃 CPU。"
                     "当前文件值即 0.03（度），偏小；常规建议 1~5"},
            {"key": "optimize_every_n_nodes", "type": "int", "default": 20,
             "min": 5, "max": 200, "step": 5,
             "label": "全局优化间隔（节点数）",
             "note": "每新增多少节点做一次后端全局优化；越小回环越快、CPU 越高"},
            {"key": "constraint_min_score", "type": "float", "default": 0.5,
             "min": 0.1, "max": 0.95, "step": 0.05,
             "label": "回环约束最小得分",
             "note": "越大回环越严格（误闭合少），太小容易错误闭合（0.4~0.7）"},
            {"key": "global_localization_min_score", "type": "float", "default": 0.5,
             "min": 0.1, "max": 0.95, "step": 0.05,
             "label": "全局定位最小得分",
             "note": "全局重定位（找回起点）时的匹配门槛"},
        ],
    },
    {
        "id": "nav2",
        "title": "Nav2 导航参数（含 AMCL）",
        "apply": "保存后重启导航生效；重启后需重新设定位姿",
        "params": [
            # ---- AMCL 定位 ----
            {"key": "min_particles", "type": "int", "default": 400,
             "min": 50, "max": 5000, "step": 50, "group": "AMCL 定位",
             "label": "最小粒子数",
             "note": "粒子越多定位越稳越吃 CPU"},
            {"key": "max_particles", "type": "int", "default": 1500,
             "min": 100, "max": 10000, "step": 100, "group": "AMCL 定位",
             "label": "最大粒子数",
             "note": "RDK X5 上 1500~3000 都可；调大定位抖动更小"},
            {"key": "update_min_d", "type": "float", "default": 0.25,
             "min": 0.02, "max": 2.0, "step": 0.01, "group": "AMCL 定位",
             "label": "滤波更新最小平移（米）",
             "note": "移动超过该距离才做一次粒子滤波更新；越小越灵敏越耗 CPU"},
            {"key": "update_min_a", "type": "float", "default": 0.2,
             "min": 0.02, "max": 3.0, "step": 0.01, "group": "AMCL 定位",
             "label": "滤波更新最小转角（弧度）",
             "note": "原地旋转场景可调小"},
            {"key": "laser_max_range", "type": "float", "default": 20.0,
             "min": 1.0, "max": 30.0, "step": 0.5, "group": "AMCL 定位",
             "label": "AMCL 激光最大距离（米）",
             "note": "建议与雷达驱动/建图参数一致"},
            {"key": "transform_tolerance", "type": "float", "default": 0.3,
             "min": 0.05, "max": 2.0, "step": 0.05, "group": "AMCL 定位",
             "label": "TF 容忍延迟（秒）",
             "note": "map->odom 时间戳允许的延迟；TF 报老（extrapolation）时可调大"},
            {"key": "alpha1", "type": "float", "default": 0.2,
             "min": 0.0, "max": 2.0, "step": 0.05, "group": "AMCL 定位",
             "label": "运动噪声 alpha1",
             "note": "旋转→旋转噪声；里程计旋转抖动大时调大"},
            {"key": "alpha2", "type": "float", "default": 0.2,
             "min": 0.0, "max": 2.0, "step": 0.05, "group": "AMCL 定位",
             "label": "运动噪声 alpha2",
             "note": "平移→旋转噪声"},
            {"key": "alpha3", "type": "float", "default": 0.2,
             "min": 0.0, "max": 2.0, "step": 0.05, "group": "AMCL 定位",
             "label": "运动噪声 alpha3",
             "note": "平移→平移噪声；轮子打滑/速度标定不准时调大"},
            {"key": "alpha4", "type": "float", "default": 0.2,
             "min": 0.0, "max": 2.0, "step": 0.05, "group": "AMCL 定位",
             "label": "运动噪声 alpha4",
             "note": "旋转→平移噪声"},
            # ---- 控制器/速度 ----
            {"key": "controller_frequency", "type": "float", "default": 10.0,
             "min": 1.0, "max": 30.0, "step": 1.0, "group": "控制器与速度",
             "label": "控制频率（Hz）",
             "note": "DWB 规划+发送 cmd_vel 的频率；CPU 紧张可降到 5"},
            {"key": "max_vel_x", "type": "float", "default": 0.22,
             "min": 0.05, "max": 1.5, "step": 0.01, "group": "控制器与速度",
             "label": "最大前进速度（米/秒）",
             "note": "上限 1.5 即底盘固件限幅（originbot_base kMaxLinearX），再大无效。"
                     "配送场景 0.2~0.35 比较稳；提速同时加大加速度"},
            {"key": "max_vel_theta", "type": "float", "default": 0.8,
             "min": 0.1, "max": 5.0, "step": 0.05, "group": "控制器与速度",
             "label": "最大旋转速度（弧度/秒）",
             "note": "上限 5.0 即底盘固件限幅（kMaxAngularZ）。1.0 以内比较稳"},
            {"key": "acc_lim_x", "type": "float", "default": 1.0,
             "min": 0.1, "max": 10.0, "step": 0.05, "group": "控制器与速度",
             "label": "前进加速度（米/秒²）",
             "note": "调到 10 相当于不再限制（底盘本身也到不了）；托盘有餐品时调小防洒"},
            {"key": "acc_lim_theta", "type": "float", "default": 0.8,
             "min": 0.1, "max": 10.0, "step": 0.05, "group": "控制器与速度",
             "label": "旋转加速度（弧度/秒²）",
             "note": "调小转弯更柔和"},
            {"key": "decel_lim_x", "type": "float", "default": -1.0,
             "min": -10.0, "max": -0.1, "step": 0.05, "group": "控制器与速度",
             "label": "前进减速度（米/秒²，负值）",
             "note": "绝对值越大刹得越急；-10 相当于不再限制"},
            {"key": "decel_lim_theta", "type": "float", "default": -0.8,
             "min": -10.0, "max": -0.1, "step": 0.05, "group": "控制器与速度",
             "label": "旋转减速度（弧度/秒²，负值）",
             "note": "保持与旋转加速度对称"},
            {"key": "sim_time", "type": "float", "default": 1.5,
             "min": 0.5, "max": 5.0, "step": 0.1, "group": "控制器与速度",
             "label": "轨迹前瞻时长（秒）",
             "note": "DWB 采样轨迹的时间窗；大了绕障远虑、小了反应灵活"},
            {"key": "vx_samples", "type": "int", "default": 20,
             "min": 1, "max": 40, "step": 1, "group": "控制器与速度",
             "label": "线速度采样数",
             "note": "越多越精细越耗 CPU"},
            {"key": "vtheta_samples", "type": "int", "default": 40,
             "min": 1, "max": 80, "step": 1, "group": "控制器与速度",
             "label": "角速度采样数",
             "note": "同上"},
            {"key": "progress_movement_radius", "type": "float", "default": 0.5,
             "min": 0.05, "max": 2.0, "step": 0.05, "group": "控制器与速度",
             "label": "进度检查最小位移（米）",
             "note": "时间窗口内位移小于该值即判为\"卡住\"并中止导航。"
                     "到点前 DWB 会先原地转向对准朝向（此期间无位移），"
                     "原底包值 0.25 配 5 秒窗口过于严格，会把正常对准误判为卡死导致状态码 6"},
            {"key": "progress_movement_time", "type": "float", "default": 15.0,
             "min": 1.0, "max": 60.0, "step": 1.0, "group": "控制器与速度",
             "label": "进度检查时间窗口（秒）",
             "note": "与上一项配合判断是否卡住；15 秒可容纳到点前的转向/微调"},
            {"key": "failure_tolerance", "type": "float", "default": 1.0,
             "min": 0.0, "max": 10.0, "step": 0.1, "group": "控制器与速度",
             "label": "控制器失败容忍时间（秒）",
             "note": "控制器短暂算不出有效速度时的容忍时长，超过才中止任务；"
                     "原底包 0.3 秒太敏感"},
            {"key": "xy_goal_tolerance", "type": "float", "default": 0.25,
             "min": 0.02, "max": 1.0, "step": 0.01, "group": "控制器与速度",
             "label": "到达点位置容差（米）",
             "note": "到站判定半径；餐台对接可调小，但太小会反复调整"},
            {"key": "yaw_goal_tolerance", "type": "float", "default": 0.25,
             "min": 0.02, "max": 1.5, "step": 0.01, "group": "控制器与速度",
             "label": "到达点朝向容差（弧度）",
             "note": "约 14°；对接要求朝向准时调小"},
            # ---- 代价地图 ----
            {"key": "robot_radius", "type": "float", "default": 0.2,
             "min": 0.05, "max": 1.0, "step": 0.01, "group": "代价地图",
             "label": "车身半径（米）",
             "note": "按整车外接圆填；偏大绕障保守，偏小容易剐蹭"},
            {"key": "inflation_radius", "type": "float", "default": 0.25,
             "min": 0.05, "max": 2.0, "step": 0.01, "group": "代价地图",
             "label": "膨胀半径（米）",
             "note": "障碍物向外的代价衰减距离；窄通道会过不去时可调小"},
            {"key": "cost_scaling_factor", "type": "float", "default": 3.0,
             "min": 0.5, "max": 10.0, "step": 0.1, "group": "代价地图",
             "label": "膨胀衰减系数",
             "note": "越大代价衰减越快、越贴近障碍走；越小越远离障碍"},
            {"key": "costmap_resolution", "type": "float", "default": 0.05,
             "min": 0.02, "max": 0.2, "step": 0.01, "group": "代价地图",
             "label": "代价地图分辨率（米/格）",
             "note": "建议与建图分辨率一致"},
            {"key": "local_update_frequency", "type": "float", "default": 8.0,
             "min": 1.0, "max": 30.0, "step": 0.5, "group": "代价地图",
             "label": "局部代价地图刷新频率（Hz）",
             "note": "CPU 紧张可降到 5"},
            {"key": "global_update_frequency", "type": "float", "default": 2.0,
             "min": 0.2, "max": 10.0, "step": 0.1, "group": "代价地图",
             "label": "全局代价地图刷新频率（Hz）",
             "note": "静态场景 1~2 足够"},
            {"key": "local_width", "type": "int", "default": 4,
             "min": 2, "max": 15, "step": 1, "group": "代价地图",
             "label": "局部代价地图宽度（米）",
             "note": "滚动窗口边长；Nav2 只接受整数米，填小数会被自动取整"},
            {"key": "local_height", "type": "int", "default": 4,
             "min": 2, "max": 15, "step": 1, "group": "代价地图",
             "label": "局部代价地图高度（米）",
             "note": "同上"},
            {"key": "obstacle_max_range", "type": "float", "default": 2.0,
             "min": 0.5, "max": 12.0, "step": 0.5, "group": "代价地图",
             "label": "障碍物标记最大距离（米，局部）",
             "note": "超过该距离的回波不作为动态障碍；全局为 10.0"},
            {"key": "global_obstacle_max_range", "type": "float", "default": 10.0,
             "min": 0.5, "max": 20.0, "step": 0.5, "group": "代价地图",
             "label": "障碍物标记最大距离（米，全局）",
             "note": "全局代价地图的动态障碍标记范围"},
        ],
    },
    {
        "id": "chassis",
        "title": "底盘速度标定",
        "apply": "保存后需重启底盘生效；开机自启动自动加载",
        "params": [
            # 这两个系数用于修正底盘上报的轮式速度：odom 里积分用的是
            # correct_factor_* × 固件原始速度。系数偏小→里程计走得比实际少。
            # 历史遗留：老版本只在 robot.launch.py 里写死默认值，且
            # originbot_bringup/originbot.launch.py 不转发这两个 launch 参数，
            # 导致从这里之外的任何地方都无法覆盖（改了不生效）。现已接入本页。
            # 标定工具 odom_calib.py 的"旧值"优先取运行中节点的活参数
            # （ros2 param get），读不到才回落本 json，并在输出里标明来源——
            # 因为两者可能不一致（例如有人用 ros2 param set 临时改过）。
            {"key": "correct_factor_vx", "type": "float", "default": 1.00715,
             "min": 0.5, "max": 2.0, "step": 0.0001,
             "label": "线速度标定系数 vx",
             "note": "直线里程修正。用 odom_calib.py 一键自动测量：小车自行直行 1m，"
                     "用雷达 /scan 算出真实位移（雷达不可用时回退钢尺人工输入），"
                     "系数 × (真实距离 / 里程计距离)。该工具只打印建议值，不写配置"},
            {"key": "correct_factor_vth", "type": "float", "default": 1.016434,
             "min": 0.5, "max": 2.0, "step": 0.0001,
             "label": "角速度标定系数 vth",
             "note": "原地旋转角度修正。用 odom_calib.py --rotate 自动测量：小车自行"
                     "原地转整圈，雷达算出真实转角（独立于里程计）。"
                     "系数 × (真实转角 / 里程计转角)。环境过于对称（如正方形房间）时"
                     "雷达真值有歧义，工具会判为不可靠并让你人工确认"},
        ],
    },
    {
        "id": "ekf",
        "title": "EKF 里程计融合（robot_localization）",
        "apply": "保存后重启底盘+定位生效；开启/关闭后建议重新设定位姿",
        "params": [
            {"key": "enabled", "type": "bool", "default": False, "skip_yaml": True,
             "label": "启用 EKF 融合",
             "note": "开启后由 robot_localization 融合轮式里程计+IMU 角速度，"
                     "接管 odom→base_footprint TF（底盘停发该 TF，避免冲突）；"
                     "关闭则回到纯轮式里程计。切换后需重启底盘与定位，导航需重新设位姿"},
            {"key": "frequency", "type": "float", "default": 30.0,
             "min": 5.0, "max": 100.0, "step": 1.0,
             "label": "滤波输出频率（Hz）",
             "note": "/odometry/filtered 的输出频率"},
            {"key": "sensor_timeout", "type": "float", "default": 0.1,
             "min": 0.02, "max": 2.0, "step": 0.01,
             "label": "传感器超时（秒）",
             "note": "超过该时间无测量则做纯预测；漂移快时可调大"},
            {"key": "two_d_mode", "type": "bool", "default": True,
             "label": "二维模式",
             "note": "平面机器人保持开启，忽略 Z/俯仰等小扰动"},
            {"key": "odom0_pose_rejection_threshold", "type": "float", "default": 20.0,
             "min": 0.5, "max": 100.0, "step": 0.5,
             "label": "里程计位姿马氏距离阈值",
             "note": "卡方检验剔除里程计离群值；调小更激进地拒绝坏数据"},
            {"key": "odom0_twist_rejection_threshold", "type": "float", "default": 1.542,
             "min": 0.5, "max": 100.0, "step": 0.1,
             "label": "里程计速度马氏距离阈值",
             "note": "剔除速度突刺（打滑）"},
            {"key": "imu0_pose_rejection_threshold", "type": "float", "default": 20.0,
             "min": 0.5, "max": 100.0, "step": 0.5,
             "label": "IMU 姿态马氏距离阈值",
             "note": "IMU 标定偏大时该值偏小会频繁拒绝测量；先查 IMU 再调"},
            {"key": "imu0_twist_rejection_threshold", "type": "float", "default": 1.542,
             "min": 0.5, "max": 100.0, "step": 0.1,
             "label": "IMU 角速度马氏距离阈值",
             "note": "剔除角速度离群值"},
            {"key": "imu0_linear_acceleration_rejection_threshold", "type": "float",
             "default": 10.0, "min": 0.5, "max": 100.0, "step": 0.5,
             "label": "IMU 加速度马氏距离阈值",
             "note": "剔除加计离群值（振动/碰撞）"},
        ],
    },
    {
        "id": "route",
        "title": "轨道路线导航（路网跟随器）",
        "apply": "保存后需重启路网跟随器生效；开机自启动自动加载",
        "params": [
            # ---- 速度与加速度 ----
            {"key": "max_vel_x", "type": "float", "default": 0.22,
             "min": 0.05, "max": 1.5, "step": 0.01, "group": "速度与加速度",
             "label": "巡航线速度（米/秒）",
             "note": "沿路网行驶的巡航速度上限；转弯/避障/到点前会自动再降速。"
                     "上限 1.5 即底盘固件限幅（kMaxLinearX），再大无效"},
            {"key": "max_vel_theta", "type": "float", "default": 0.8,
             "min": 0.2, "max": 5.0, "step": 0.05, "group": "速度与加速度",
             "label": "最大角速度（弧度/秒）",
             "note": "原地转向与循线转向的角速度上限；上限 5.0 即固件限幅（kMaxAngularZ）"},
            {"key": "v_accel", "type": "float", "default": 0.5,
             "min": 0.1, "max": 10.0, "step": 0.05, "group": "速度与加速度",
             "label": "线加速度（米/秒²）",
             "note": "速度斜率限幅，避免底盘电流冲击；调到 10 相当于不再限制"},
            {"key": "w_accel", "type": "float", "default": 2.0,
             "min": 0.5, "max": 20.0, "step": 0.1, "group": "速度与加速度",
             "label": "角加速度（弧度/秒²）",
             "note": "角速度斜率限幅；调到 20 相当于不再限制"},
            # ---- 跟随与到站 ----
            {"key": "lookahead", "type": "float", "default": 0.35,
             "min": 0.15, "max": 0.8, "step": 0.01, "group": "跟随与到站",
             "label": "纯跟踪前视距离（米）",
             "note": "向前看多远取跟踪点。越大切弯越顺、贴线越松；越小跟线越紧、越容易画龙"},
            {"key": "rotate_threshold", "type": "float", "default": 0.9,
             "min": 0.3, "max": 2.0, "step": 0.05, "group": "跟随与到站",
             "label": "原地转向阈值（弧度）",
             "note": "航向偏差超过该值先原地转向再前进（约 51°）；掉头场景必经"},
            {"key": "goal_xy_tolerance", "type": "float", "default": 0.10,
             "min": 0.03, "max": 0.5, "step": 0.01, "group": "跟随与到站",
             "label": "到点距离容差（米）",
             "note": "距目标节点小于该值即认为到点，进入朝向对准阶段"},
            {"key": "goal_yaw_tolerance", "type": "float", "default": 0.10,
             "min": 0.02, "max": 0.5, "step": 0.01, "group": "跟随与到站",
             "label": "到站朝向容差（弧度）",
             "note": "对准误差小于该值判定导航完成（约 5.7°）"},
            {"key": "align_yaw_speed", "type": "float", "default": 0.6,
             "min": 0.1, "max": 5.0, "step": 0.05, "group": "跟随与到站",
             "label": "对准角速度（弧度/秒）",
             "note": "到站后原地转向对准朝向的速度上限；上限 5.0 即固件限幅"},
            {"key": "end_decel", "type": "float", "default": 0.5,
             "min": 0.1, "max": 10.0, "step": 0.05, "group": "跟随与到站",
             "label": "到点减速（米/秒²）",
             "note": "按 v=√(2×减速×剩余距离) 平滑刹停，调大刹车更晚更急；"
                     "调到 10 相当于不再限制"},
            # ---- 前向避障 ----
            {"key": "scan_half_fov_deg", "type": "float", "default": 30.0,
             "min": 10.0, "max": 60.0, "step": 1.0, "group": "前向避障",
             "label": "避障扇区半角（度）",
             "note": "以车头正前为中心的检测扇区。沿墙/窄通道误减速时可调小"},
            {"key": "stop_distance", "type": "float", "default": 0.45,
             "min": 0.2, "max": 1.0, "step": 0.01, "group": "前向避障",
             "label": "停车距离（米）",
             "note": "扇区内障碍小于该值立即停车等待。务必小于减速距离"},
            {"key": "slow_distance", "type": "float", "default": 0.80,
             "min": 0.3, "max": 1.5, "step": 0.01, "group": "前向避障",
             "label": "减速距离（米）",
             "note": "扇区内障碍小于该值开始按比例减速。务必大于停车距离"},
            {"key": "resume_clear_sec", "type": "float", "default": 1.5,
             "min": 0.5, "max": 10.0, "step": 0.1, "group": "前向避障",
             "label": "畅通确认时长（秒）",
             "note": "障碍消除后需持续畅通该时长才恢复行驶（防抖）"},
            {"key": "scan_stale_sec", "type": "float", "default": 3.0,
             "min": 1.0, "max": 10.0, "step": 0.5, "group": "前向避障",
             "label": "激光过期判停（秒）",
             "note": "雷达数据缺失/超过该时长未更新则停车等待，不盲走"},
            # ---- 上车接入 ----
            {"key": "on_ramp_max", "type": "float", "default": 0.60,
             "min": 0.2, "max": 1.5, "step": 0.05, "group": "上车接入",
             "label": "路边投影上车距离（米）",
             "note": "起点向路网边投影，垂距不超过该值直接从边上接入路网"},
            {"key": "nearest_node_max", "type": "float", "default": 1.50,
             "min": 0.5, "max": 3.0, "step": 0.05, "group": "上车接入",
             "label": "最近节点上车距离（米）",
             "note": "投影超限改为就近节点直线接入；超过该值报错要求先挪车"},
        ],
    },
]

# 生成文件位置（相对 ~/.originbot_delivery/config）
GENERATED_DIRNAME = "generated"

# 各参数组 → 生成文件方式
#   launch_json: 生成 JSON，由 launch 文件启动时读取覆盖默认值
#   yaml_merge:  以包内原始 yaml 为底，按 path 深合并后另存
#   lua:         用模板重新生成 lds_2d.lua


def _schema_map():
    return {section["id"]: section for section in SCHEMA}


def _coerce(param, value):
    """按 schema 类型转换并校验一个值，失败抛 ValueError。"""
    ptype = param["type"]
    if ptype == "bool":
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)) and value in (0, 1):
            return bool(value)
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in ("true", "1", "yes", "on"):
                return True
            if lowered in ("false", "0", "no", "off"):
                return False
        raise ValueError(f"{param['label']} 需要布尔值")
    if ptype in ("int", "float"):
        number = float(value)
        if ptype == "int":
            number = int(round(number))
        if "min" in param and number < param["min"]:
            raise ValueError(f"{param['label']} 不能小于 {param['min']}")
        if "max" in param and number > param["max"]:
            raise ValueError(f"{param['label']} 不能大于 {param['max']}")
        return number
    if ptype == "enum":
        text = str(value).strip()
        if text not in param["options"]:
            raise ValueError(f"{param['label']} 只能是 {'/'.join(param['options'])}")
        return text
    return value


class ParamsStore:
    """参数仓库：加载/保存 robot_params.json，并把参数生成为各子系统的配置文件。"""

    def __init__(self, config_dir):
        self.config_dir = Path(config_dir).expanduser()
        self.generated_dir = self.config_dir / GENERATED_DIRNAME
        self.path = self.config_dir / "robot_params.json"
        self.data = self._load()

    # ---------- 存取 ----------

    def _load(self):
        data = {section["id"]: {} for section in SCHEMA}
        if self.path.exists():
            try:
                with open(self.path, "r", encoding="utf-8") as handle:
                    stored = json.load(handle)
                for section_id, values in stored.items():
                    if section_id in data and isinstance(values, dict):
                        data[section_id].update(values)
            except (OSError, ValueError):
                pass
        # 用 schema 补齐缺失项（新版本新增的参数自动获得默认值）；
        # 同时迁移带 legacy 字段的旧键名（键名改过但语义相同，直接丢弃会让
        # 用户已保存的设置悄悄回到默认值）。
        for section in SCHEMA:
            for param in section["params"]:
                key = param["key"]
                legacy = param.get("legacy")
                values = data[section["id"]]
                if key not in values and legacy and legacy in values:
                    values[key] = values.pop(legacy)
                values.setdefault(key, param["default"])
        return data

    def save(self):
        self.config_dir.mkdir(parents=True, exist_ok=True)
        tmp_path = str(self.path) + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(self.data, handle, indent=2, ensure_ascii=False)
        os.replace(tmp_path, self.path)

    def defaults(self, section_id):
        schema = _schema_map()[section_id]
        return {param["key"]: param["default"] for param in schema["params"]}

    def set(self, section_id, values):
        """校验并写入一个参数组，返回 (是否成功, 消息)。"""
        schema = _schema_map().get(section_id)
        if schema is None:
            return False, f"未知参数组：{section_id}"
        if not isinstance(values, dict):
            return False, "参数值必须是对象（{键: 值}）"
        clean = {}
        for key, value in values.items():
            param = next((p for p in schema["params"] if p["key"] == key), None)
            if param is None:
                return False, f"未知参数：{section_id}.{key}"
            try:
                clean[key] = _coerce(param, value)
            except (TypeError, ValueError) as exc:
                return False, str(exc)
        self.data.setdefault(section_id, {}).update(clean)
        self.save()
        return True, f"{schema['title']} 已保存"

    def reset(self, section_id):
        if section_id == "all":
            for section in SCHEMA:
                self.data[section["id"]] = self.defaults(section["id"])
            self.save()
            return True, "已恢复全部默认参数"
        if section_id not in _schema_map():
            return False, f"未知参数组：{section_id}"
        self.data[section_id] = self.defaults(section_id)
        self.save()
        return True, f"{_schema_map()[section_id]['title']} 已恢复默认"

    def describe(self):
        """给 Web 端的完整描述：schema + 当前值。"""
        return {"sections": SCHEMA, "values": copy.deepcopy(self.data)}

    # ---------- 配置文件生成 ----------

    def write_generated(self):
        """把当前参数物化为各子系统消费的配置文件。幂等，可反复调用。"""
        self.generated_dir.mkdir(parents=True, exist_ok=True)
        self._write_launch_json("lidar.json", "lidar")
        self._write_launch_json("imu.json", "imu")
        self._write_launch_json("chassis.json", "chassis")
        self._write_launch_json("camera.json", "camera")
        self._write_launch_json("ekf.json", "ekf")
        self._write_launch_json("route.json", "route")
        self._write_yaml_merge(
            "ekf.yaml", "ekf",
            "originbot_navigation", "config/ekf.yaml",
            "ekf_filter_node.ros__parameters",
        )
        self._write_yaml_merge(
            "slam_toolbox.yaml", "slam_toolbox",
            "originbot_navigation", "config/mapper_params_online_async.yaml",
            "slam_toolbox.ros__parameters",
        )
        self._write_nav2()
        self._write_cartographer_lua()

    def generated_paths(self):
        """delivery_server 注入 launch 命令需要的路径。"""
        return {
            "slam_params_file": str(self.generated_dir / "slam_toolbox.yaml"),
            "cartographer_config_dir": str(self.generated_dir / "cartographer"),
            "nav2_params_file": str(self.generated_dir / "nav2_params.yaml"),
        }

    def _write_launch_json(self, filename, section_id):
        payload = {
            param["key"]: self.data[section_id][param["key"]]
            for param in _schema_map()[section_id]["params"]
        }
        path = self.generated_dir / filename
        tmp_path = str(path) + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, ensure_ascii=False)
        os.replace(tmp_path, path)

    @staticmethod
    def _match_base_type(existing, value):
        """把新值转换成基配置文件里已有值的类型。

        Nav2 等节点对参数类型强校验（如 local_costmap 的 width/height 在底层
        声明为整数，写成 4.0 会让组件加载失败），因此覆盖已有键时保形：
        基文件是 int 就四舍五入成 int，是 float 就写 float；bool 要先于 int 判断。
        """
        if isinstance(existing, bool):
            if isinstance(value, (int, float)):
                return bool(value)
            return value
        if isinstance(existing, int):
            if isinstance(value, (int, float)):
                return int(round(value))
            return value
        if isinstance(existing, float):
            if isinstance(value, (int, float)):
                return float(value)
            return value
        return value

    @classmethod
    def _deep_set(cls, root, dotted_path, value):
        keys = dotted_path.split(".")
        node = root
        for key in keys[:-1]:
            node = node.setdefault(key, {})
        leaf = keys[-1]
        node[leaf] = cls._match_base_type(node.get(leaf), value)

    def _write_yaml_merge(self, out_name, section_id, pkg, base_relpath, prefix):
        """以包内原始配置为底，仅覆盖 Web 端开放的参数键，保留其余全部原样。

        带 skip_yaml 标记的参数（如 ekf.enabled 开关）只进 ekf.json 供 launch
        读取，不写入节点参数文件。
        """
        from ament_index_python.packages import get_package_share_directory

        base_path = Path(get_package_share_directory(pkg)) / base_relpath
        with open(base_path, "r", encoding="utf-8") as handle:
            doc = yaml.safe_load(handle) or {}
        for param in _schema_map()[section_id]["params"]:
            if param.get("skip_yaml"):
                continue
            key_path = param.get("path") or f"{param['key']}"
            self._deep_set(doc, f"{prefix}.{key_path}", self.data[section_id][param["key"]])
        out_path = self.generated_dir / out_name
        tmp_path = str(out_path) + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            yaml.safe_dump(doc, handle, allow_unicode=True, sort_keys=False)
        os.replace(tmp_path, out_path)

    def _nav2_paths(self):
        """nav2 参数键 → 生成 yaml 中的点分路径。"""
        amcl = "amcl.ros__parameters"
        ctrl = "controller_server.ros__parameters"
        follow = f"{ctrl}.FollowPath"
        goal = f"{ctrl}.general_goal_checker"
        local = "local_costmap.local_costmap.ros__parameters"
        glob = "global_costmap.global_costmap.ros__parameters"
        return {
            "min_particles": f"{amcl}.min_particles",
            "max_particles": f"{amcl}.max_particles",
            "update_min_d": f"{amcl}.update_min_d",
            "update_min_a": f"{amcl}.update_min_a",
            "laser_max_range": f"{amcl}.laser_max_range",
            "transform_tolerance": f"{amcl}.transform_tolerance",
            "alpha1": f"{amcl}.alpha1",
            "alpha2": f"{amcl}.alpha2",
            "alpha3": f"{amcl}.alpha3",
            "alpha4": f"{amcl}.alpha4",
            "controller_frequency": f"{ctrl}.controller_frequency",
            "xy_goal_tolerance": f"{goal}.xy_goal_tolerance",
            "yaw_goal_tolerance": f"{goal}.yaw_goal_tolerance",
            "max_vel_x": f"{follow}.max_vel_x",
            "max_vel_theta": f"{follow}.max_vel_theta",
            "acc_lim_x": f"{follow}.acc_lim_x",
            "acc_lim_theta": f"{follow}.acc_lim_theta",
            "decel_lim_x": f"{follow}.decel_lim_x",
            "decel_lim_theta": f"{follow}.decel_lim_theta",
            "sim_time": f"{follow}.sim_time",
            "vx_samples": f"{follow}.vx_samples",
            "vtheta_samples": f"{follow}.vtheta_samples",
            "progress_movement_radius": f"{ctrl}.progress_checker.required_movement_radius",
            "progress_movement_time": f"{ctrl}.progress_checker.movement_time_allowance",
            "failure_tolerance": f"{ctrl}.failure_tolerance",
            "robot_radius": None,  # 局部+全局两处，单独处理
            "inflation_radius": None,
            "cost_scaling_factor": None,
            "costmap_resolution": None,
            "local_update_frequency": f"{local}.update_frequency",
            "global_update_frequency": f"{glob}.update_frequency",
            "local_width": f"{local}.width",
            "local_height": f"{local}.height",
            "obstacle_max_range": f"{local}.obstacle_layer.scan.obstacle_max_range",
            "global_obstacle_max_range": f"{glob}.obstacle_layer.scan.obstacle_max_range",
        }

    def _write_nav2(self):
        from ament_index_python.packages import get_package_share_directory

        base_path = (
            Path(get_package_share_directory("originbot_navigation"))
            / "param" / "originbot_nav2.yaml"
        )
        with open(base_path, "r", encoding="utf-8") as handle:
            doc = yaml.safe_load(handle) or {}
        paths = self._nav2_paths()
        values = self.data["nav2"]
        for key, dotted in paths.items():
            if dotted is None:
                continue
            self._deep_set(doc, dotted, values[key])
        # 差速底盘合成速度上限必须与 max_vel_x 一致；模板里写死的 0.22
        # 会把实际平移速度压到 max_vel_x 以下，表现为小车整体偏慢
        self._deep_set(
            doc,
            "controller_server.ros__parameters.FollowPath.max_speed_xy",
            values["max_vel_x"],
        )
        # 双代价地图共享的键：局部与全局同时写入
        shared = {
            "robot_radius": ("robot_radius", "robot_radius"),
            "costmap_resolution": ("resolution", "resolution"),
            "inflation_radius": ("inflation_layer.inflation_radius",) * 2,
            "cost_scaling_factor": ("inflation_layer.cost_scaling_factor",) * 2,
        }
        for key, (local_tail, global_tail) in shared.items():
            self._deep_set(doc, f"local_costmap.local_costmap.ros__parameters.{local_tail}", values[key])
            self._deep_set(doc, f"global_costmap.global_costmap.ros__parameters.{global_tail}", values[key])
        out_path = self.generated_dir / "nav2_params.yaml"
        tmp_path = str(out_path) + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            yaml.safe_dump(doc, handle, allow_unicode=True, sort_keys=False)
        os.replace(tmp_path, out_path)

    CARTOGRAPHER_LUA_TEMPLATE = """-- 由 Web 端参数配置页生成，请勿手工修改；再次保存参数会覆盖本文件。
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 1e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = %(min_range)s
TRAJECTORY_BUILDER_2D.max_range = %(max_range)s
TRAJECTORY_BUILDER_2D.missing_data_ray_length = %(missing_data_ray_length)s
TRAJECTORY_BUILDER_2D.use_imu_data = %(use_imu_data)s
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = %(use_online_correlative_scan_matching)s
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(%(motion_filter_max_angle_deg)s)

POSE_GRAPH.constraint_builder.min_score = %(constraint_min_score)s
POSE_GRAPH.constraint_builder.global_localization_min_score = %(global_localization_min_score)s
POSE_GRAPH.optimize_every_n_nodes = %(optimize_every_n_nodes)s

return options
"""

    def _write_cartographer_lua(self):
        values = self.data["cartographer"]

        def lua_bool(flag):
            return "true" if flag else "false"

        def lua_number(number):
            text = f"{float(number):.6f}".rstrip("0").rstrip(".")
            return text if text else "0"

        payload = {
            "min_range": lua_number(values["min_range"]),
            "max_range": lua_number(values["max_range"]),
            "missing_data_ray_length": lua_number(values["missing_data_ray_length"]),
            "use_imu_data": lua_bool(values["use_imu_data"]),
            "use_online_correlative_scan_matching":
                lua_bool(values["use_online_correlative_scan_matching"]),
            # 与现有配置约定一致：该键以“度”为单位包进 math.rad()
            "motion_filter_max_angle_deg": lua_number(values["motion_filter_max_angle_deg"]),
            "constraint_min_score": lua_number(values["constraint_min_score"]),
            "global_localization_min_score": lua_number(values["global_localization_min_score"]),
            "optimize_every_n_nodes": str(int(values["optimize_every_n_nodes"])),
        }
        content = self.CARTOGRAPHER_LUA_TEMPLATE % payload
        lua_dir = self.generated_dir / "cartographer"
        lua_dir.mkdir(parents=True, exist_ok=True)
        out_path = lua_dir / "lds_2d.lua"
        tmp_path = str(out_path) + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            handle.write(content)
        os.replace(tmp_path, out_path)
