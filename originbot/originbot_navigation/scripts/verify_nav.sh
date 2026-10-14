#!/usr/bin/env bash
# =============================================================================
# OriginBot 导航调优验证脚本
# -----------------------------------------------------------------------------
# 功能：
#   1. 检查关键参数是否生效
#   2. 观察全局路径与障碍的距离（需配合 RViz2）
#   3. 观察实际速度曲线（急停距离、转弯是否流畅）
#   4. 定位精度验证（对比实际偏差）
#   5. "Stuck"/无进展 排查
#
# 使用方法（在机器人端执行）：
#   source install/setup.bash
#   bash scripts/verify_nav.sh           # 运行全部检查（交互式菜单）
#   bash scripts/verify_nav.sh params    # 只检查参数
#   bash scripts/verify_nav.sh vel       # 只监控速度曲线
#   bash scripts/verify_nav.sh amcl      # 只查看定位
#   bash scripts/verify_nav.sh stuck     # 只做 Stuck 排查
#
# 提示：ros2 topic echo 为阻塞式命令，脚本默认用 timeout 5 限制时长，
#       避免长时间卡住终端。可修改 ECHO_TIMEOUT 调整。
# =============================================================================

set -e

ECHO_TIMEOUT=${ECHO_TIMEOUT:-5}   # topic echo 单次最长运行秒数
CONTROLLER_NODE=${CONTROLLER_NODE:-/controller_server}

# -----------------------------------------------------------------------------
# 1. 检查参数是否生效
#    参数修改后，对比此处输出与 yaml 文件中的设定值是否一致。
# -----------------------------------------------------------------------------
check_params() {
    echo "========== [1/5] 检查关键参数是否生效 =========="
    echo ">>> 速度上限 / 加速度 / 刹车 / 停止判定阈值（应与你改的 yaml 一致）:"
    ros2 param get "$CONTROLLER_NODE" FollowPath.max_vel_x
    ros2 param get "$CONTROLLER_NODE" FollowPath.max_vel_theta
    ros2 param get "$CONTROLLER_NODE" FollowPath.acc_lim_x
    ros2 param get "$CONTROLLER_NODE" FollowPath.decel_lim_x
    ros2 param get "$CONTROLLER_NODE" FollowPath.trans_stopped_velocity
    echo ">>> 规划器类型:"
    ros2 param get "$CONTROLLER_NODE" FollowPath.plugin
    echo ""
}

# -----------------------------------------------------------------------------
# 2. 观察全局路径是否贴障碍
#    此步无法用命令行完成，需要在 RViz2 中人工检查：
#      - Add -> By topic -> /plan (Global Plan)
#      - 路径与障碍的最小距离应 > inflation_radius（如 0.25m）
#      - 若贴障碍走，说明 inflation_radius 太小或 cost_scaling_factor 太大
# -----------------------------------------------------------------------------
check_global_plan() {
    echo "========== [2/5] 观察全局路径是否贴障碍 =========="
    echo ">>> 此步需在 RViz2 中人工检查："
    echo "    Add -> By topic -> /plan（Global Plan）"
    echo "    路径与障碍的最小距离应 > inflation_radius（当前 0.25m）"
    echo "    若贴障碍走：增大 inflation_radius 或减小 cost_scaling_factor"
    echo ""
}

# -----------------------------------------------------------------------------
# 3. 观察实际速度曲线（急停距离、转弯是否流畅）
#    让机器人走一段直线后急停，观察：
#      - linear.x 是否平滑上升/下降（抖动说明加速度设置不合理）
#      - 急停是否干脆（decel_lim_x 生效）
#    再走一段弧线，观察：
#      - angular.z 是否平滑变化（抖动说明 acc_lim_theta 过小或过大）
# -----------------------------------------------------------------------------
check_velocity() {
    echo "========== [3/5] 观察实际速度曲线（${ECHO_TIMEOUT}s） =========="
    echo ">>> 线速度 /cmd_vel linear.x:"
    timeout "$ECHO_TIMEOUT" ros2 topic echo /cmd_vel --field linear.x || true
    echo ""
    echo ">>> 角速度 /cmd_vel angular.z:"
    timeout "$ECHO_TIMEOUT" ros2 topic echo /cmd_vel --field angular.z || true
    echo ""
}

# -----------------------------------------------------------------------------
# 4. 定位精度验证
#    让机器人走到一个已知坐标点（如地图原点附近），对比：
#      - /amcl_pose 输出的位姿
#      - 实际测量位置
#    偏差大于 5cm / 2° 时说明定位需要调优（AMCL 粒子数、EKF 协方差）
# -----------------------------------------------------------------------------
check_amcl() {
    echo "========== [4/5] 定位精度验证（${ECHO_TIMEOUT}s） =========="
    echo ">>> 让机器人停在已知点，对比 amcl_pose 与实际位置:"
    timeout "$ECHO_TIMEOUT" ros2 topic echo /amcl_pose --field pose.pose || true
    echo ""
}

# -----------------------------------------------------------------------------
# 5. "Stuck" / 无进展 排查
#    - 检查 /scan 话题是否有数据
#    - 检查 /scan 是否进入 local_costmap（rqt_graph 查看数据流）
#    - 查看 local_costmap 地图内容（costmap_raw 中 254/255 为致命障碍）
# -----------------------------------------------------------------------------
check_stuck() {
    echo "========== [5/5] Stuck / 无进展 排查 =========="
    echo ">>> 1) 检查激光雷达数据是否正常（${ECHO_TIMEOUT}s）:"
    timeout "$ECHO_TIMEOUT" ros2 topic echo /scan --field ranges --once || true
    echo ""
    echo ">>> 2) 检查数据流（需人工查看）:"
    echo "    rqt_graph  # 确认 /scan -> local_costmap 的连接存在"
    echo ""
    echo ">>> 3) 查看局部代价图内容（${ECHO_TIMEOUT}s）:"
    echo "    # 254/255 = 致命障碍，0 = 空闲，255 为膨胀层"
    timeout "$ECHO_TIMEOUT" ros2 topic echo /local_costmap/costmap_raw --field data || true
    echo ""
    echo ">>> 4) 若 /scan 无数据：检查激光雷达驱动节点是否正常"
    echo "    ros2 node list | grep -i laser"
    echo ""
}

# -----------------------------------------------------------------------------
# 主菜单
# -----------------------------------------------------------------------------
case "${1:-all}" in
    params) check_params ;;
    plan)   check_global_plan ;;
    vel)    check_velocity ;;
    amcl)   check_amcl ;;
    stuck)  check_stuck ;;
    all)
        check_params
        check_global_plan
        check_velocity
        check_amcl
        check_stuck
        ;;
    *)
        echo "用法: bash $0 [params|plan|vel|amcl|stuck|all]"
        exit 1
        ;;
esac

echo "完成。"
